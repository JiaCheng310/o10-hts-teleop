"""Persistent HTTP bridge for the selected ARX-local O10. No queued trajectories."""
import argparse,json,math,os,secrets,socket,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from usb_bridge import limited_target

class BridgeError(Exception):
    def __init__(self,code,message):super().__init__(message);self.code=code

class Bridge:
    def __init__(self,hand,port,limits):
        self.hand,self.port,self.cfg=hand,port,limits
        self.side=limits['hand']
        if self.side not in ('left','right') or not all(n.startswith('L_' if self.side=='left' else 'R_') for n in limits['joint_names']):
            raise ValueError('Hand configuration mismatch')
        self.lock=threading.Lock();self.tokens={};self.sent=0;self.last_command=None;self.last_error=None
    def token(self):
        now=time.monotonic();self.tokens={k:v for k,v in self.tokens.items() if now-v<.25}
        value=secrets.token_hex(16);self.tokens[value]=now;return value
    def consume(self,value):
        started=self.tokens.pop(value,None)
        if started is None or time.monotonic()-started>.25:raise BridgeError('EXPIRED_TOKEN','Expired or reused command; fetch a fresh state')
        return started
    def read(self):
        errors=self.hand.get_all_error_reports()
        if len(errors)!=10 or any(any(getattr(e,k) for k in ('stalled','overheat','over_current','motor_except','commu_except')) for e in errors):
            raise BridgeError('DEVICE_FAULT','Motor fault or missing fault report')
        angles=list(self.hand.get_all_active_joint_angles())
        limited_target([max(lo,min(hi,q)) for q,(lo,hi) in zip(angles,self.cfg['limits_rad'])],angles,self.cfg['limits_rad'],0)
        return angles
    def snapshot(self,angles):
        return dict(hand=self.side,usb_port=self.port,units='rad',joint_names=self.cfg['joint_names'],angles=angles,
                    allow_motion=True,speed_limit=0,max_lead=0,nonce=self.token(),commands_sent=self.sent,protocol=2)
    def target(self,data):
        if data.get('hand')!=self.side or data.get('joint_names')!=self.cfg['joint_names'] or data.get('enabled') is not True:
            raise BridgeError('IDENTITY_MISMATCH','Hand/order/enabled mismatch')
        issued=self.consume(data.get('nonce'));started=time.monotonic()
        budget=float(data.get('freshness_budget_s',.2))
        if not math.isfinite(budget) or not 0<budget<=.2:raise BridgeError('STALE_TARGET','Invalid target freshness budget')
        actual=self.read()
        command=limited_target(data['angles'],actual,self.cfg['limits_rad'],float('inf'))
        if time.monotonic()-issued>.25 or time.monotonic()-started>budget:
            raise BridgeError('STALE_TARGET','Readback too slow; latest target required')
        self.hand.set_all_active_joint_angles(command);self.sent+=1;self.last_command=command
        result=self.snapshot(actual);result.update(sent=command,driver_ms=(time.monotonic()-started)*1000)
        return result

def validate_usb_identity(port, side):
    serial = os.environ.get('O10_'+side.upper()+'_SERIAL', '')
    if not serial:
        raise RuntimeError('Set O10_'+side.upper()+'_SERIAL in config/devices.env first.')
    expected = Path('/dev/serial/by-id') / ('usb-STMicroelectronics_STM32_Virtual_ComPort_'+serial+'-if00')
    if Path(port) != expected:
        raise RuntimeError('Expected O10 '+side+' port '+str(expected))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',required=True);parser.add_argument('--hand',choices=('left','right'),required=True);parser.add_argument('--http-port',type=int,default=8890);args=parser.parse_args()
    validate_usb_identity(args.port, args.hand)
    from omnihand import OmniHand2025,HandType
    hand=OmniHand2025.create_hand_by_usb(hand_type=getattr(HandType,args.hand.upper()),hand_device_id=OmniHand2025.kDefaultHandDeviceId,uart_port=args.port,baudrate=460800)
    hand.set_request_interval(0);hand.set_frame_recv_timeout(100)
    if not hand.init() or hand.get_vendor_info().dof!=10:raise RuntimeError('O10 initialization failed')
    bridge=Bridge(hand,args.port,json.loads(Path(__file__).with_name(args.hand+'_limits.json').read_text()))
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        def setup(self):
            super().setup();self.connection.settimeout(2);self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        def reply(self,status,data):
            raw=json.dumps(data,allow_nan=False).encode();self.send_response(status)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def handle_request(self,data=None):
            try:
                with bridge.lock:
                    if self.path=='/state' and data is None:result=bridge.snapshot(bridge.read())
                    elif self.path=='/target' and data is not None:result=bridge.target(data)
                    else:raise BridgeError('BAD_ROUTE','Unknown route')
                self.reply(200,result)
            except Exception as e:
                code=getattr(e,'code','DRIVER_ERROR');error=dict(code=code,error=str(e))
                if bridge.last_error!=error:print('BRIDGE_ERROR',json.dumps(error),flush=True);bridge.last_error=error
                self.reply(409 if code in ('EXPIRED_TOKEN','STALE_TARGET') else 503,error)
        def do_GET(self):self.handle_request()
        def do_POST(self):
            try:
                n=int(self.headers.get('Content-Length',0))
                if not 0<n<=4096:raise ValueError('Invalid payload size')
                data=json.loads(self.rfile.read(n))
            except Exception as e:self.reply(400,dict(code='BAD_REQUEST',error=str(e)));return
            self.handle_request(data)
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',args.http_port),Handler);server.daemon_threads=True
    print('ARX O10 '+args.hand.upper()+' '+serial+' ready / protocol 2 / direct targets',flush=True)
    server.serve_forever()
if __name__=='__main__':main()
