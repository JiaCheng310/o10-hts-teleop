import json,threading,time,unittest,sys
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from direct_sender import DirectSender
class SenderTests(unittest.TestCase):
 def setUp(self):
  self.targets=[];self.fail='EXPIRED_TOKEN';outer=self
  self.names=['L_'+str(i) for i in range(10)]
  class Handler(BaseHTTPRequestHandler):
   protocol_version='HTTP/1.1'
   def reply(self,status,value):
    raw=json.dumps(value).encode();self.send_response(status);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
   def base(self):return dict(hand='left',usb_port='TEST_LEFT_SERIAL',joint_names=outer.names,allow_motion=True,nonce='x',angles=[0]*10)
   def do_GET(self):self.reply(200,self.base())
   def do_POST(self):
    data=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
    time.sleep(.008)
    if outer.fail:
     code=outer.fail;outer.fail=None;self.reply(409,dict(code=code,error='injected test failure'))
    else:outer.targets.append(data['angles']);self.reply(200,self.base())
   def log_message(self,*args):pass
  self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler);self.server.daemon_threads=True
  self.t=threading.Thread(target=self.server.serve_forever,daemon=True);self.t.start()
  self.sender=DirectSender(self.names,f'http://127.0.0.1:{self.server.server_port}','TEST_LEFT_SERIAL');self.sender.enable()
 def tearDown(self):self.sender.close();self.server.shutdown();self.server.server_close()
 def test_driver_warms_without_motor_commands(self):
  self.sender.hold()
  for _ in range(50):
   if self.sender.ready:break
   time.sleep(.01)
  self.assertTrue(self.sender.ready)
  self.assertEqual(self.targets,[])
  self.assertEqual(self.sender.sent,0)
 def test_expiration_recovers_using_newest_target(self):
  for i in range(25):self.sender.submit([i/25]*10,time.monotonic());time.sleep(.012)
  time.sleep(.1)
  self.assertGreater(self.sender.sent,0);self.assertEqual(self.sender.transient_errors,1);self.assertIsNone(self.sender.fault)
  self.assertEqual(self.targets[-1],[24/25]*10)
  self.assertLess(len(self.targets),25)
  self.sender.hold();n=len(self.targets);self.sender.submit([.1]*10,time.monotonic());time.sleep(.08);self.assertEqual(len(self.targets),n)
 def test_fault_latches(self):
  self.fail='DEVICE_FAULT';self.sender.submit([.1]*10,time.monotonic())
  for _ in range(40):
   if self.sender.fault:break
   time.sleep(.01)
  self.assertIn('DEVICE_FAULT',self.sender.fault)
  self.sender.submit([.2]*10,time.monotonic());time.sleep(.04);self.assertEqual(self.targets,[])
if __name__=='__main__':unittest.main()
