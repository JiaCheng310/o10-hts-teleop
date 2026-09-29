"""One in-flight request, one replaceable latest target, persistent HTTP."""
import http.client,json,socket,threading,time
from urllib.parse import urlsplit

class DirectSender:
    def __init__(self,names,url,serial,side='left'):
        if side not in ('left','right'):raise ValueError('Invalid hand side')
        self.side=side
        self.names,self.url,self.serial=names,url,serial
        self.condition=threading.Condition();self.pending=None;self.running=True;self.active=False;self.epoch=0
        self.fault=None;self.status='waiting';self.sent=0;self.transient_errors=0;self.last=None
        self.connection=None;self.nonce=None;self.nonce_time=0;self.ready=False
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def enable(self):
        with self.condition:self.fault=None;self.active=True;self.epoch+=1;self.condition.notify()
    def hold(self):
        with self.condition:self.pending=None;self.active=False;self.epoch+=1
    def submit(self,angles,captured):
        with self.condition:
            if self.active and not self.fault:self.pending=(list(angles),captured,self.epoch);self.condition.notify()
    def request(self,path,data=None):
        if self.connection is None:
            u=urlsplit(self.url);self.connection=http.client.HTTPConnection(u.hostname,u.port,timeout=.3)
            self.connection.connect();self.connection.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        body=None if data is None else json.dumps(data)
        self.connection.request('GET' if data is None else 'POST',path,body,{'Content-Type':'application/json'})
        reply=self.connection.getresponse();value=json.loads(reply.read())
        if reply.status!=200:
            error=RuntimeError(value.get('error',str(value)));error.code=value.get('code','DRIVER_ERROR');raise error
        if value.get('hand')!=self.side or value.get('joint_names')!=self.names or self.serial not in value.get('usb_port','') or not value.get('allow_motion'):
            error=RuntimeError('Physical hand identity mismatch');error.code='IDENTITY_MISMATCH';raise error
        self.nonce=value['nonce'];self.nonce_time=time.monotonic();self.ready=True;return value
    def close_connection(self):
        if self.connection:
            try:self.connection.close()
            except Exception:pass
        self.connection=None;self.nonce=None;self.ready=False
    def run(self):
        while self.running:
            with self.condition:
                self.condition.wait_for(lambda:not self.running or self.pending is not None,timeout=.08)
                if not self.running:break
                item,self.pending=self.pending,None
            try:
                if item is None:
                    if not self.fault and (self.nonce is None or time.monotonic()-self.nonce_time>.1):
                        self.request('/state')
                        if not self.active:self.status='ready / no targets'
                    continue
                if self.connection and time.monotonic()-self.nonce_time>1:self.close_connection()
                if self.nonce is None or time.monotonic()-self.nonce_time>.16:self.request('/state')
                # During a read, replace the candidate with the newest pose.
                with self.condition:
                    if self.pending is not None:item,self.pending=self.pending,None
                    angles,captured,epoch=item
                    if not self.active or self.fault or epoch!=self.epoch:continue
                age=time.monotonic()-captured
                if age>.18:self.status='dropped stale target';continue
                started=time.monotonic()
                value=self.request('/target',dict(nonce=self.nonce,enabled=True,hand=self.side,joint_names=self.names,
                    angles=angles,freshness_budget_s=min(.2,.2-age)))
                self.sent+=1;self.status='live / direct targets'
                self.last=dict(sent=self.sent,measured=value['angles'],target=angles,bridge_ms=(time.monotonic()-started)*1000,
                    driver_ms=value.get('driver_ms'),target_age_ms=age*1000,updated=time.time())
            except Exception as e:
                code=getattr(e,'code','TRANSPORT');self.close_connection()
                if code in ('EXPIRED_TOKEN','STALE_TARGET','TRANSPORT'):
                    self.transient_errors+=1;self.status=f'reconnecting: {code}: {e}'
                    # Never resend the old target after an uncertain reply.
                    with self.condition:self.pending=None
                    time.sleep(.03)
                else:
                    self.fault=f'{code}: {e}';self.status=self.fault;self.hold()
                    print('HARDWARE_FAULT',self.fault,flush=True)
        self.close_connection()
    def snapshot(self):return dict(ready=self.ready,status=self.status,sent=self.sent,fault=self.fault,transient_errors=self.transient_errors,last=self.last)
    def close(self):
        self.running=False
        with self.condition:self.pending=None;self.condition.notify()
        self.thread.join(1)
