"""Quest hand-only service v3: one XR owner, latest frame, recoverable tracking loss."""
import argparse,asyncio,contextlib,fcntl,json,os,secrets,time
from pathlib import Path
import numpy as np
from aiohttp import web
from mapping import O10Retargeter,palm_coordinates,validate_frame
from direct_sender import DirectSender
ROOT=Path(__file__).resolve().parent
VERSION=3

class HandService:
    def __init__(self,hardware=False,side='left'):
        if side not in ('left','right'):raise ValueError('Invalid hand side')
        self.side=side;self.label='左手' if side=='left' else '右手'
        self.serial=os.environ.get('O10_EXPECTED_SERIAL',os.environ.get('O10_'+side.upper()+'_SERIAL', ''))
        if hardware and not self.serial:
            raise ValueError('Configure the selected O10 USB serial before enabling hardware.')
        self.rt=O10Retargeter(side,'dexpilot',alpha=1,extension_assist=True,fist_assist=True)
        sign=1 if side=='left' else -1
        p=self.rt.reference_from_active(np.array([0.,sign*.6,-sign*.5,0.,.5,.5,0.,.5,0.,.5]))
        for _ in range(3):self.rt.retarget(palm_coordinates(p,side))
        self.rt.filtered=None
        self.rt.seq.optimizer.opt.set_maxeval(100)
        self.rt.seq.optimizer.opt.set_maxtime(.012)
        self.owner=None;self.owner_id=None;self.epoch=0;self.follow=False;self.tracked=False;self.output_enabled=False
        self.latest=None;self.event=asyncio.Event();self.valid_run=0;self.received=0.;self.started=time.monotonic()
        self.frames=0;self.dropped=0;self.errors=0;self.reason='等待进入 Quest 裸手跟踪';self.angles=None;self.retarget_ms=None;self.frame_age_ms=None
        self.client_phase={};self.client_metrics={};self.sender=None
        if hardware:
            self.lock=open('/tmp/o10-hamer-preview.lock','w');fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.sender=DirectSender(self.rt.names,os.environ.get('O10_BRIDGE_URL','http://127.0.0.1:8890'),self.serial,side=side)
    def hold(self,reason,manual=False):
        if manual:self.follow=False
        self.epoch+=1;self.latest=None;self.valid_run=0;self.tracked=False;self.reason=reason
        if self.output_enabled and self.sender:self.sender.hold()
        self.output_enabled=False
    def enable(self):
        self.follow=True;self.reason='已启用，等待'+self.label
        if self.sender and self.sender.fault:self.sender.fault=None
    def snapshot(self):
        return dict(version=VERSION,side=self.side,hand_only=True,mode='hardware' if self.sender else 'preview',status=self.reason,
            follow_requested=self.follow,tracking=self.tracked,enabled=self.output_enabled,frames=self.frames,dropped=self.dropped,
            age_ms=None if not self.received else (time.monotonic()-self.received)*1000,retarget_ms=self.retarget_ms,
            frame_age_upper_ms=self.frame_age_ms,o10_angles_rad=self.angles,session=self.owner_id,
            client_phase=self.client_phase,client_metrics=self.client_metrics,hardware_usb_serial=self.serial,hardware=None if not self.sender else self.sender.snapshot())
    async def ws(self,request):
        if request.query.get('v')!=str(VERSION):return web.Response(status=426,text='Refresh Quest page: protocol v3 required')
        ws=web.WebSocketResponse(max_msg_size=32768,heartbeat=5);await ws.prepare(request)
        ident=secrets.token_hex(8);issued={};last_seq=-1;claimed=False
        async def challenge_loop():
            while not ws.closed:
                token=secrets.token_hex(8);now=time.monotonic();issued[token]=now
                for k in list(issued):
                    if now-issued[k]>.4:issued.pop(k)
                await ws.send_json(dict(type='challenge',id=token,side=self.side,version=VERSION));await asyncio.sleep(.04)
        task=asyncio.create_task(challenge_loop())
        try:
            async for message in ws:
                if message.type!=web.WSMsgType.TEXT:continue
                try:
                    data=json.loads(message.data);kind=data.get('type');now=time.monotonic()
                    if kind=='start':
                        if claimed:continue
                        old=self.owner;self.hold('新跟踪会话');self.owner=ws;self.owner_id=ident;claimed=True
                        if data.get('enable') is True:self.enable()
                        if old and old is not ws:await old.close(code=4001,message=b'New XR session')
                        await ws.send_json(dict(type='ready',side=self.side,session=ident));continue
                    if not claimed or self.owner is not ws:continue
                    if kind=='metrics':
                        self.client_metrics={k:data[k] for k in ('first_frame_ms','xr_ready_ms','ws_ready_ms','hand_input_ms','viewer_pose_ms','visible_ms','all_joints_ms') if isinstance(data.get(k),(int,float))};continue
                    if kind=='release':
                        self.hold('跟踪会话结束');self.owner=None;self.owner_id=None;claimed=False
                        await ws.send_json(dict(type='released'));continue
                    if kind=='enable':self.enable();continue
                    if kind=='pause':self.hold('已手动暂停',manual=True);continue
                    if kind=='lost':self.hold(data.get('reason','等待'+self.label));continue
                    if kind!='frame':continue
                    seq=int(data['seq'])
                    if seq<=last_seq:self.dropped+=1;continue
                    last_seq=seq;issued_at=issued.get(data.get('challenge'))
                    if issued_at is None or now-issued_at>.2:
                        self.dropped+=1;continue # watchdog handles prolonged loss; one delayed packet must not reset intent
                    try:points,_,_=validate_frame(data,self.side)
                    except (ValueError,KeyError,TypeError):self.hold('等待'+self.label+'进入视野');continue
                    self.received=now;self.frame_age_ms=(now-issued_at)*1000;self.frames+=1;self.valid_run+=1;self.tracked=True
                    self.latest=(points,now,self.epoch);self.event.set()
                except Exception as e:self.errors+=1;self.hold('跟踪数据错误: '+str(e))
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):await task
            if self.owner is ws:self.hold('Quest 已断开，等待重连');self.owner=None;self.owner_id=None
        return ws
    async def worker(self):
        while True:
            await self.event.wait();self.event.clear();item,self.latest=self.latest,None
            if item is None:continue
            points,captured,epoch=item
            if time.monotonic()-captured>.2:self.dropped+=1;continue
            try:
                started=time.perf_counter()
                _,active=await asyncio.to_thread(self.rt.retarget,palm_coordinates(points,self.side))
                self.retarget_ms=(time.perf_counter()-started)*1000
                if epoch!=self.epoch or time.monotonic()-captured>.2:self.dropped+=1;continue
                self.angles=active.tolist()
                if self.sender and self.sender.fault:self.hold('驱动故障: '+self.sender.fault,manual=True);continue
                if self.follow and self.valid_run>=1:
                    if not self.output_enabled:
                        if self.sender:self.sender.enable()
                        self.output_enabled=True
                    if self.sender:self.sender.submit(self.angles,captured)
                    self.reason=self.label+'跟随中'
                else:self.reason='已识别'+self.label+' / 已暂停' if not self.follow else '正在确认'+self.label
            except Exception as e:self.errors+=1;self.hold('映射错误: '+str(e),manual=True)
    async def watchdog(self):
        saved=0
        while True:
            await asyncio.sleep(.02)
            if self.tracked and time.monotonic()-self.received>.2:self.hold('等待'+self.label+' / 跟踪超时')
            if self.sender and self.sender.fault and self.follow:self.hold('驱动故障: '+self.sender.fault,manual=True)
            if time.monotonic()-saved>1:
                (ROOT/'output/state.json').write_text(json.dumps(self.snapshot(),ensure_ascii=False));saved=time.monotonic()
    async def control(self,request):
        data=await request.json()
        # Older cached pages must never modify the new session.
        if data.get('version')!=VERSION:return web.json_response(dict(error='Refresh page: v3 required'),status=409)
        if data.get('action')=='enable':self.enable()
        elif data.get('action')=='pause':self.hold('已手动暂停',manual=True)
        else:return web.json_response(dict(error='Unknown action'),status=400)
        return web.json_response(self.snapshot())

def application(service):
    @web.middleware
    async def no_cache(request,handler):
        response=await handler(request)
        if request.path!='/ws':response.headers['Cache-Control']='no-store'
        return response
    app=web.Application(middlewares=[no_cache]);app.router.add_get('/ws',service.ws)
    async def state(_):return web.json_response(service.snapshot())
    app.router.add_get('/state',state);app.router.add_post('/control',service.control)
    async def client_event(request):
        data=await request.json()
        if data.get('version')!=VERSION:return web.Response(status=409)
        service.client_phase={k:str(data[k])[:300] for k in ('phase','error') if k in data}
        service.client_phase['updated']=time.time()
        print('CLIENT_EVENT',json.dumps(service.client_phase,ensure_ascii=False),flush=True)
        return web.json_response({'ok':True})
    app.router.add_post('/client_event',client_event)
    app.router.add_get('/',lambda _:web.FileResponse(ROOT/'web/hand.html'));app.router.add_get('/hand-v3.js',lambda _:web.FileResponse(ROOT/'web/hand-v3.js'))
    async def start(a):a['tasks']=[asyncio.create_task(service.worker()),asyncio.create_task(service.watchdog())]
    async def stop(a):
        service.hold('服务停止',manual=True)
        if service.owner:await service.owner.close()
        for task in a['tasks']:task.cancel()
        for task in a['tasks']:
            with contextlib.suppress(asyncio.CancelledError):await task
        if service.sender:service.sender.close()
    app.on_startup.append(start);app.on_cleanup.append(stop);return app

def main():
    p=argparse.ArgumentParser();p.add_argument('--hardware',action='store_true');p.add_argument('--port',type=int,default=8042);args=p.parse_args()
    lock=open('/tmp/quest-o10-x5.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    service=HandService(args.hardware);web.run_app(application(service),host='127.0.0.1',port=args.port,shutdown_timeout=1)
if __name__=='__main__':main()
