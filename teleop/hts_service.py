"""Native HTS -> side-specific DexPilot -> serial-locked ARX-local O10 bridge."""
import argparse
import asyncio
import contextlib
import fcntl
import json
import os
import socket
import time
from aiohttp import web, ClientError
from hand_service import HandService, ROOT
from hts_input import HTSParser, HeadParser
from arm_client import ArmClient

class HTSService(HandService):
    def __init__(self, hardware=False, side='left', arm_url=None):
        if arm_url and side != 'right':
            raise ValueError('This X5 deployment requires the right hand')
        self.arm = ArmClient(arm_url) if arm_url else None
        self.head = None
        self.head_owner = None
        super().__init__(hardware, side=side)
        self.owner_writer = None
        self.connections = set()
        self.tcp_server = None
        self.require_metadata = True
        self.last_input_error = None
        self.hts_first_frame_ms = None
        self.hts_connected_at = None
        self.last_source_id = None
        self.transport_lag_ms = None
        self.reason = 'HTS 已就绪，等待'+self.label+'数据'
        self.enable()

    def hold(self, reason, manual=False):
        super().hold(reason, manual)
        if self.arm:
            self.arm.hold(tracking_only=not manual)

    def snapshot(self):
        state = super().snapshot()
        state.update(source='hts', host='arx', protocol='USB TCP (local ARX)', input_error=self.last_input_error,
                     native_frame_id=self.last_source_id, transport_lag_ms=self.transport_lag_ms,
                     first_frame_after_connect_ms=self.hts_first_frame_ms,
                     hts_connections=len(self.connections))
        if self.arm:
            state.update(hand_only=False, arm=self.arm.snapshot(),
                         head_age_ms=None if self.head is None else (time.monotonic()-self.head.captured)*1000)
        return state

    async def receive(self, reader, writer):
        self.connections.add(writer)
        sock = writer.get_extra_info('socket')
        if sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        parser, buffer = HTSParser(self.require_metadata, side=self.side), b''
        head_parser = HeadParser()
        connected = time.monotonic()
        try:
            while True:
                chunk = await reader.read(65536)
                if not chunk:
                    break
                buffer += chunk
                lines = buffer.split(b'\n')
                buffer = lines.pop()
                if len(buffer) > 8192:
                    raise ValueError('Oversized HTS line')
                latest = None
                for line in lines:
                    if len(line) > 8192:
                        raise ValueError('Oversized HTS line')
                    try:
                        if line.startswith(b'Head ') and self.arm:
                            try:
                                head = head_parser.feed(line, time.monotonic())
                                if head is not None:
                                    self.head, self.head_owner = head, writer
                            except ValueError:
                                if self.head_owner is writer:
                                    self.head = None
                                    self.arm.hold()
                            continue
                        frame = parser.feed(line, time.monotonic())
                        if frame is not None:
                            if latest is not None:
                                self.dropped += 1
                            latest = frame
                    except (ValueError, UnicodeError) as error:
                        self.dropped += 1
                        latest = None
                        message = str(error)
                        if self.last_input_error != message:
                            print('HTS_INPUT', message, flush=True)
                        self.last_input_error = message
                        if self.owner_writer is writer:
                            self.hold(message)
                        elif self.owner_writer is None:
                            self.reason = message
                if latest is None:
                    continue
                # Probe, opposite-hand, and head sockets never take control.
                if self.owner_writer is not writer:
                    old = self.owner_writer
                    self.hold('HTS '+self.label+'已连接')
                    self.owner_writer = writer
                    self.owner_id = 'hts-' + str(id(writer))
                    self.hts_connected_at = connected
                    self.hts_first_frame_ms = (time.monotonic()-connected)*1000
                    if old:
                        old.close()
                    print('HTS_FIRST_FRAME', json.dumps(dict(ms=self.hts_first_frame_ms,
                          frame_id=latest.frame_id)), flush=True)
                self.last_input_error = None
                self.received = latest.captured
                self.frame_age_ms = latest.lag_ms
                self.transport_lag_ms = latest.lag_ms
                self.last_source_id = latest.frame_id
                self.frames += 1
                self.valid_run += 1
                self.tracked = True
                self.latest = (latest.points, latest.captured, self.epoch)
                self.event.set()
                if self.arm and self.follow:
                    self.arm.submit(latest, self.owner_id, self.head)
        except (ValueError, ConnectionError, OSError) as error:
            self.last_input_error = str(error)
        finally:
            self.connections.discard(writer)
            if self.head_owner is writer:
                self.head, self.head_owner = None, None
                if self.arm:
                    self.arm.hold()
            if self.owner_writer is writer:
                self.hold('HTS 已断开，等待重新开始传输')
                self.owner_writer = None
                self.owner_id = None
            writer.close()
            with contextlib.suppress(ConnectionError, OSError):
                await writer.wait_closed()

    async def watchdog(self):
        saved = 0
        while True:
            await asyncio.sleep(.02)
            if self.tracked and time.monotonic()-self.received > .2:
                self.hold('等待'+self.label+' / HTS 跟踪超时')
            if self.sender and self.sender.fault and self.follow:
                self.hold('驱动故障: '+self.sender.fault, manual=True)
            if time.monotonic()-saved > 1:
                (ROOT/'output/hts_state.json').write_text(json.dumps(self.snapshot(),ensure_ascii=False))
                saved = time.monotonic()

    async def control(self, request):
        data = await request.json()
        if data.get('source') != 'hts':
            return web.json_response(dict(error='HTS control required'), status=409)
        if data.get('action') in ('arm_align', 'arm_pause', 'arm_home'):
            if self.arm is None:
                return web.json_response(dict(error='未启动机械臂模式'), status=409)
            if data['action'] == 'arm_align' and not self.follow:
                return web.json_response(dict(error='请先恢复手部跟随，再对齐机械臂'), status=409)
            try:
                await self.arm.control({'arm_align': 'align', 'arm_pause': 'pause', 'arm_home': 'home'}[data['action']])
            except (ValueError, OSError, ClientError, asyncio.TimeoutError) as error:
                return web.json_response(dict(error=str(error)), status=409)
        elif data.get('action') == 'enable':
            self.enable()
        elif data.get('action') == 'pause':
            self.hold('已手动暂停', manual=True)
        else:
            return web.Response(status=400)
        return web.json_response(self.snapshot())

def application(service, tcp_port=8000):
    app = web.Application()
    async def state(_):
        return web.json_response(service.snapshot(), headers={'Cache-Control':'no-store'})
    async def page(_):
        return web.FileResponse(ROOT/'web/hts.html',headers={'Cache-Control':'no-store'})
    app.router.add_get('/', page)
    app.router.add_get('/state', state)
    app.router.add_post('/control', service.control)
    async def start(a):
        service.tcp_server = await asyncio.start_server(service.receive,'127.0.0.1',tcp_port)
        (ROOT/'output').mkdir(exist_ok=True)
        a['tasks'] = [asyncio.create_task(service.worker()),asyncio.create_task(service.watchdog())]
        if service.arm:
            a['tasks'].append(asyncio.create_task(service.arm.run()))
        print('HTS_READY', json.dumps(dict(tcp_port=service.tcp_server.sockets[0].getsockname()[1],
                    hardware=bool(service.sender),side=service.side,serial=service.serial)),flush=True)
    async def stop(a):
        service.hold('HTS 服务停止',manual=True)
        if service.arm:
            await service.arm.close()
        service.tcp_server.close()
        await service.tcp_server.wait_closed()
        for writer in list(service.connections):
            writer.close()
        for task in a.get('tasks',[]):
            task.cancel()
        for task in a.get('tasks',[]):
            with contextlib.suppress(asyncio.CancelledError):
                await task
        if service.sender:
            service.sender.close()
    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    return app

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware',action='store_true')
    parser.add_argument('--arm-url', help='Local persistent X5 service, e.g. http://127.0.0.1:8043')
    parser.add_argument('--side', choices=('left','right'), default='right')
    parser.add_argument('--port',type=int,default=8042)
    parser.add_argument('--tcp-port',type=int,default=8000)
    args = parser.parse_args()
    # Same ownership lock as WebXR: never run both against the real hand.
    lock = open('/tmp/quest-o10-x5.lock' if args.hardware else '/tmp/quest-hts-preview.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    os.environ.setdefault('O10_BRIDGE_URL', 'http://127.0.0.1:8890')
    service = HTSService(args.hardware, side=args.side, arm_url=args.arm_url)
    web.run_app(application(service,args.tcp_port),host='127.0.0.1',port=args.port,shutdown_timeout=1)

if __name__ == '__main__':
    main()
