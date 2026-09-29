"""Latest-only local HTS pose forwarding, independent of finger retargeting."""
import asyncio
import contextlib
import time
import uuid

import aiohttp


class ArmClient:
    def __init__(self, url):
        self.url = url.rstrip('/')
        self.identity = uuid.uuid4().hex
        self.session = None
        self.latest = None
        self.epoch = 0
        self.need_pause = True
        self.tracking_pause_only = True
        self.state = dict(status='正在连接机械臂服务', enabled=False)
        self.error = None
        self.http = None
        self.event = asyncio.Event()

    def submit(self, frame, owner, head):
        self.session = self.identity + '-' + owner
        self.latest = dict(side='right', session=self.session, sequence=frame.frame_id,
                           captured=frame.captured, position=frame.wrist_position.tolist(),
                           rotation=frame.wrist_rotation.tolist(),
                           head_rotation=None if head is None else head.rotation.tolist(),
                           head_captured=None if head is None else head.captured)
        self.event.set()

    def hold(self, tracking_only=True):
        self.latest = None
        self.epoch += 1
        self.need_pause = True
        self.tracking_pause_only = self.tracking_pause_only and tracking_only
        self.event.set()

    async def control(self, action, **options):
        if self.http is None:
            raise ValueError('机械臂服务尚未连接')
        if action == 'align' and self.need_pause:
            raise ValueError('跟踪刚恢复，请稍后重新对齐')
        async with self.http.post(self.url+'/control', json=dict(
                source='hts-x5', action=action, session=self.session, **options)) as reply:
            data = await reply.json()
            if reply.status != 200:
                raise ValueError(data.get('error', '机械臂请求失败'))
            self.state = data
            return data

    async def run(self):
        timeout = aiohttp.ClientTimeout(total=.15)
        async with aiohttp.ClientSession(timeout=timeout) as self.http:
            last_key, last_poll = None, 0.
            while True:
                self.event.clear()
                start = time.monotonic()
                try:
                    if self.need_pause:
                        epoch = self.epoch
                        await self.control('pause', tracking_only=self.tracking_pause_only)
                        if epoch == self.epoch:
                            self.need_pause = False
                            self.tracking_pause_only = True
                    data = self.latest
                    key = None if data is None else (data['session'], data['sequence'])
                    if data is not None and key != last_key and start-data['captured'] < .2:
                        last_key = key
                        async with self.http.post(self.url+'/frame', json=data) as reply:
                            state = await reply.json()
                            if reply.status != 200:
                                raise ValueError(state.get('error', '机械臂位姿被拒绝'))
                            self.state = state
                        last_poll = start
                    elif start-last_poll > .3:
                        async with self.http.get(self.url+'/state') as reply:
                            self.state = await reply.json()
                        last_poll = start
                    self.error = None
                except (aiohttp.ClientError, asyncio.TimeoutError, ValueError) as error:
                    self.error = str(error) or type(error).__name__
                    self.state = dict(status='机械臂服务失联/拒绝数据', enabled=False)
                    self.need_pause = True
                    await asyncio.sleep(.03)
                # Wake on the newest native packet: no additional 30 Hz relay delay.
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self.event.wait(), timeout=.1)

    async def close(self):
        self.hold()
        if self.http is not None and not self.http.closed:
            with contextlib.suppress(Exception):
                await self.control('pause')

    def snapshot(self):
        return dict(self.state, transport_error=self.error)
