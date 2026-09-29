"""Use Quest's system compositor capture for undistorted right-eye video.

CaptureService actions are the USB equivalent of MQDH single-eye recording.
No runtime restart, proximity override, Guardian change, or robot commands.
"""
import asyncio
from datetime import datetime
import re
import shlex
import shutil
import struct
import time
import uuid

from quest_recording import Recorder, write_json

DIRECTORY = '/sdcard/Oculus/VideoShots'
BACKEND = 'meta-native'
PROPERTIES = {
    'debug.oculus.screenCaptureEye': '1',
    'debug.oculus.capture.width': '1080',
    'debug.oculus.capture.height': '1080',
    'debug.oculus.capture.bitrate': '10000000',
    'debug.oculus.capture.fps': '36',
    'debug.oculus.fullRateCapture': '0',
}


class NativeRecorder(Recorder):
    def snapshot(self):
        state = super().snapshot()
        state.update(version=2, backend=BACKEND,
                     settings=dict(size='1080x1080', bitrate_mbps=10, fps=36,
                                   eye='right', view='undistorted-mono', audio=False))
        return state

    async def videos(self):
        _, text = await self.adb('shell',
            "for file in /sdcard/Oculus/VideoShots/*.mp4; do "
            "if [ -f \"$file\" ]; then stat -c '%n|%s|%Y' \"$file\"; fi; done")
        result = {}
        for line in text.splitlines():
            parts = line.split('|')
            if len(parts) != 3:
                continue
            path, size, modified = parts
            if re.fullmatch(re.escape(DIRECTORY) + r'/[A-Za-z0-9_.-]+\.mp4', path):
                result[path] = dict(size=int(size), modified=int(modified))
        return result

    async def header(self, path, offset):
        _, value = await self.adb('shell',
            f'dd if={shlex.quote(path)} bs=1 skip={int(offset)} count=16 2>/dev/null | od -An -v -tx1')
        return bytes.fromhex(value)

    async def finalized(self, path):
        """Read only MP4 box headers, never transfer video while it is recording.

        Meta's writer leaves mdat size zero while active, then writes a complete
        moov box on stop/unmount. File-size stability alone cannot identify this.
        """
        _, value = await self.adb('shell', 'stat -c %s ' + shlex.quote(path))
        length, offset = int(value), 0
        for _ in range(32):
            if offset + 8 > length:
                return False
            header = await self.header(path, offset)
            if len(header) < 8:
                return False
            size, kind = struct.unpack('>I4s', header[:8])
            minimum = 8
            if size == 1:
                if len(header) < 16:
                    return False
                size, minimum = struct.unpack('>Q', header[8:16])[0], 16
            if size < minimum or offset + size > length:
                return False
            if kind == b'moov':
                return True
            offset += size
        return False

    async def capture_action(self, action):
        if action not in ('START_INTERNAL_CAPTURE_TO_DISK', 'STOP_INTERNAL_CAPTURE_TO_DISK'):
            raise ValueError('Unknown capture action')
        _, result = await self.adb('shell', 'am', 'start-foreground-service', '-n',
                                  'com.oculus.metacam/.capture.CaptureService', '-a', action,
                                  '--es', 'capture_entrypoint', 'ODH')
        if 'Error' in result or 'Exception' in result:
            raise RuntimeError(result)

    async def restore_properties(self):
        for key, value in self.job.get('original_properties', {}).items():
            _, current = await self.adb('shell', 'getprop', key)
            # Preserve a user change made after our capture started.
            if current == PROPERTIES[key]:
                await self.adb('shell', 'setprop ' + key + ' ' + shlex.quote(value))

    async def locate_job_file(self):
        if self.job.get('remote'):
            return self.job['remote']
        current = await self.videos()
        candidates = set(current) - set(self.job['baseline'])
        if len(candidates) == 1:
            self.job['remote'] = candidates.pop()
            write_json(self.active_file, self.job)
            return self.job['remote']
        if len(candidates) > 1:
            raise RuntimeError('发现多个新的系统录像，无法确认归属；未停止其他录制。')
        return None

    async def start(self):
        async with self.lock:
            if self.job or self.phase not in ('idle', 'error'):
                raise RuntimeError('已有录制或待保存视频，请先停止并保存。')
            _, state = await self.adb('get-state')
            if state != 'device':
                raise RuntimeError('请连接 Quest USB 并允许调试。')
            _, power = await self.adb('shell', 'dumpsys power')
            if 'mWakefulness=Asleep' in power or 'mWakefulness=Dozing' in power:
                raise RuntimeError('Quest 正在休眠，请戴上头显、进入 HTS 后再开始录制。')
            _, pids = await self.adb('shell', 'pidof screenrecord', check=False)
            if pids:
                raise RuntimeError('已有 ADB 录屏，请先结束原录制。')
            if shutil.disk_usage(self.output).free < 1024**3:
                raise RuntimeError('ARX 剩余空间不足 1 GB，请先整理视频。')
            await self.adb('shell', 'mkdir -p ' + DIRECTORY)
            baseline = await self.videos()
            if baseline:
                newest = max(baseline, key=lambda p: (baseline[p]['modified'], p))
                if not await self.finalized(newest):
                    raise RuntimeError('Quest 系统已有未完成录像，请先在头显内结束录制。')
            original = {}
            for key in PROPERTIES:
                _, original[key] = await self.adb('shell', 'getprop', key)
            name = datetime.now().strftime('hts_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8] + '.mp4'
            self.job = dict(name=name, remote=None, started_at=time.time(), backend=BACKEND,
                            baseline=list(baseline), original_properties=original)
            write_json(self.active_file, self.job)
            self.phase, self.error = 'starting', None
            try:
                for key, value in PROPERTIES.items():
                    await self.adb('shell', 'setprop', key, value)
                await self.capture_action('START_INTERNAL_CAPTURE_TO_DISK')
                for _ in range(30):
                    if await self.locate_job_file():
                        self.phase = 'recording'
                        self.watcher = asyncio.create_task(self.watch_native())
                        return
                    await asyncio.sleep(.1)
                raise RuntimeError('系统录制未生成文件，请确认头显已佩戴并保持 HTS 在前台。')
            except (OSError, RuntimeError, asyncio.TimeoutError, ValueError) as error:
                # Leave a durable ownership record to resolve an uncertain start.
                self.fail(error)
                raise

    async def signal_owned(self):
        if self.job.get('backend') != BACKEND:
            return await super().signal_owned()
        path = await self.locate_job_file()
        if not path:
            raise RuntimeError('尚未找到本次系统录像；请确认设备连接后重试保存。')
        if await self.finalized(path):
            return
        videos = await self.videos()
        newest = max(videos, key=lambda p: (videos[p]['modified'], p)) if videos else None
        if newest != path:
            raise RuntimeError('当前系统录像与本次任务不匹配，未停止其他录制。')
        await self.capture_action('STOP_INTERNAL_CAPTURE_TO_DISK')

    async def wait_device_recording_end(self):
        if self.job.get('backend') != BACKEND:
            return await super().wait_device_recording_end()
        for _ in range(40):
            if self.job.get('remote') and await self.finalized(self.job['remote']):
                await self.restore_properties()
                return
            await asyncio.sleep(.25)
        raise RuntimeError('系统录像仍在收尾，请稍后重试保存。')

    async def watch_native(self):
        while True:
            await asyncio.sleep(1)
            async with self.lock:
                if not self.job or self.phase not in ('recording', 'stopping'):
                    return
                try:
                    if self.phase == 'stopping' or await self.finalized(self.job['remote']):
                        await self.save()
                        return
                except (OSError, RuntimeError, asyncio.TimeoutError, ValueError) as error:
                    self.fail(error)
                    return

    async def prepare_video(self, path):
        # Stream copy: retain compositor pixels exactly, remove audio and move
        # the index to the front for immediate browser playback. No re-encoding.
        output = path.with_suffix('.ready.mp4')
        await self.run('ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(path),
                       '-map', '0:v:0', '-c:v', 'copy', '-an', '-movflags', '+faststart',
                       str(output), timeout=120)
        output.replace(path)

    def capture_metadata(self):
        if self.job.get('backend') != BACKEND:
            return super().capture_metadata()
        return dict(capture_backend=BACKEND, view='undistorted-mono', eye='right', audio=False)
