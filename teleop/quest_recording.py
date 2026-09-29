"""Independent Quest display recorder. Never imports or controls robot drivers."""
import argparse
import asyncio
import contextlib
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import time
import uuid

from aiohttp import web

ROOT = Path(__file__).resolve().parent
SERIAL = os.environ.get('QUEST_SERIAL', '')
OUTPUT = ROOT / 'output' / 'quest_recordings'
REMOTE = '/sdcard/Movies/HTS'
NAME = re.compile(r'hts_\d{8}_\d{6}_[0-9a-f]{8}\.mp4\Z')


class EmptyRecordingError(RuntimeError):
    pass


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(path)


class Recorder:
    def __init__(self, output=OUTPUT, serial=SERIAL):
        self.output, self.serial = Path(output), serial
        self.output.mkdir(parents=True, exist_ok=True)
        self.active_file = self.output / 'active.json'
        self.job = json.loads(self.active_file.read_text()) if self.active_file.exists() else None
        self.phase = 'recoverable' if self.job else 'idle'
        self.error = '上次录制尚未保存，请点击重试保存。' if self.job else None
        self.connected = False
        self.awake = None
        self.device_error = '正在检查 Quest USB'
        self.lock = asyncio.Lock()
        self.process = self.watcher = self.health = None

    async def run(self, *args, timeout=8, check=True):
        process = await asyncio.create_subprocess_exec(
            *args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        try:
            out, err = await asyncio.wait_for(process.communicate(), timeout)
        except BaseException:
            if process.returncode is None:
                process.kill()
            await process.communicate()
            raise
        if check and process.returncode:
            raise RuntimeError((err or out).decode(errors='replace').strip()[-600:] or '命令执行失败')
        return process.returncode, out.decode(errors='replace').strip()

    async def adb(self, *args, **kwargs):
        return await self.run('adb', '-s', self.serial, *args, **kwargs)

    async def device_health(self):
        while True:
            try:
                _, result = await self.adb('get-state', timeout=4)
                self.connected = result == 'device'
                self.device_error = None if self.connected else 'Quest USB 未就绪'
                if self.connected:
                    _, power = await self.adb('shell', 'dumpsys power', timeout=4)
                    match = re.search(r'mWakefulness=(\w+)', power)
                    self.awake = match[1] == 'Awake' if match else None
            except (OSError, RuntimeError, asyncio.TimeoutError) as error:
                self.connected = False
                self.awake = None
                self.device_error = str(error) or 'Quest USB 连接超时'
            await asyncio.sleep(3)

    def files(self):
        results = []
        for path in sorted(self.output.glob('hts_*.mp4'), reverse=True)[:30]:
            if not NAME.fullmatch(path.name) or path.is_symlink():
                continue
            entry = dict(name=path.name, bytes=path.stat().st_size)
            metadata = path.with_suffix('.json')
            if metadata.exists():
                with contextlib.suppress(ValueError, OSError):
                    entry.update(json.loads(metadata.read_text()))
            results.append(entry)
        return results

    def snapshot(self):
        return dict(source='quest-recording', version=1, phase=self.phase,
                    connected=self.connected, awake=self.awake, device_error=self.device_error,
                    error=self.error, job=self.job,
                    elapsed_s=max(0, time.time()-self.job['started_at']) if self.job else 0,
                    settings=dict(size='1280x720', bitrate_mbps=8, audio=False),
                    output=str(self.output), files=self.files())

    async def start(self):
        async with self.lock:
            if self.job or self.phase not in ('idle', 'error'):
                raise RuntimeError('已有录制或待保存视频，请先停止并保存。')
            _, state = await self.adb('get-state')
            if state != 'device':
                raise RuntimeError('请连接 Quest USB 并允许调试。')
            # Do not interfere with a system/user-owned capture.
            _, pids = await self.adb('shell', 'pidof screenrecord', check=False)
            if pids.strip():
                raise RuntimeError('Quest 上已有录屏进程，请先结束原录制。')
            _, power = await self.adb('shell', 'dumpsys power')
            if 'mWakefulness=Asleep' in power or 'mWakefulness=Dozing' in power:
                raise RuntimeError('Quest 正在休眠，请戴上头显、进入 HTS 后再开始录制。')
            if shutil.disk_usage(self.output).free < 1024**3:
                raise RuntimeError('ARX 剩余空间不足 1 GB，请先整理视频。')
            await self.adb('shell', 'mkdir -p ' + shlex.quote(REMOTE))
            name = datetime.now().strftime('hts_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8] + '.mp4'
            self.job = dict(name=name, remote=REMOTE+'/'+name, started_at=time.time(), pid=None)
            write_json(self.active_file, self.job)
            self.phase, self.error = 'starting', None
            command = ('echo HTS_REC_PID:$$; exec screenrecord --size 1280x720 '
                       '--bit-rate 8000000 --time-limit 0 ' + shlex.quote(self.job['remote']))
            try:
                self.process = await asyncio.create_subprocess_exec(
                    'adb', '-s', self.serial, 'shell', command,
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
                line = await asyncio.wait_for(self.process.stdout.readline(), 5)
                match = re.fullmatch(rb'HTS_REC_PID:(\d+)\r?\n', line)
                if not match:
                    raise RuntimeError('录屏启动失败：' + line.decode(errors='replace').strip())
                self.job['pid'] = int(match[1])
                write_json(self.active_file, self.job)
                self.phase = 'recording'
                self.watcher = asyncio.create_task(self.watch())
            except BaseException:
                self.phase, self.error = 'recoverable', '启动未完成，请点击重试保存检查录制。'
                if self.process and self.process.returncode is None:
                    self.process.terminate()
                    await self.process.communicate()
                raise

    async def signal_owned(self):
        """Only SIGINT the recorded PID when its full output path still matches."""
        pid = self.job.get('pid')
        if not pid:
            return
        target = shlex.quote(self.job['remote'])
        # The device-side check and signal avoid a stale local pidof snapshot.
        command = (f'if [ -r /proc/{int(pid)}/cmdline ]; then '
                   f"tr '\\000' '\\n' < /proc/{int(pid)}/cmdline | grep -Fxq {target} "
                   f'&& kill -2 {int(pid)}; fi')
        await self.adb('shell', command, check=False)

    async def stop(self):
        async with self.lock:
            if self.phase in ('stopping', 'saving'):
                return
            if self.phase != 'recording' or not self.job:
                raise RuntimeError('当前没有正在录制的视频。')
            await self.signal_owned()
            self.phase = 'stopping'

    async def watch(self):
        out, _ = await self.process.communicate()
        async with self.lock:
            try:
                await self.save()
            except (OSError, RuntimeError, asyncio.TimeoutError, ValueError) as error:
                self.fail(error)
                if out:
                    print('SCREENRECORD:', out.decode(errors='replace')[-1000:], flush=True)

    def fail(self, error):
        self.error = str(error) or '保存超时，请连接 Quest 后重试保存。'
        if isinstance(error, EmptyRecordingError):
            # An empty failed attempt contains no video to recover. Keep its diagnostic record.
            write_json(self.output / (self.job['name'] + '.failed.json'), dict(self.job, error=self.error))
            self.active_file.unlink(missing_ok=True)
            self.job, self.phase = None, 'error'
        else:
            self.phase = 'recoverable'

    async def wait_device_recording_end(self):
        pid = self.job.get('pid')
        if not pid:
            return
        # A USB failure can leave the remote process running; never pull a live MP4.
        for _ in range(30):
            _, cmd = await self.adb('shell', f'cat /proc/{int(pid)}/cmdline', check=False)
            if self.job['remote'] not in cmd.split('\x00'):
                return
            await asyncio.sleep(.25)
        raise RuntimeError('Quest 录屏仍在收尾，请稍后重试保存。')

    async def save(self):
        self.phase, self.error = 'saving', None
        await self.wait_device_recording_end()
        _, size = await self.adb('shell', 'stat -c %s ' + shlex.quote(self.job['remote']))
        if size.strip() == '0':
            raise EmptyRecordingError('Quest 没有输出画面；请戴上头显、打开 HTS 后重新录制。')
        name = self.job['name']
        partial = self.output / (name + '.partial')
        # Transfer happens only after capture finishes, not in the teleop loop.
        await self.adb('pull', self.job['remote'], str(partial), timeout=600)
        await self.prepare_video(partial)
        _, info = await self.run('ffprobe', '-v', 'error', '-select_streams', 'v:0',
                                '-show_entries', 'stream=codec_name,width,height:format=duration',
                                '-of', 'json', str(partial), timeout=20)
        probe = json.loads(info)
        duration = float(probe.get('format', {}).get('duration', 0))
        if not probe.get('streams') or duration <= 0:
            raise RuntimeError('视频没有有效画面，原始文件仍保留在 Quest。')
        metadata = dict(name=name, duration_s=duration, created_at=self.job['started_at'],
                        **probe['streams'][0])
        metadata.update(self.capture_metadata())
        write_json(self.output / name.replace('.mp4', '.json'), metadata)
        partial.replace(self.output / name)
        # The ARX copy is verified before removing our temporary headset copy.
        with contextlib.suppress(OSError, RuntimeError, asyncio.TimeoutError):
            await self.adb('shell', 'rm -f ' + shlex.quote(self.job['remote']))
        self.active_file.unlink(missing_ok=True)
        self.job, self.phase, self.error = None, 'idle', None

    async def prepare_video(self, path):
        pass

    def capture_metadata(self):
        return dict(capture_backend='adb-screenrecord', view='raw-stereo')

    async def retry(self):
        async with self.lock:
            if self.phase != 'recoverable' or not self.job:
                raise RuntimeError('没有待恢复的视频。')
            await self.signal_owned()
            self.phase, self.error = 'saving', None
            self.watcher = asyncio.create_task(self.recover())

    async def recover(self):
        async with self.lock:
            try:
                await self.save()
            except (OSError, RuntimeError, asyncio.TimeoutError, ValueError) as error:
                self.fail(error)

    async def close(self):
        if self.health:
            self.health.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.health
        if self.phase == 'recording':
            with contextlib.suppress(OSError, RuntimeError, asyncio.TimeoutError):
                await self.stop()
        if self.watcher:
            with contextlib.suppress(asyncio.TimeoutError):
                await asyncio.wait_for(asyncio.shield(self.watcher), 30)


def application(recorder):
    app = web.Application(client_max_size=4096)

    async def page(request):
        return web.FileResponse(ROOT / 'web/recording.html', headers={'Cache-Control': 'no-store'})

    async def state(request):
        return web.json_response(recorder.snapshot(), headers={'Cache-Control': 'no-store'})

    async def control(request):
        # Browser writes must originate from this local recorder page.
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='Origin rejected')
        if request.content_type != 'application/json':
            raise web.HTTPUnsupportedMediaType()
        try:
            data = await request.json()
            if not isinstance(data, dict) or data.get('source') != 'quest-recording':
                raise ValueError('Recording control required')
            action = data.get('action')
            if action not in ('start', 'stop', 'retry'):
                raise ValueError('Unknown recording action')
            await getattr(recorder, action)()
            return web.json_response(recorder.snapshot())
        except (ValueError, OSError, RuntimeError, asyncio.TimeoutError) as error:
            return web.json_response(dict(error=str(error) or 'Quest 请求超时'), status=409)

    async def video(request):
        name = request.match_info['name']
        path = recorder.output / name
        if not NAME.fullmatch(name) or not path.is_file() or path.is_symlink():
            raise web.HTTPNotFound()
        headers = {'Content-Type': 'video/mp4'}
        if request.query.get('download') == '1':
            headers['Content-Disposition'] = f'attachment; filename="{name}"'
        return web.FileResponse(path, headers=headers)

    async def start(app):
        recorder.health = asyncio.create_task(recorder.device_health())

    async def stop(app):
        await recorder.close()

    app.router.add_get('/', page)
    app.router.add_get('/state', state)
    app.router.add_post('/control', control)
    app.router.add_get('/videos/{name}', video)
    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8044)
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT / 'service.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        from quest_native_capture import NativeRecorder
        web.run_app(application(NativeRecorder()), host='127.0.0.1', port=args.port, shutdown_timeout=35)
