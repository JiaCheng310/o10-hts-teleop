import asyncio
import json
from pathlib import Path
import tempfile
import unittest

from aiohttp.test_utils import TestClient, TestServer
from quest_recording import EmptyRecordingError, Recorder, application, write_json

NAME = 'hts_20260923_171500_1234abcd.mp4'


class RecorderTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.rec = Recorder(self.tmp.name)

    async def asyncTearDown(self):
        self.tmp.cleanup()

    async def test_pending_recording_survives_restart(self):
        write_json(self.rec.active_file, dict(name=NAME, remote='/sdcard/Movies/HTS/'+NAME,
                                             pid=123, started_at=1))
        recovered = Recorder(self.tmp.name)
        self.assertEqual(recovered.phase, 'recoverable')
        with self.assertRaisesRegex(RuntimeError, '待保存'):
            await recovered.start()

    async def test_stop_signals_only_owned_pid_and_path(self):
        self.rec.job = dict(pid=123, remote='/sdcard/Movies/HTS/'+NAME, started_at=1)
        self.rec.phase = 'recording'
        calls = []
        async def adb(*args, **kwargs):
            calls.append(args)
            return 0, ''
        self.rec.adb = adb
        await self.rec.stop()
        await self.rec.stop()  # double click is harmless
        self.assertEqual(len(calls), 1)
        self.assertIn('kill -2 123', calls[0][1])
        self.assertIn('grep -Fxq /sdcard/Movies/HTS/'+NAME, calls[0][1])
        self.assertNotIn('pkill', calls[0][1])

    async def test_save_validates_before_publishing_and_removing_device_copy(self):
        self.rec.job = dict(name=NAME, remote='/sdcard/Movies/HTS/'+NAME, started_at=1)
        write_json(self.rec.active_file, self.rec.job)
        calls = []
        async def adb(*args, **kwargs):
            calls.append(args)
            if args[0] == 'pull':
                Path(args[2]).write_bytes(b'test-video')
            return 0, ''
        async def invalid(*args, **kwargs):
            return 0, json.dumps(dict(streams=[], format=dict(duration='0')))
        self.rec.adb, self.rec.run = adb, invalid
        with self.assertRaisesRegex(RuntimeError, '有效画面'):
            await self.rec.save()
        self.assertFalse((Path(self.tmp.name)/NAME).exists())
        self.assertTrue(self.rec.active_file.exists())
        self.assertFalse(any('rm -f' in str(c) for c in calls))
        async def valid(*args, **kwargs):
            return 0, json.dumps(dict(streams=[dict(codec_name='h264', width=1280, height=720)],
                                      format=dict(duration='3.25')))
        self.rec.run = valid
        await self.rec.save()
        self.assertEqual(self.rec.files()[0]['duration_s'], 3.25)
        self.assertTrue((Path(self.tmp.name)/NAME).is_file())
        self.assertFalse(self.rec.active_file.exists())
        self.assertTrue(any('rm -f' in str(c) for c in calls))

    async def test_api_blocks_cross_origin_write_and_unlisted_files(self):
        async def health():
            await asyncio.Future()
        self.rec.device_health = health
        client = TestClient(TestServer(application(self.rec)))
        await client.start_server()
        try:
            response = await client.post('/control', json=dict(source='quest-recording', action='start'),
                                         headers={'Origin':'https://unrelated.invalid'})
            self.assertEqual(response.status, 403)
            response = await client.get('/videos/active.json')
            self.assertEqual(response.status, 404)
            path = Path(self.tmp.name)/NAME
            path.symlink_to('/etc/hostname')
            response = await client.get('/videos/'+NAME)
            self.assertEqual(response.status, 404)
            response = await client.post('/control', json=dict(source='quest-recording', action='shell'))
            self.assertEqual(response.status, 409)
        finally:
            await client.close()

    async def test_foreign_recording_is_not_stopped(self):
        async def adb(*args, **kwargs):
            if args == ('get-state',):
                return 0, 'device'
            if args == ('shell', 'pidof screenrecord'):
                return 0, '999'
            self.fail('No further device commands allowed')
        self.rec.adb = adb
        with self.assertRaisesRegex(RuntimeError, '已有录屏'):
            await self.rec.start()
        self.assertIsNone(self.rec.job)

    async def test_sleeping_headset_fails_before_creating_a_job(self):
        async def adb(*args, **kwargs):
            return 0, {('get-state',): 'device', ('shell', 'pidof screenrecord'): '',
                       ('shell', 'dumpsys power'): '  mWakefulness=Asleep'}[args]
        self.rec.adb = adb
        with self.assertRaisesRegex(RuntimeError, '休眠'):
            await self.rec.start()
        self.assertIsNone(self.rec.job)

    async def test_empty_failed_capture_does_not_block_new_recording(self):
        self.rec.job = dict(name=NAME, remote='/sdcard/Movies/HTS/'+NAME, started_at=1)
        write_json(self.rec.active_file, self.rec.job)
        self.rec.fail(EmptyRecordingError('未录到画面'))
        self.assertIsNone(self.rec.job)
        self.assertEqual(self.rec.phase, 'error')
        self.assertFalse(self.rec.active_file.exists())
        self.assertTrue((Path(self.tmp.name)/(NAME+'.failed.json')).exists())


if __name__ == '__main__':
    unittest.main()
