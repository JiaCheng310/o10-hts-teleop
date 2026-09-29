import struct
import tempfile
import unittest

from quest_native_capture import BACKEND, DIRECTORY, PROPERTIES, NativeRecorder


class NativeCaptureTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.rec = NativeRecorder(self.temp.name)

    async def asyncTearDown(self):
        self.temp.cleanup()

    async def test_mp4_requires_complete_moov_not_stable_size(self):
        data = struct.pack('>I4s', 16, b'ftyp') + b'12345678' + struct.pack('>I4s', 0, b'mdat') + b'data'
        async def adb(*args, **kwargs):
            return 0, str(len(data))
        async def header(path, offset):
            return data[offset:offset+16]
        self.rec.adb, self.rec.header = adb, header
        self.assertFalse(await self.rec.finalized('test.mp4'))
        data = data[:16] + struct.pack('>I4s', 12, b'mdat') + b'data' + struct.pack('>I4s', 12, b'moov') + b'info'
        self.assertTrue(await self.rec.finalized('test.mp4'))
        data = data[:-2]
        self.assertFalse(await self.rec.finalized('test.mp4'))

    async def test_new_foreign_capture_not_stopped(self):
        mine, other = DIRECTORY+'/mine.mp4', DIRECTORY+'/other.mp4'
        self.rec.job = dict(backend=BACKEND, remote=mine)
        async def finalized(path): return False
        async def videos(): return {mine:dict(modified=1), other:dict(modified=2)}
        async def action(name): self.fail('Foreign capture must not be stopped')
        self.rec.finalized, self.rec.videos, self.rec.capture_action = finalized, videos, action
        with self.assertRaisesRegex(RuntimeError, '不匹配'):
            await self.rec.signal_owned()

    async def test_own_capture_is_stopped_and_finalized_capture_is_left_alone(self):
        mine = DIRECTORY+'/mine.mp4'
        self.rec.job = dict(backend=BACKEND, remote=mine)
        is_final = False
        calls = []
        async def finalized(path): return is_final
        async def videos(): return {mine:dict(modified=1)}
        async def action(name): calls.append(name)
        self.rec.finalized, self.rec.videos, self.rec.capture_action = finalized, videos, action
        await self.rec.signal_owned()
        is_final = True
        await self.rec.signal_owned()
        self.assertEqual(calls, ['STOP_INTERNAL_CAPTURE_TO_DISK'])

    async def test_property_restore_preserves_user_changes(self):
        self.rec.job = dict(original_properties={
            'debug.oculus.screenCaptureEye':'', 'debug.oculus.capture.width':'720'})
        writes = []
        async def adb(*args, **kwargs):
            if args[1] == 'getprop':
                return 0, '1' if args[2].endswith('Eye') else '2048'
            writes.append(args[1]); return 0, ''
        self.rec.adb = adb
        await self.rec.restore_properties()
        self.assertEqual(writes, ["setprop debug.oculus.screenCaptureEye ''"])

    async def test_ambiguous_new_files_do_not_get_claimed(self):
        self.rec.job = dict(remote=None, baseline=[])
        async def videos(): return {'one.mp4':{}, 'two.mp4':{}}
        self.rec.videos = videos
        with self.assertRaisesRegex(RuntimeError, '多个'):
            await self.rec.locate_job_file()
        self.assertIsNone(self.rec.job['remote'])

    async def test_user_settings_do_not_include_tracking_or_guardian(self):
        self.assertTrue(all(k.startswith('debug.oculus.capture.') or k in (
            'debug.oculus.screenCaptureEye', 'debug.oculus.fullRateCapture') for k in PROPERTIES))
        state = self.rec.snapshot()
        self.assertEqual(state['settings']['view'], 'undistorted-mono')
        self.assertEqual(state['settings']['eye'], 'right')


if __name__ == '__main__':
    unittest.main()
