"""RIGHT HTS -> real DexPilot -> HTTP -> simulated SDK; no physical outputs."""
import asyncio
import json
import os
from unittest.mock import patch
from pathlib import Path
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
import unittest

import numpy as np
from aiohttp.test_utils import TestClient, TestServer
from direct_sender import DirectSender
from hts_input import HTSParser
from hts_service import HTSService, application
from test_hts import packet

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'teleop/driver'))
from quest_usb_bridge import Bridge, BridgeError


class FakeHand:
    def __init__(self):
        self.angles = [0.1, -.8, .4, -.05, .7, .6, .07, .7, .1, .8]
        self.targets = []

    def get_all_error_reports(self):
        return [SimpleNamespace(stalled=False, overheat=False, over_current=False,
                                motor_except=False, commu_except=False) for _ in range(10)]

    def get_all_active_joint_angles(self):
        return self.angles

    def set_all_active_joint_angles(self, angles):
        self.targets.append(angles)
        self.angles = angles


def fixture(rt, active=None):
    active = np.array([.1, -.8, .4, -.05, .7, .6, .07, .7, .1, .8]) if active is None else active
    points = rt.reference_from_active(active)
    points[1] = points[2] / 2
    points[3] = (points[2] + points[4]) / 2
    for b in (5, 9, 13, 17):
        points[b+1] = (2*points[b] + points[b+3])/3
        points[b+2] = (points[b] + 2*points[b+3])/3
    return points


class RightPipelineTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.cfg = json.loads((ROOT / 'teleop/driver/right_limits.json').read_text())
        self.hand = FakeHand()
        self.bridge = Bridge(self.hand, 'TEST_RIGHT_SERIAL', self.cfg)
        self.reply_side = 'right'
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def respond(self, data):
                data['hand'] = outer.reply_side
                raw = json.dumps(data).encode()
                self.send_response(200)
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                self.respond(outer.bridge.snapshot(outer.bridge.read()))

            def do_POST(self):
                data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                self.respond(outer.bridge.target(data))

            def log_message(self, *args):
                pass

        self.http = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.http.daemon_threads = True
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        with patch.dict(os.environ, {'O10_EXPECTED_SERIAL': 'TEST_RIGHT_SERIAL'}):
            self.service = HTSService(False, side='right')
        self.service.sender = DirectSender(self.service.rt.names,
            f'http://127.0.0.1:{self.http.server_port}', 'TEST_RIGHT_SERIAL', side='right')
        self.client = TestClient(TestServer(application(self.service, 0)))
        await self.client.start_server()
        port = self.service.tcp_server.sockets[0].getsockname()[1]
        _, self.writer = await asyncio.open_connection('127.0.0.1', port)
        self.points = fixture(self.service.rt)

    async def asyncTearDown(self):
        self.writer.close()
        await self.client.close()
        await asyncio.to_thread(self.http.shutdown)
        self.http.server_close()

    async def wait_until(self, predicate):
        for _ in range(100):
            if predicate():
                return
            await asyncio.sleep(.01)
        self.fail('Timed out waiting for test pipeline')

    async def test_right_frame_reaches_right_sdk_with_valid_right_limits(self):
        self.writer.write(packet(self.points, side='Right'))
        await self.writer.drain()
        await self.wait_until(lambda: bool(self.hand.targets))
        state = self.service.snapshot()
        self.assertEqual(state['side'], 'right')
        self.assertEqual(state['hardware_usb_serial'], 'TEST_RIGHT_SERIAL')
        self.assertTrue(all(n.startswith('R_') for n in self.service.rt.names))
        target = np.array(self.hand.targets[-1])
        limits = np.array(self.cfg['limits_rad'])
        self.assertTrue(np.all(target >= limits[:, 0]-1e-6))
        self.assertTrue(np.all(target <= limits[:, 1]+1e-6))
        self.assertLessEqual(target[1], 0.046)
        self.assertGreaterEqual(target[2], 0)
        await asyncio.sleep(.25)
        self.assertFalse(self.service.output_enabled)

    async def test_left_frame_is_ignored_and_does_not_move_right_hand(self):
        self.writer.write(packet(self.points, side='Left'))
        await self.writer.drain()
        await asyncio.sleep(.25)
        self.assertEqual(self.service.frames, 0)
        self.assertFalse(self.service.output_enabled)
        self.assertEqual(self.hand.targets, [])

    async def test_wrong_driver_handedness_faults_without_commands(self):
        self.reply_side = 'left'
        await self.wait_until(lambda: bool(self.service.sender.fault))
        self.assertIn('IDENTITY_MISMATCH', self.service.sender.fault)
        self.assertEqual(self.hand.targets, [])

    async def test_parser_preserves_right_wrist_local_coordinates(self):
        parser = HTSParser(side='right')
        result = None
        for line in packet(self.points, side='Right').splitlines():
            result = parser.feed(line, 2.0)
        np.testing.assert_allclose(result.points, self.points)

    async def test_driver_rejects_left_identity_before_sdk_write(self):
        state = self.bridge.snapshot(self.bridge.read())
        payload = {'hand': 'left', 'joint_names': self.cfg['joint_names'], 'enabled': True,
                   'nonce': state['nonce'], 'angles': self.hand.angles, 'freshness_budget_s': .2}
        with self.assertRaises(BridgeError) as error:
            self.bridge.target(payload)
        self.assertEqual(error.exception.code, 'IDENTITY_MISMATCH')
        self.assertEqual(self.hand.targets, [])


if __name__ == '__main__':
    unittest.main()
