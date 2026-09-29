import asyncio
import time
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from aiohttp.test_utils import TestClient, TestServer
from hts_input import HTSParser
from hts_service import HTSService, application
from mapping import O10Retargeter, palm_coordinates

def complete_fixture(rt):
    # The robot FK helper only supplies bases/tips. Fill intermediate joints
    # for a complete synthetic protocol fixture; never send this to hardware.
    p=rt.reference_from_active(np.array([0,.6,-.5,0,.5,.5,0,.5,0,.5]))
    p[1]=p[2]/2;p[3]=(p[2]+p[4])/2
    for b in (5,9,13,17):
        p[b+1]=(2*p[b]+p[b+3])/3
        p[b+2]=(p[b]+2*p[b+3])/3
    return p

def packet(points, fid=1, ns=1_000_000_000, side='Left', metadata=True):
    suffix=f' | f = {fid} | t = {ns}' if metadata else ''
    unity=np.asarray(points)*[1,1,-1]
    values=', '.join(str(v) for v in unity.ravel())
    return (f'{side} wrist{suffix}:, 0, 0, 0, 0, 0, 0, 1\n'
            f'{side} landmarks{suffix}:, {values}\n').encode()

class ParserTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rt=O10Retargeter('left','dexpilot',alpha=1)
        cls.points=complete_fixture(rt)
    def parse(self, parser, blob, now=2.):
        out=None
        for line in blob.splitlines():
            value=parser.feed(line,now)
            if value is not None:out=value
        return out
    def test_unity_reflection_preserves_left_curl_and_palm_axes(self):
        rotation=Rotation.from_euler('xyz',[.4,-.8,.3]).as_matrix()
        points=self.points@rotation.T
        result=self.parse(HTSParser(),packet(points))
        np.testing.assert_allclose(result.points,points,atol=1e-9)
        np.testing.assert_allclose(palm_coordinates(result.points,'left'),palm_coordinates(self.points,'left'),atol=1e-8)
    def test_wrong_hand_and_head_are_ignored(self):
        p=HTSParser()
        self.assertIsNone(self.parse(p,packet(self.points,side='Right')))
        self.assertIsNone(p.feed(b'Head pose:, 0,0,0,0,0,0,1',2))
    def test_timestamp_required_and_sections_must_match(self):
        p=HTSParser()
        with self.assertRaisesRegex(ValueError,'Debug Info'):
            self.parse(p,packet(self.points,metadata=False))
        p.feed(packet(self.points,fid=1).splitlines()[0],2)
        with self.assertRaisesRegex(ValueError,'mismatched'):
            p.feed(packet(self.points,fid=2).splitlines()[1],2)
    def test_replay_and_transport_backlog_drop_then_fresh_recovers(self):
        p=HTSParser();self.parse(p,packet(self.points),2)
        with self.assertRaisesRegex(ValueError,'Repeated'):
            self.parse(p,packet(self.points),2.01)
        with self.assertRaisesRegex(ValueError,'Stale'):
            self.parse(p,packet(self.points,2,1_020_000_000),2.5)
        result=self.parse(p,packet(self.points,3,1_600_000_000),2.6)
        self.assertEqual(result.frame_id,3)
        self.assertAlmostEqual(result.lag_ms,0,places=5)
    def test_incomplete_geometry_and_nonfinite_values_rejected(self):
        points=self.points.copy();points[8]=points[7]
        with self.assertRaisesRegex(ValueError,'Incomplete HTS finger'):
            self.parse(HTSParser(),packet(points))
        points[8,0]=float('nan')
        with self.assertRaisesRegex(ValueError,'Nonfinite'):
            self.parse(HTSParser(),packet(points))
    def test_landmarks_cannot_reuse_a_wrist_after_bad_number(self):
        p=HTSParser();p.feed(packet(self.points).splitlines()[0],2)
        with self.assertRaises(ValueError):p.feed(b'Left landmarks | f = 1 | t = 1000000000:, oops',2)
        with self.assertRaisesRegex(ValueError,'mismatched'):
            p.feed(packet(self.points).splitlines()[1],2)

class ServiceTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.service=HTSService(False)
        self.client=TestClient(TestServer(application(self.service,0)))
        await self.client.start_server()
        self.port=self.service.tcp_server.sockets[0].getsockname()[1]
        self.points=complete_fixture(self.service.rt)
        self.writers=[];self.fid=0
    async def asyncTearDown(self):
        for writer in self.writers:writer.close()
        await self.client.close()
    async def connection(self):
        _,writer=await asyncio.open_connection('127.0.0.1',self.port)
        self.writers.append(writer);return writer
    async def send(self,writer,side='Left',split=False):
        self.fid+=1
        data=packet(self.points,self.fid,time.monotonic_ns(),side)
        if split:
            writer.write(data[:37]);await writer.drain();await asyncio.sleep(.005)
            writer.write(data[37:])
        else:writer.write(data)
        await writer.drain();await asyncio.sleep(.04)
    async def test_fragmented_frame_loss_resume_and_manual_pause(self):
        w=await self.connection();await self.send(w,split=True)
        self.assertTrue(self.service.output_enabled)
        await asyncio.sleep(.24)
        self.assertFalse(self.service.output_enabled);self.assertTrue(self.service.follow)
        await self.send(w);self.assertTrue(self.service.output_enabled)
        await self.client.post('/control',json={'source':'hts','action':'pause'})
        await self.send(w);self.assertFalse(self.service.output_enabled)
        await self.client.post('/control',json={'source':'hts','action':'enable'})
        await self.send(w);self.assertTrue(self.service.output_enabled)
        w.close();await asyncio.sleep(.03);self.assertFalse(self.service.output_enabled)
    async def test_probe_right_hand_old_page_cannot_control_active_left(self):
        w=await self.connection();await self.send(w)
        owner=self.service.owner_writer
        probe=await self.connection();probe.close()
        right=await self.connection();await self.send(right,'Right');right.close()
        reply=await self.client.post('/control',json={'version':3,'action':'pause'})
        self.assertEqual(reply.status,409)
        self.assertIs(self.service.owner_writer,owner);self.assertTrue(self.service.output_enabled)
        self.assertEqual((await self.client.get('/ws?v=3')).status,404)
    async def test_slow_mapping_invalidated_on_disconnect(self):
        w=await self.connection();original=self.service.rt.retarget
        def slow(p):time.sleep(.14);return original(p)
        with patch.object(self.service.rt,'retarget',slow):
            await self.send(w)
            started=time.monotonic();await self.client.get('/state')
            self.assertLess(time.monotonic()-started,.08)
            w.close();await asyncio.sleep(.17)
            self.assertFalse(self.service.output_enabled);self.assertIsNone(self.service.angles)
    async def test_no_timestamp_never_enables_output(self):
        w=await self.connection();w.write(packet(self.points,metadata=False));await w.drain()
        await asyncio.sleep(.03)
        self.assertFalse(self.service.output_enabled)
        self.assertIn('Debug Info',self.service.last_input_error)
    async def test_queued_burst_keeps_only_latest_frame(self):
        w=await self.connection();ns=time.monotonic_ns()
        w.write(b''.join(packet(self.points,i,ns+i) for i in range(1,6)))
        await w.drain();await asyncio.sleep(.04)
        self.assertEqual(self.service.last_source_id,5)
        self.assertGreaterEqual(self.service.dropped,4)
        self.assertEqual(self.service.frames,1)
    async def test_new_left_connection_replaces_owner_without_old_disconnect_stopping_it(self):
        first=await self.connection();await self.send(first)
        old_owner=self.service.owner_writer
        second=await self.connection();await self.send(second)
        self.assertIsNot(self.service.owner_writer,old_owner)
        first.close();await asyncio.sleep(.02)
        self.assertTrue(self.service.output_enabled)

if __name__=='__main__':unittest.main()
