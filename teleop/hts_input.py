"""HTS v1.1 CSV input, based on the pinned upstream HandLandmarkStreamer.cs.

The landmarks are wrist-local Unity coordinates, not world coordinates.
Reflect Z to obtain a right-handed frame before our palm normalization.
"""
import re
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

UNITY_TO_RH = np.diag([1., 1., -1.])

def world_pose(values):
    """Change handedness of both position and rotation (S R S)."""
    values = np.asarray(values, float)
    if values.shape != (7,) or not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1) > .05:
        raise ValueError("Invalid HTS world pose")
    return UNITY_TO_RH @ values[:3], UNITY_TO_RH @ Rotation.from_quat(values[3:]).as_matrix() @ UNITY_TO_RH


HEADER = re.compile(r'^(Left|Right) (wrist|landmarks)(?:\s*\|\s*f\s*=\s*(\d+)\s*\|\s*t\s*=\s*(\d+))?:\s*,\s*(.*)$')

@dataclass
class HandFrame:
    points: np.ndarray
    frame_id: int | None
    source_ns: int | None
    captured: float
    lag_ms: float
    wrist_position: np.ndarray | None = None
    wrist_rotation: np.ndarray | None = None

class HTSParser:
    def __init__(self, require_metadata=True, side='left'):
        if side not in ('left', 'right'):
            raise ValueError('Invalid hand side')
        self.side = side.title()
        self.require_metadata = require_metadata
        self.pending = None
        self.last_id = None
        self.last_ns = None
        self.min_offset = None

    def feed(self, line, now):
        text = line.decode('utf-8').strip()
        other = 'Left ' if self.side == 'Right' else 'Right '
        if not text or text.startswith(('Head ', other)):
            return None
        match = HEADER.fullmatch(text)
        if not match:
            self.pending = None
            raise ValueError('Invalid HTS line')
        side, kind, frame_id, source_ns, payload = match.groups()
        if side != self.side:
            return None
        meta = (int(frame_id), int(source_ns)) if frame_id is not None else (None, None)
        if self.require_metadata and meta[0] is None:
            self.pending = None
            raise ValueError('请在 HTS 中开启 Debug Info（帧序号和时间戳）')
        try:
            values = np.array([float(x.strip()) for x in payload.split(',')])
        except ValueError:
            self.pending = None
            raise ValueError('Invalid numeric HTS data')
        if not np.isfinite(values).all():
            self.pending = None
            raise ValueError('Nonfinite HTS data')
        if kind == 'wrist':
            if values.shape != (7,) or abs(np.linalg.norm(values[3:])-1) > .05:
                self.pending = None
                raise ValueError('Invalid HTS wrist pose')
            self.pending = (meta, now, *world_pose(values))
            return None
        pending, self.pending = self.pending, None
        if values.shape != (63,) or pending is None or pending[0] != meta or now-pending[1] > .2:
            raise ValueError('Incomplete or mismatched HTS frame')
        points = values.reshape(21, 3) * np.array([1., 1., -1.])
        lengths = np.linalg.norm(points[[5,9,17]]-points[0], axis=1)
        if np.any(lengths < .025) or np.any(lengths > .2):
            raise ValueError('Invalid hand geometry or units')
        # Reject zero-filled/missing fingertips instead of moving on partial input.
        for start in (1,5,9,13,17):
            segments = np.linalg.norm(np.diff(points[start:start+4], axis=0), axis=1)
            if np.any(segments < .001) or np.any(segments > .1):
                raise ValueError('Incomplete HTS finger joints')
        lag = 0.
        if meta[0] is not None:
            fid, ns = meta
            if ns <= 0 or (self.last_id is not None and (fid <= self.last_id or ns <= self.last_ns)):
                raise ValueError('Repeated or reversed HTS frame')
            self.last_id, self.last_ns = fid, ns
            offset = now - ns / 1e9
            self.min_offset = offset if self.min_offset is None else min(self.min_offset, offset)
            # Excess transport delay relative to the best arrival on this socket;
            # this is not an absolute clock-synchronized end-to-end latency.
            lag = max(0., offset-self.min_offset)
            if lag > .2:
                raise ValueError('Stale HTS frame')
        return HandFrame(points, meta[0], meta[1], now-lag, lag*1000, pending[2], pending[3])


@dataclass
class HeadFrame:
    position: np.ndarray
    rotation: np.ndarray
    captured: float
    source_ns: int


class HeadParser:
    header = re.compile(r'^Head pose\s*\|\s*f\s*=\s*(\d+)\s*\|\s*t\s*=\s*(\d+):\s*,\s*(.*)$')

    def __init__(self):
        self.last_id = self.last_ns = self.min_offset = None

    def feed(self, line, now):
        text = line.decode('utf-8').strip()
        if not text.startswith('Head '):
            return None
        match = self.header.fullmatch(text)
        if not match:
            raise ValueError('Head Pose 需要开启 Debug Info')
        fid, ns = map(int, match.groups()[:2])
        if ns <= 0 or (self.last_id is not None and (fid <= self.last_id or ns <= self.last_ns)):
            raise ValueError('Repeated or reversed HTS head frame')
        p, r = world_pose([float(x.strip()) for x in match.group(3).split(',')])
        self.last_id, self.last_ns = fid, ns
        offset = now - ns / 1e9
        self.min_offset = offset if self.min_offset is None else min(self.min_offset, offset)
        lag = max(0., offset-self.min_offset)
        if lag > .2:
            raise ValueError('Stale HTS head frame')
        return HeadFrame(p, r, now-lag, ns)
