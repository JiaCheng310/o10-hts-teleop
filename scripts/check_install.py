"""Validate SDK imports and both O10 mappings without connecting hardware."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/dex-retargeting/src')]

import numpy as np
import omnihand
from o10teleop.core import O10Retargeter

for side in ('left', 'right'):
    retargeter = O10Retargeter(side, 'dexpilot', alpha=1)
    active = retargeter.limits.mean(axis=1)
    full = retargeter.expand(active)
    assert len(active) == 10 and np.isfinite(full).all()
    points = retargeter.reference_from_active(active)
    _, result = retargeter.retarget(points)
    assert result.shape == (10,) and np.isfinite(result).all()
    assert np.all(result >= retargeter.limits[:, 0] - 1e-6)
    assert np.all(result <= retargeter.limits[:, 1] + 1e-6)
    print(f'{side}: DexPilot + O10 coupling + 10 SDK joints OK')
print('SDK import OK. No device was opened and no motor command was sent.')
