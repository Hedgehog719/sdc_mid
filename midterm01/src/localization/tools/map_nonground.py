#!/usr/bin/env python3
"""Sub-sampled non-ground map points (height above ground > 0.3 m) for occlusion reasoning (z-buffer)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from bev import GroundRaster  # noqa: E402

track = sys.argv[1]
keep_frac = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
xyz = np.load(os.path.join(track, 'map.npy'), mmap_mode='r')
G = GroundRaster(xyz)
rng = np.random.default_rng(0)
out = []
for s in range(0, len(xyz), 5_000_000):
    p = np.asarray(xyz[s:s + 5_000_000])
    p = p[rng.random(len(p)) < keep_frac]
    hag = p[:, 2] - G.height(p[:, 0], p[:, 1])
    out.append(p[(hag > 0.3) & (hag < 25)])
pts = np.concatenate(out).astype(np.float32)
np.save(os.path.join(track, 'map_nonground.npy'), pts)
print(track, 'non-ground points kept', len(pts))
