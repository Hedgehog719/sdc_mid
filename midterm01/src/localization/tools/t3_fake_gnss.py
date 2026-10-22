#!/usr/bin/env python3
"""GNSS-free runs (Track03): sensors/gnss.csv holding only the initial pose (first second), for fuse.py
--gnss_keep_s (initial position / heading of the smoothers); all later positions come from the map fixes.
usage: t3_fake_gnss.py <track_dir> <pass1.csv | x,y,yaw> [--seconds 1.0]"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('pose')
ap.add_argument('--seconds', type=float, default=1.0)
a = ap.parse_args()
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
ts = np.asarray(ts, np.int64)
if os.path.exists(a.pose):
    P = pd.read_csv(a.pose)
    k = ts <= ts[0] + a.seconds * 1e9
    x, y, yaw = P.x.values[:len(ts)][k], P.y.values[:len(ts)][k], P.yaw.values[:len(ts)][k]
    t = ts[k]
else:
    x0, y0, yaw0 = (float(v) for v in a.pose.split(','))
    t = np.linspace(ts[0], ts[0] + a.seconds * 1e9, 5).astype(np.int64)
    x, y, yaw = np.full(5, x0), np.full(5, y0), np.full(5, yaw0)
os.makedirs(os.path.join(a.track_dir, 'sensors'), exist_ok=True)
pd.DataFrame({'t': t, 'stamp': t, 'x': x, 'y': y, 'z': np.zeros(len(t)), 'qx': 0.0, 'qy': 0.0, 'qz': np.sin(yaw / 2),
              'qw': np.cos(yaw / 2), 'frame': 'fake_init'}).to_csv(os.path.join(a.track_dir, 'sensors', 'gnss.csv'), index=False)
print(f'{a.track_dir}/sensors/gnss.csv: {len(t)} rows (initial pose only)')
