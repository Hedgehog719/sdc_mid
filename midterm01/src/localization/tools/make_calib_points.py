#!/usr/bin/env python3
"""Precompute map lane-marking points for the manual camera-calibration UI (pole_review_server /calib).

For selected frames of a track with an accurate trajectory, map road-marking cells (vehicle frame, on the road
plane) are projected into each camera with the current calibration. Saved per frame/camera:
work/calib_pts/<Track>_<frame>_<cam>.json with ids, 3D vehicle-frame points and their pixel positions.
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import bev_grid, lane_response, map_bev  # noqa: E402
from bev_match import load_map  # noqa: E402
from camera_model import Cameras  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--frames', type=int, nargs='+', default=[300, 700, 1100, 1500, 1900, 2300, 2700, 3100])
a = ap.parse_args()
track = os.path.basename(os.path.normpath(a.track_dir))
out = os.path.join(os.path.dirname(__file__), '..', 'work', 'calib_pts')
os.makedirs(out, exist_ok=True)
tr = pd.read_csv(a.traj_csv)
M = load_map(a.track_dir)
C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
xs, ys = bev_grid()
for i in a.frames:
    x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    idx = M.query_idx(x, y, 42.0)
    mg, road = map_bev(M.pts[idx], M.inten[idx], np.array([x, y]), yaw)
    sel = lane_response(mg, road) > 1.8
    P = np.stack([xs[sel], ys[sel], np.full(sel.sum(), -1.5)], 1)
    P = P[(P[:, 0] > -5) & (P[:, 0] < 25)]
    if len(P) > 2500:
        P = P[np.random.default_rng(i).choice(len(P), 2500, replace=False)]
    for c in ['fl', 'f', 'fr']:
        uv, ok = C.project(c, P @ Rm)
        pts = [{'id': int(k), 'xyz': [round(float(v), 3) for v in P[k]], 'uv': [round(float(uv[k, 0]), 1), round(float(uv[k, 1]), 1)]}
               for k in np.flatnonzero(ok)]
        json.dump({'track': track, 'frame': int(i), 'cam': c, 'points': pts},
                  open(os.path.join(out, f'{track}_{i}_{c}.json'), 'w'))
    print(track, i, 'marking points', len(P))
