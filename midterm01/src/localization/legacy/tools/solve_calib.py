#!/usr/bin/env python3
"""Solve per-camera calibration from hand-marked correspondences (pole_review_server /calib).

Each pair links a map lane-marking point (vehicle frame, on the road plane, from an accurate trajectory) to the
pixel where that marking really is. Per camera, least squares over the rotation correction (yaw, pitch, roll) and,
with enough well-spread pairs (--distortion), the four fisheye distortion coefficients, minimising the pixel
reprojection error (robust Huber loss). Writes work/cam_rot_fix.json (and work/cam_dist_fix.json).
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from camera_model import Cameras  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--info', default='/root/data/Track2/Track2_images/camera_info')
ap.add_argument('--distortion', action='store_true', help='also refine fisheye k1..k4 (needs ~30+ spread pairs)')
ap.add_argument('--min_pairs', type=int, default=8)
a = ap.parse_args()
work = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'work')
pairs = json.load(open(os.path.join(work, 'calib_pairs.json')))['pairs']
C = Cameras(a.info)
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
rot_out = {k: list(v) for k, v in C.rot_fix.items()}
dist_out = {}
for cam in ['fl', 'f', 'fr']:
    P = [p for p in pairs if p['cam'] == cam]
    if len(P) < a.min_pairs:
        print(f'{cam}: {len(P)} pairs (< {a.min_pairs}), skipped')
        continue
    X = np.array([p['xyz'] for p in P]) @ Rm
    U = np.array([p['uv_img'] for p in P])
    K, D0, T, w, h = C.cams[cam]
    r0 = list(C.rot_fix.get(cam, (C.yaw_fix.get(cam, 0.0), 0.0, 0.0)))

    def resid(v):
        C.rot_fix[cam] = tuple(v[:3])
        if a.distortion:
            C.cams[cam] = (K, D0 + v[3:7], T, w, h)
        uv, ok = C.project(cam, X)
        r = uv - U
        r[~ok] = 200.0
        return r.ravel()
    v0 = np.array(r0 + ([0.0] * 4 if a.distortion else []))
    e0 = np.sqrt(np.mean(resid(v0) ** 2))
    sol = least_squares(resid, v0, loss='huber', f_scale=8.0, x_scale=np.array([1, 1, 1] + ([0.01] * 4 if a.distortion else [])))
    e1 = np.sqrt(np.mean(resid(sol.x) ** 2))
    rot_out[cam] = [round(float(v), 3) for v in sol.x[:3]]
    if a.distortion:
        dist_out[cam] = [float(v) for v in (D0 + sol.x[3:7])]
        C.cams[cam] = (K, D0, T, w, h)
    print(f'{cam}: {len(P)} pairs, reprojection rms {e0:.1f} px -> {e1:.1f} px; rot (yaw, pitch, roll) {rot_out[cam]}'
          + (f'; distortion {np.round(dist_out[cam], 4).tolist()}' if a.distortion else ''))
json.dump(rot_out, open(os.path.join(work, 'cam_rot_fix.json'), 'w'), indent=1)
if dist_out:
    json.dump(dist_out, open(os.path.join(work, 'cam_dist_fix.json'), 'w'), indent=1)
print('saved', rot_out)
