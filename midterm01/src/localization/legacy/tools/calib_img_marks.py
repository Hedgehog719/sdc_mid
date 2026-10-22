#!/usr/bin/env python3
"""Camera calibration in image space against the 3D marking map (no flat-ground assumption).

The global marking map (marking_map.py, mask mode) gives road markings with their 3D map height. For many frames,
the markings within 35 m are taken into the vehicle frame with the road tilt under the vehicle (vehicle_frame) and
projected into each camera with candidate corrections (yaw, pitch, roll in deg; tx, ty, tz in m, front FLU frame).
Score = mean marking response of the raw image at the projected points (white top-hat / local std, blurred), per
frame then averaged. Coordinate descent per camera maximises it. A BEV built with the result has no IPM stretch
from a wrong pitch / height; the near / far split shows up directly as a better score for the right geometry.

usage: calib_img_marks.py <track_dir> <traj_csv> <marks_npz> --out cam_fix.json [--cams f fl fr]
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from common import ply_frames  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('marks_npz')
ap.add_argument('--out', required=True)
ap.add_argument('--cams', nargs='+', default=['f', 'fl', 'fr'])
ap.add_argument('--frames', default='450:-1:30')
ap.add_argument('--params', default='yaw,pitch,roll,tx,ty,tz')
ap.add_argument('--max_range', type=float, default=35.0)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
a = ap.parse_args()

tr = pd.read_csv(a.traj_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
tt = (ts - ts[0]) * 1e-9
spd = gaussian_filter1d(np.hypot(np.gradient(tr.x, tt), np.gradient(tr.y, tt)), 5)
lo, hi, st = (int(v) for v in a.frames.split(':'))
frames = [i for i in range(lo, len(tr) if hi < 0 else hi, st) if spd[i] > 3]
MK = MarkingMap(a.marks_npz)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
rng = np.random.default_rng(0)


def image_response(img, bad):
    g = img.astype(np.float32)
    th = cv2.morphologyEx(g, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (41, 41)))
    sd = np.sqrt(np.maximum(cv2.blur(g * g, (31, 31)) - cv2.blur(g, (31, 31)) ** 2, 1.0))
    r = np.clip(th / sd, 0, 4)
    r[bad > 0] = np.nan
    return cv2.GaussianBlur(np.nan_to_num(r, nan=0.0), (0, 0), 2.0), cv2.dilate((bad > 0).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0


data = {c: [] for c in a.cams}
for i in frames:
    x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    Pw = MK.points(x, y, a.max_range + 5)
    if len(Pw) < 200:
        continue
    q = vehicle_frame(Pw, (x, y, yaw), MK.height)
    q = q[(q[:, 0] > -8) & (np.hypot(q[:, 0], q[:, 1]) < a.max_range) & road_level(q)]
    if len(q) > 6000:
        q = q[rng.choice(len(q), 6000, replace=False)]
    Q = q @ Rm
    for c in a.cams:
        img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][i]),
                         cv2.IMREAD_GRAYSCALE)
        bad = B.ego_bad(c, img.shape).copy()
        if c in B.masks:
            m = np.unpackbits(B.masks[c][i], axis=1)[:, :img.shape[1] // 2]
            bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        resp, badd = image_response(img, bad)
        data[c].append((i, Q, resp, badd))
print(f'{len(frames)} frames, kept per camera: ' + ', '.join(f'{c} {len(v)}' for c, v in data.items()), flush=True)

PN = a.params.split(',')
IDX = {'yaw': 0, 'pitch': 1, 'roll': 2, 'tx': 3, 'ty': 4, 'tz': 5}


def set_params(cam, prm):
    C.rot_fix[cam] = (prm[0], prm[1], prm[2])
    C.trans_fix[cam] = np.array([prm[3], prm[4], prm[5]])


def score(cam, prm, per_frame=False):
    set_params(cam, prm)
    vals = []
    for i, Q, resp, badd in data[cam]:
        uv, ok = C.project(cam, Q)
        if ok.sum() < 30:
            continue
        u, v = uv[ok, 0], uv[ok, 1]
        keep = ~badd[v.astype(int), u.astype(int)]
        if keep.sum() < 30:
            continue
        vals.append(float(cv2.remap(resp, u[keep].astype(np.float32).reshape(-1, 1), v[keep].astype(np.float32).reshape(-1, 1),
                                    cv2.INTER_LINEAR).mean()))
    return np.array(vals) if per_frame else (float(np.mean(vals)) if vals else -1.0)


base_fix = json.load(open(os.environ.get('SDC_ROT_FIX', os.path.join(os.path.dirname(__file__), '..', 'work', 'cam_rot_fix.json'))))
result = dict(base_fix)
for cam in a.cams:
    r0 = list(C.rot_fix.get(cam, (0.0, 0.0, 0.0)))
    t0 = list(C.trans_fix.get(cam, np.zeros(3)))
    cur = r0 + [float(v) for v in t0]
    s0 = score(cam, cur)
    best = s0
    print(f'{cam}: start {np.round(cur, 3).tolist()} score {s0:.4f}', flush=True)
    for sa, stt in [(0.4, 0.10), (0.2, 0.05), (0.1, 0.02), (0.05, 0.01)]:
        improved = True
        while improved:
            improved = False
            for pn in PN:
                k = IDX[pn]
                for sgn in (-1, 1):
                    cand = list(cur)
                    cand[k] += sgn * (sa if k < 3 else stt)
                    sc = score(cam, cand)
                    if sc > best + 1e-5:
                        best, cur, improved = sc, cand, True
        print(f'   step {sa} deg / {stt} m: {np.round(cur, 3).tolist()} score {best:.4f} ({(best / s0 - 1) * 100:+.1f} %)', flush=True)
    pf0, pf1 = score(cam, r0 + [float(v) for v in t0], True), score(cam, cur, True)
    n = min(len(pf0), len(pf1))
    print(f'   {cam}: better on {np.mean(pf1[:n] > pf0[:n]) * 100:.0f} % of {n} frames', flush=True)
    set_params(cam, cur)
    result[cam] = [round(v, 3) for v in cur[:3]]
    result[cam + '_t'] = [round(v, 3) for v in cur[3:]]
json.dump(result, open(a.out, 'w'), indent=1)
print('saved', a.out, result)
