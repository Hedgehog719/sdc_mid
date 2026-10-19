#!/usr/bin/env python3
"""Side-camera calibration in image space with the img_match score (front camera = reference).

For frames along the best trajectory, road-only marking points and road background points of the marking map
are projected into one camera; score = mean paint response at the marks minus at the background (img_match).
Coordinate descent over that camera's (yaw, pitch, roll, tx, ty, tz) corrections maximises the mean score.
Poses are held at the trajectory (Track2 best: ~0.17 m), so a camera that disagrees with the map / front camera
geometry gets corrected; printed per step with the share of frames that improve.
usage: calib_img2.py <track_dir> <traj_csv> <marks_npz> --cam fr --out cam_fix.json
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('marks_npz')
ap.add_argument('--poses', default='', help='fix npz (idx, mx, my, myaw): calibrate at these per-frame poses (e.g. front-only img_match)')
ap.add_argument('--also', action='append', default=[], metavar='TRACK_DIR,TRAJ,MARKS[,POSES]', help='more tracks fitted jointly')
ap.add_argument('--cam', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--frames', default='450:-1:20')
ap.add_argument('--params', default='yaw,pitch,roll,tx,ty,tz')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--eval', default='', help='only score: comma list of calibration json files (cross-track check)')
a = ap.parse_args()
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
rng = np.random.default_rng(0)
cam = a.cam
C = None


def response(j, B, RD):
    img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{cam}_images', B.names[cam][j]), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)
    bad = B.ego_bad(cam, img.shape).copy()
    if cam in B.masks:
        m = np.unpackbits(B.masks[cam][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    r[cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0] = 0
    return cv2.GaussianBlur(r, (0, 0), 2.0)


def onroad(Q, j, RD):
    uv, ok = C.project(cam, Q)
    keep = np.zeros(len(Q), bool)
    if ok.any():
        h4, w4 = RD.shape[1:]
        keep[np.flatnonzero(ok)[RD[j][np.clip((uv[ok, 1] / 4).astype(int), 0, h4 - 1), np.clip((uv[ok, 0] / 4).astype(int), 0, w4 - 1)] > 128]] = True
    return keep


lo, hi, st = (int(v) for v in a.frames.split(':'))
data = []
tracks = [(a.track_dir, a.traj_csv, a.marks_npz, a.poses)] + [tuple((q.split(',') + [''])[:4]) for q in a.also]
for track_dir, traj_csv, marks_npz, poses in tracks:
  tr = pd.read_csv(traj_csv)
  MK = MarkingMap(marks_npz)
  B = ImageBEV(track_dir, 1.5, 0, 0, a.yaw_mount)
  if C is None:
      C = B.C
  RD = np.load(os.path.join(track_dir, 'road', f'{cam}.npz'))['road']
  PZ = dict(np.load(poses)) if poses else None
  pidx = {int(k): n for n, k in enumerate(PZ['idx'])} if PZ is not None else None
  for i in range(lo, len(tr) if hi < 0 else hi, st):
    if pidx is not None:
        if i not in pidx:
            continue
        n_ = pidx[i]
        x, y, yaw = PZ['mx'][n_], PZ['my'][n_], PZ['myaw'][n_]
    else:
        x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
    P = MK.points(x, y, 40)
    if len(P) < 100:
        continue
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > -8) & (np.hypot(q[:, 0], q[:, 1]) < 30)]
    Bw = MK.background(x, y, 40, 1500, rng)
    qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
    qb = qb[road_level(qb, tol=0.15, slope=0.02) & (qb[:, 0] > -8) & (np.hypot(qb[:, 0], qb[:, 1]) < 30)]
    Qm, Qb = q @ Rm, qb @ Rm
    Qm, Qb = Qm[onroad(Qm, i, RD)], Qb[onroad(Qb, i, RD)]
    if len(Qm) < 50 or len(Qb) < 50:
        continue
    if len(Qm) > 2000:
        Qm = Qm[rng.choice(len(Qm), 2000, replace=False)]
    data.append((Qm, Qb, response(i, B, RD)))
print(f'{cam}: {len(data)} frames', flush=True)


def per_frame(prm):
    C.rot_fix[cam] = (prm[0], prm[1], prm[2])
    C.trans_fix[cam] = np.array(prm[3:6])
    out = []
    for Qm, Qb, R in data:
        vals = []
        for Q in (Qm, Qb):
            uv, ok = C.project(cam, Q)
            vals.append(cv2.remap(R, uv[ok, 0].astype(np.float32).reshape(-1, 1), uv[ok, 1].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).mean() if ok.sum() > 20 else np.nan)
        out.append(vals[0] - vals[1])
    return np.array(out)


def score(prm):
    v = per_frame(prm)
    return float(np.nanmean(v))


IDX = {'yaw': 0, 'pitch': 1, 'roll': 2, 'tx': 3, 'ty': 4, 'tz': 5}
if a.eval:
    base = None
    for f in a.eval.split(','):
        d = json.load(open(f))
        prm = list(d.get(cam, (0.0, 0.0, 0.0))) + list(d.get(cam + '_t', (0.0, 0.0, 0.0)))
        v = per_frame(prm)
        if base is None:
            base = v
        ok = np.isfinite(v) & np.isfinite(base)
        print(f'  {cam} {os.path.basename(f):28s} {np.round(prm, 3).tolist()}: score {np.nanmean(v):.4f} '
              f'({(np.nanmean(v) / np.nanmean(base) - 1) * 100:+.1f} %), better than the first on {np.mean(v[ok] > base[ok]) * 100:.0f} % of frames', flush=True)
    sys.exit(0)
cur = list(C.rot_fix.get(cam, (0.0, 0.0, 0.0))) + [float(v) for v in C.trans_fix.get(cam, np.zeros(3))]
start = list(cur)
best = score(cur)
s0 = best
print(f'start {np.round(cur, 3).tolist()} score {best:.4f}', flush=True)
for sa, stt in [(0.5, 0.10), (0.25, 0.05), (0.1, 0.02), (0.05, 0.01)]:
    improved = True
    while improved:
        improved = False
        for pn in a.params.split(','):
            k = IDX[pn]
            for sgn in (-1, 1):
                cand = list(cur)
                cand[k] += sgn * (sa if k < 3 else stt)
                sc = score(cand)
                if sc > best + 1e-5:
                    best, cur, improved = sc, cand, True
    print(f'  step {sa}/{stt}: {np.round(cur, 3).tolist()} score {best:.4f} ({(best / s0 - 1) * 100:+.1f} %)', flush=True)
f0, f1 = per_frame(start), per_frame(cur)
ok = np.isfinite(f0) & np.isfinite(f1)
print(f'  better on {np.mean(f1[ok] > f0[ok]) * 100:.0f} % of {ok.sum()} frames')
res = json.load(open(os.environ.get('SDC_ROT_FIX', os.path.join(os.path.dirname(__file__), '..', 'work', 'cam_rot_fix.json'))))
res[cam] = [round(v, 3) for v in cur[:3]]
res[cam + '_t'] = [round(v, 3) for v in cur[3:]]
json.dump(res, open(a.out, 'w'), indent=1)
print('saved', a.out, res)
