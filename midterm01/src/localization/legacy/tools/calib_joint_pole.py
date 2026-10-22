#!/usr/bin/env python3
"""Joint side-camera calibration: rotation from lane markings AND poles, position from poles.

Lane term: calib_side's per-camera peak NCC (lower image, ground markings; insensitive to the camera position
because of its +-1 m search). Pole term: pole_offsets.py measurements (upper image, poles beside the road); for a
candidate (yaw, pitch, roll, dx, dy) the predicted residual is off - (u_candidate - u_base). Coordinate descent
minimises the median |pole residual| while keeping the lane NCC >= keep x its baseline.
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
from bev import ImageBEV, lane_response, map_bev  # noqa: E402
from bev_match import load_map  # noqa: E402
from calib_side import BLUR, score  # noqa: E402
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('pole_npz')
ap.add_argument('--out', required=True)
ap.add_argument('--cams', nargs='+', default=['fl', 'fr'])
ap.add_argument('--keep', type=float, default=0.99, help='lane NCC must stay >= keep x baseline')
ap.add_argument('--step', type=int, default=60)
ap.add_argument('--no_trans', action='store_true', help='rotation only (camera position fixed)')
a = ap.parse_args()

track = os.path.basename(os.path.normpath(a.track_dir))
ref = pd.read_csv(a.traj_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
tt = (ts - ts[0]) * 1e-9
spd = np.hypot(np.gradient(ref.x, tt), np.gradient(ref.y, tt))
frames = [i for i in range(60, len(ref), a.step) if spd[i] > 3]
M = load_map(a.track_dir)
B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
C = B.C
cache = {}
for i in frames:
    x, y, yaw = ref.x[i], ref.y[i], ref.yaw[i]
    idx = M.query_idx(x, y, 42.0)
    mg, road = map_bev(M.pts[idx], M.inten[idx], np.array([x, y]), yaw)
    lr = lane_response(mg, road)
    d = {'ref': lr, 'ref_blur': cv2.GaussianBlur(lr, (0, 0), BLUR),
         'road': cv2.dilate(road.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0}
    for c in a.cams:
        img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][i]),
                         cv2.IMREAD_GRAYSCALE)
        bad = B.ego_bad(c, img.shape).copy()
        if c in B.masks:
            m = np.unpackbits(B.masks[c][i], axis=1)[:, :img.shape[1] // 2]
            bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        d[c], d[c + '_bad'] = img, bad
    cache[i] = d
print(f'{len(frames)} lane frames', flush=True)

# pole measurements -> 3D points in the vehicle frame
z = np.load(a.pole_npz)
poles = np.load(os.path.join(a.track_dir, 'poles_curated.npy'))
Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
good = z['contrast'] > 2.5
cams = ['fl', 'f', 'fr']
P = {c: [] for c in a.cams}
for fi, ci, k, off in zip(z['frame'][good].astype(int), z['cam'][good].astype(int), z['pole'][good].astype(int), z['off'][good]):
    cam = cams[ci]
    if cam not in P:
        continue
    x, y, yaw = ref.x[fi], ref.y[fi], ref.yaw[fi]
    c_, s_ = np.cos(yaw), np.sin(yaw)
    dx, dy = poles[k, 0] - x, poles[k, 1] - y
    pu, pv = c_ * dx + s_ * dy, -s_ * dx + c_ * dy
    hs = np.linspace(2.0, min(poles[k, 4], 7.0), 24)
    Q = np.stack([np.full(len(hs), pu), np.full(len(hs), pv), hs - 1.5], 1) @ Rm
    P[cam].append((Q, off))


def pole_u(cam):
    us = []
    for Q, _ in P[cam]:
        uv, ok = C.project(cam, Q)
        us.append(np.median(uv[ok, 0]) if ok.sum() >= 4 else np.nan)
    return np.array(us)


def set_params(cam, prm):
    C.rot_fix[cam] = (prm[0], prm[1], prm[2])
    C.trans_fix[cam] = np.array([prm[3], prm[4], 0.0])


result = json.load(open(os.environ.get('SDC_ROT_FIX', os.path.join(os.path.dirname(__file__), '..', 'work', 'cam_rot_fix.json'))))
for cam in a.cams:
    base = list(C.rot_fix.get(cam, (0.0, 0.0, 0.0))) + [0.0, 0.0]
    set_params(cam, base)
    ncc0, n = score(B, cam, frames, cache)
    u0 = pole_u(cam)
    offs = np.array([o for _, o in P[cam]])

    def evaluate(prm):
        set_params(cam, prm)
        ncc, _ = score(B, cam, frames, cache)
        res = offs - (pole_u(cam) - u0)
        return ncc, float(np.nanmedian(np.abs(res))), float(np.nanmedian(res))

    def objective(ncc, mad):
        return mad + (1e3 * (a.keep * ncc0 - ncc) if ncc < a.keep * ncc0 else 0.0)

    cur = list(base)
    ncc, mad, med = evaluate(cur)
    best = objective(ncc, mad)
    print(f'{cam}: base {np.round(cur, 3).tolist()} lane NCC {ncc0:.4f} (n={n}) pole |res| median {mad:.1f}px (median {med:+.1f})', flush=True)
    steps = [(1.0, 0.2), (0.5, 0.1), (0.25, 0.05)]
    for sa, st in steps:
        improved = True
        while improved:
            improved = False
            for k in range(3 if a.no_trans else 5):
                for sgn in (-1, 1):
                    cand = list(cur)
                    cand[k] += sgn * (sa if k < 3 else st)
                    nc, md, me = evaluate(cand)
                    ob = objective(nc, md)
                    if ob < best - 1e-3:
                        best, cur, improved = ob, cand, True
                        ncc, mad, med = nc, md, me
        print(f'   step {sa}/{st}: {np.round(cur, 3).tolist()} lane NCC {ncc:.4f} ({ncc / ncc0 * 100:.1f} %) pole |res| {mad:.1f}px (median {med:+.1f})', flush=True)
    set_params(cam, cur)
    result[cam] = [round(v, 3) for v in cur[:3]]
    result[cam + '_t'] = [round(cur[3], 3), round(cur[4], 3), 0.0]
json.dump(result, open(a.out, 'w'), indent=1)
print('saved', a.out, result)
