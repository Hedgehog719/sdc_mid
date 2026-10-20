#!/usr/bin/env python3
"""Keep only marking-map cells that lie on the road, by votes of the camera road segmentation.

The LiDAR marking map also responds to verges, bushes, walls and parked cars (bright / textured returns at about road
height). For every frame (step --step) the map cells within --radius m are placed with the trajectory pose (vehicle
frame with the map's road tilt, camera yaw mount) and projected into the three cameras; each projection that is not on
the ego vehicle votes with the road probability of its pixel (work: <track>/road/<cam>.npz, 1/4 resolution). Cells seen
at least --min_votes times whose mean road probability is below --keep are cleared; so are cells that are mostly not
at road level (road_level of marking_map) when in view. Writes a new marking npz (same format).

usage: marking_road_vote.py <track_dir> <traj.csv> <marks_in.npz> <marks_out.npz> [--step 3 --thr 0.02]
"""
import argparse
import os
import sys
from multiprocessing import Pool

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from marking_map import RES_M, SCALE, TILE, MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj')
ap.add_argument('marks_in')
ap.add_argument('marks_out')
ap.add_argument('--step', type=int, default=3)
ap.add_argument('--radius', type=float, default=35.0)
ap.add_argument('--thr', type=float, default=0.02, help='cells with response above this are voted on')
ap.add_argument('--keep', type=float, default=0.4)
ap.add_argument('--min_votes', type=int, default=3)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=4)
a = ap.parse_args()

TR = pd.read_csv(a.traj)
MK = MarkingMap(a.marks_in)
z = dict(np.load(a.marks_in))
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
CAMS = ('fl', 'f', 'fr')
RD = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in CAMS}
KEYS = {tuple(k): t for t, k in enumerate(z['keys'].astype(int))}
N = z['resp'].shape[1]
BAD = {}
for c in CAMS:
    K_, D_, T_, w, h = C.cams[c]
    BAD[c] = (w, h, B.ego_bad(c, (h, w)).copy())


def cell_ids(P):
    tx, ty = np.floor(P[:, 0] / TILE).astype(np.int64), np.floor(P[:, 1] / TILE).astype(np.int64)
    ii = np.clip(np.floor((P[:, 0] - tx * TILE) / RES_M).astype(np.int64), 0, N - 1)
    jj = np.clip(np.floor((P[:, 1] - ty * TILE) / RES_M).astype(np.int64), 0, N - 1)
    t = np.array([KEYS.get((x_, y_), -1) for x_, y_ in zip(tx.tolist(), ty.tolist())])
    return t, ii, jj


def work(j):
    x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, a.radius, thr=a.thr)
    if not len(P):
        return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    near = (q[:, 0] > -8) & (np.hypot(q[:, 0], q[:, 1]) < a.radius)
    P, q = P[near], q[near]
    lvl = road_level(q)
    Q = q @ Rm
    vote = np.zeros(len(P), np.float32)
    nv = np.zeros(len(P), np.int16)
    seen = np.zeros(len(P), bool)
    for c in CAMS:
        w, h, bad = BAD[c]
        if c in B.masks and j < len(B.masks[c]):
            m = np.unpackbits(B.masks[c][j], axis=1)[:, :w // 2]
            bad = bad | cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        uv, ok = C.project(c, Q)
        u, v = uv[:, 0].astype(int), uv[:, 1].astype(int)
        ins = ok & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        ins[ins] &= bad[v[ins], u[ins]] == 0
        rd = RD[c][j]
        rp = rd[np.clip(v[ins] // 4, 0, rd.shape[0] - 1), np.clip(u[ins] // 4, 0, rd.shape[1] - 1)].astype(np.float32) / 255.0
        k = np.flatnonzero(ins)
        vote[k[lvl[k]]] += rp[lvl[k]]
        nv[k[lvl[k]]] += 1
        seen[k] = True
    t, ii, jj = cell_ids(P)
    return t, ii, jj, vote, nv, seen, lvl


if __name__ == '__main__':
    frames = list(range(0, len(TR), a.step))
    nt = len(z['keys'])
    vs = np.zeros((nt, N, N), np.float32)
    vn = np.zeros((nt, N, N), np.int32)
    seen_n = np.zeros((nt, N, N), np.int32)
    lvl_n = np.zeros((nt, N, N), np.int32)
    with Pool(a.workers) as pool:
        for r in pool.imap(work, frames, chunksize=4):
            if r is None:
                continue
            t, ii, jj, vote, nv, seen, lvl = r
            ok = t >= 0
            np.add.at(vs, (t[ok], ii[ok], jj[ok]), vote[ok])
            np.add.at(vn, (t[ok], ii[ok], jj[ok]), nv[ok])
            np.add.at(seen_n, (t[ok], ii[ok], jj[ok]), seen[ok].astype(np.int32))
            np.add.at(lvl_n, (t[ok], ii[ok], jj[ok]), (seen & lvl)[ok].astype(np.int32))
    resp = z['resp'].copy()
    active = resp > a.thr * SCALE
    frac = np.where(vn > 0, vs / np.maximum(vn, 1), np.nan)
    off_road = active & (vn >= a.min_votes) & (frac < a.keep)
    off_level = active & (seen_n >= a.min_votes) & (lvl_n < 0.4 * seen_n)
    clear = off_road | off_level
    resp[clear] = 0
    z['resp'] = resp
    np.savez_compressed(a.marks_out, **z)
    print(f'{os.path.basename(a.marks_in)}: {active.sum()} active cells, voted {(vn[active] >= a.min_votes).sum()}; cleared '
          f'{off_road.sum()} off-road (mean road prob < {a.keep}) + {(off_level & ~off_road).sum()} off-level -> '
          f'{(resp > a.thr * SCALE).sum()} kept ({(resp > a.thr * SCALE).sum() / max(active.sum(), 1) * 100:.0f} %) -> {a.marks_out}')
