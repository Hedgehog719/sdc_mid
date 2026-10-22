#!/usr/bin/env python3
"""Keep only LiDAR marking-map cells that the cameras see as paint, voted on per-frame aligned projections.

The LiDAR marking map (intensity) has blobs around thin lines, smeared edges and bright non-paint returns. For every
frame (step --step) the near map cells (1..--rmax m ahead, road level) are first aligned to the image paint as a whole:
a (fwd, lat) shift of the projected cells (coarse +-0.3 m grid on a subsample, refined at 0.05 m) maximises the mean
paint response over the three cameras, so pose errors do not decide the vote (near and far are shifted together).
Frames whose alignment peak is weak are skipped. At the aligned shift every cell is projected into each camera (ego /
vehicle masks applied) and votes with the paint response of its pixel. Cells seen at least --min_votes times whose
mean paint response is below --keep are cleared (spill around thin lines, non-paint returns). Same npz format.

usage: marking_paint_vote.py <track_dir> <traj.csv> <marks_in.npz> <marks_out.npz> [--step 3 --keep 0.12]
"""
import argparse
import os
import sys
from multiprocessing import Pool

import cv2
cv2.setNumThreads(1)
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
ap.add_argument('--rmax', type=float, default=16.0)
ap.add_argument('--lat', type=float, default=8.0)
ap.add_argument('--thr', type=float, default=0.02, help='cells with response above this are voted on')
ap.add_argument('--keep', type=float, default=0.12, help='min mean paint response of a kept cell')
ap.add_argument('--min_votes', type=int, default=4)
ap.add_argument('--min_contrast', type=float, default=0.15, help='skip frames whose alignment peak is weaker')
ap.add_argument('--blur', type=float, default=1.0, help='px Gaussian blur of the paint response used for voting')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=3)
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
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


def response(cam, j):
    """Paint response (same definition as the matchers: brighter than the local road median, x road probability,
    ego / vehicle masked), returned sharp (--blur) for voting and blurred 2 px for the alignment."""
    img = cv2.imread(os.path.join(a.track_dir, f'{T}_images', f'{T}_{cam}_images', B.names[cam][j]), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[cam][j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)
    bad = B.ego_bad(cam, img.shape).copy()
    if cam in B.masks and j < len(B.masks[cam]):
        m = np.unpackbits(B.masks[cam][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    bad = cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0
    r[bad] = 0
    return (cv2.GaussianBlur(r, (0, 0), a.blur) if a.blur > 0 else r), cv2.GaussianBlur(r, (0, 0), 2.0), bad


def sample(c, Q, R, bad=None):
    uv, ok = C.project(c, Q)
    h, w = R.shape
    u, v = uv[:, 0], uv[:, 1]
    ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
    if bad is not None:
        ins[ins] &= ~bad[v[ins].astype(int), u[ins].astype(int)]
    val = np.zeros(len(Q), np.float32)
    if ins.any():
        val[ins] = cv2.remap(R, u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1),
                             cv2.INTER_LINEAR).ravel()
    return val, ins


def cell_ids(P):
    tx, ty = np.floor(P[:, 0] / TILE).astype(np.int64), np.floor(P[:, 1] / TILE).astype(np.int64)
    ii = np.clip(np.floor((P[:, 0] - tx * TILE) / RES_M).astype(np.int64), 0, N - 1)
    jj = np.clip(np.floor((P[:, 1] - ty * TILE) / RES_M).astype(np.int64), 0, N - 1)
    t = np.array([KEYS.get((x_, y_), -1) for x_, y_ in zip(tx.tolist(), ty.tolist())])
    return t, ii, jj


def align_score(q, resp_al, sh):
    tot, n = 0.0, 0
    Q = (q - [sh[0], sh[1], 0]) @ Rm
    for c in CAMS:
        val, ins = sample(c, Q, resp_al[c])
        tot += val[ins].sum()
        n += ins.sum()
    return tot / max(n, 1)


def work(j):
    x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, a.rmax + 6, thr=a.thr)
    if len(P) < 50:
        return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    sel = road_level(q) & (q[:, 0] > 1.0) & (q[:, 0] < a.rmax) & (np.abs(q[:, 1]) < a.lat)
    P, q = P[sel], q[sel]
    if len(P) < 50:
        return None
    resp_v, resp_al, bads = {}, {}, {}
    for c in CAMS:
        resp_v[c], resp_al[c], bads[c] = response(c, j)
    rng = np.random.default_rng(j)
    qs = q[rng.choice(len(q), min(len(q), 1500), replace=False)]
    grid = np.arange(-0.3, 0.3001, 0.1)
    S = np.array([[align_score(qs, resp_al, (df, dl)) for dl in grid] for df in grid])
    i, k = np.unravel_index(np.argmax(S), S.shape)
    best = (grid[i], grid[k])
    fine = np.arange(-0.05, 0.0501, 0.05)
    Sf = np.array([[align_score(qs, resp_al, (best[0] + df, best[1] + dl)) for dl in fine] for df in fine])
    i2, k2 = np.unravel_index(np.argmax(Sf), Sf.shape)
    best = (best[0] + fine[i2], best[1] + fine[k2])
    peak = Sf.max()
    contrast = (peak - np.median(S)) / max(peak, 1e-6)
    if contrast < a.min_contrast:
        return j, None, best, contrast
    Q = (q - [best[0], best[1], 0]) @ Rm
    vote = np.zeros(len(P), np.float32)
    nv = np.zeros(len(P), np.int16)
    for c in CAMS:
        val, ins = sample(c, Q, resp_v[c], bads[c])
        vote[ins] += val[ins]
        nv[ins] += 1
    t, ii, jj = cell_ids(P)
    return j, (t, ii, jj, vote, nv), best, contrast


if __name__ == '__main__':
    frames = list(range(0, len(TR), a.step))
    nt = len(z['keys'])
    vs = np.zeros((nt, N, N), np.float32)
    vn = np.zeros((nt, N, N), np.int32)
    used, shifts = 0, []
    with Pool(a.workers) as pool:
        for r in pool.imap(work, frames, chunksize=2):
            if r is None:
                continue
            j, d, best, contrast = r
            shifts.append((j, best[0], best[1], contrast, d is not None))
            if d is None:
                continue
            used += 1
            t, ii, jj, vote, nv = d
            ok = t >= 0
            np.add.at(vs, (t[ok], ii[ok], jj[ok]), vote[ok])
            np.add.at(vn, (t[ok], ii[ok], jj[ok]), nv[ok].astype(np.int32))
    resp = z['resp'].copy()
    active = resp > a.thr * SCALE
    voted = active & (vn >= a.min_votes)
    mean = np.where(vn > 0, vs / np.maximum(vn, 1), np.nan)
    clear = voted & (mean < a.keep)
    resp[clear] = 0
    z['resp'] = resp
    np.savez_compressed(a.marks_out, **z)
    sh = np.array(shifts)
    np.save(a.marks_out.replace('.npz', '_shifts.npy'), sh)
    q = np.nanpercentile(mean[voted], [10, 25, 50, 75, 90]) if voted.any() else [np.nan] * 5
    print(f'{T} {os.path.basename(a.marks_in)}: frames aligned {used} / {len(sh)} (alignment shift fwd median '
          f'{np.median(sh[sh[:, 4] > 0, 1]):+.2f} lat {np.median(sh[sh[:, 4] > 0, 2]):+.2f}); active cells {active.sum()}, '
          f'voted {voted.sum()} (mean paint p10/25/50/75/90 {" / ".join(f"{v:.2f}" for v in q)}); cleared {clear.sum()} '
          f'(< {a.keep}) -> kept {(resp > a.thr * SCALE).sum()} ({(resp > a.thr * SCALE).sum() / max(active.sum(), 1) * 100:.0f} %) -> {a.marks_out}')
