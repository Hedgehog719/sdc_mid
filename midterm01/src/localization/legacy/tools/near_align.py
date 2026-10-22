#!/usr/bin/env python3
"""Near-range image-to-map alignment (local standard, no leaderboard).

For each frame: marking-map points on the road 1..--rmax m ahead (|lateral| < --lat) are placed with a trajectory
pose (vehicle frame with road tilt, camera mount) and projected into the cameras; the score is the mean paint
response of the images at those points (paint = brighter than the local road median, times road probability, ego
masked, blurred 2 px - the image-space matcher's response). Near markings have the most precise geometry (pixel
resolution, little attitude sensitivity), so a better pose scores higher. Prints per trajectory the mean score, the
share of frames where it beats the first trajectory, and (--search) the per-frame best (fwd, lat) offset of the
first trajectory, summarised per segment (systematic pose errors show as consistent offsets).

usage: near_align.py <track_dir> <marks_D.npz> traj1.csv [traj2.csv ...] [--frames 0:-1:9 --search]
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
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('marks')
ap.add_argument('trajs', nargs='+')
ap.add_argument('--frames', default='0:-1:9')
ap.add_argument('--rmax', type=float, default=12.0)
ap.add_argument('--lat', type=float, default=4.0)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--search', action='store_true')
ap.add_argument('--span', type=float, default=0.6)
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--out', default='')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TRS = [pd.read_csv(f) for f in a.trajs]
MK = MarkingMap(a.marks)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
CAMS = ('fl', 'f', 'fr')
RD = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in CAMS}


def response(cam, j):
    img = cv2.imread(os.path.join(a.track_dir, f'{T}_images', f'{T}_{cam}_images', B.names[cam][j]), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[cam][j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)
    bad = B.ego_bad(cam, img.shape).copy()
    if cam in B.masks:
        m = np.unpackbits(B.masks[cam][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    r[cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0] = 0
    return cv2.GaussianBlur(r, (0, 0), 2.0)


def score(q, resp):
    vals = []
    for c in CAMS:
        uv, ok = C.project(c, q @ Rm)
        h, w = resp[c].shape
        u, v = uv[:, 0], uv[:, 1]
        ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        if ins.any():
            vals.append(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1),
                                  cv2.INTER_LINEAR).ravel())
    v = np.concatenate(vals) if vals else np.zeros(0)
    return float(v.mean()) if len(v) > 30 else np.nan


def work(j):
    resp = {c: response(c, j) for c in CAMS}
    out = []
    best = None
    for k, TR in enumerate(TRS):
        x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
        P = MK.points(x, y, a.rmax + 5)
        if not len(P):
            out.append(np.nan)
            continue
        q = vehicle_frame(P, (x, y, yaw), MK.height)
        sel = road_level(q) & (q[:, 0] > 1.0) & (q[:, 0] < a.rmax) & (np.abs(q[:, 1]) < a.lat)
        q = q[sel]
        out.append(score(q, resp) if len(q) > 30 else np.nan)
        if a.search and k == 0 and len(q) > 30:
            offs = np.arange(-a.span, a.span + 1e-9, 0.1)
            g = np.full((len(offs), len(offs)), np.nan)
            for i, df in enumerate(offs):        # vehicle-frame shift of the map points = minus the pose correction
                for jj, dl in enumerate(offs):
                    g[i, jj] = score(q - [df, dl, 0], resp)
            if np.isfinite(g).any():
                i, jj = np.unravel_index(np.nanargmax(g), g.shape)
                edge = i in (0, len(offs) - 1) or jj in (0, len(offs) - 1)
                best = (offs[i], offs[jj], g[i, jj] - out[0], edge)
    return j, out, best


if __name__ == '__main__':
    lo, hi, st = (int(v) for v in a.frames.split(':'))
    frames = list(range(lo, len(TRS[0]) if hi < 0 else hi, st))
    with Pool(a.workers) as pool:
        R = list(pool.imap(work, frames, chunksize=2))
    S = np.array([r[1] for r in R], float)
    names = [os.path.basename(f) for f in a.trajs]
    ok = np.isfinite(S).all(1)
    print(f'{T}: near-range alignment ({ok.sum()} frames, 1-{a.rmax:.0f} m ahead, |lat| < {a.lat:.0f} m), higher is better')
    for k, nm in enumerate(names):
        beat = np.mean(S[ok, k] > S[ok, 0]) * 100 if k else np.nan
        print(f'   {nm:40s} mean {np.nanmean(S[ok, k]):.4f}' + (f' | better than the first on {beat:.0f} % of frames' if k else ''))
    if a.search:
        rows = [(j, b[0], b[1], b[2], b[3]) for j, _, b in R if b is not None]
        D = pd.DataFrame(rows, columns=['frame', 'fwd', 'lat', 'gain', 'edge'])
        D = D[~D.edge]
        # score(q - s) best at s = (fwd, lat) <=> true pose = pose + R(yaw) s (checked: T1 2907 needs lat -0.3..-0.4)
        print(f'   best map shift for {names[0]} (= the pose correction: true pose = pose + R(yaw) (fwd, lat)), per segment (non-edge frames):')
        n = len(TRS[0])
        for s0 in range(0, n, 300):
            m = (D.frame >= s0) & (D.frame < s0 + 300)
            if m.sum() >= 3:
                print(f'      {s0:4d}-{min(s0 + 300, n):4d}: n {m.sum():3d} fwd {D.fwd[m].median():+.2f} lat {D.lat[m].median():+.2f} gain {D.gain[m].median():.3f}')
        if a.out:
            D.to_csv(a.out, index=False)
