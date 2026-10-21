#!/usr/bin/env python3
"""Near-range image fixes: per frame, the (fwd, lat) pose correction that best aligns the near road markings.

Built on near_align.py (same score). Near-range image-to-map alignment (local standard, no leaderboard).

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
cv2.setNumThreads(1)
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
ap.add_argument('--out', required=True)
ap.add_argument('--bg_weight', type=float, default=0.0, help='>0: score = marks - w * background: paint on map road without markings is penalised')
ap.add_argument('--cams', default='fl,f,fr', help='cameras used (e.g. f at the T2 start, where the side cameras see parked '
                'cars that leak into the marking map)')
ap.add_argument('--yaw_search', type=float, default=0.0, help='>0: also search the heading (+-deg, 0.1 deg steps)')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TRS = [pd.read_csv(f) for f in a.trajs]
MK = MarkingMap(a.marks)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
CAMS = tuple(a.cams.split(','))
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



def fix(j):
    TR = TRS[0]
    x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, a.rmax + 5)
    if not len(P):
        return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > 1.0) & (q[:, 0] < a.rmax) & (np.abs(q[:, 1]) < a.lat)]
    if len(q) < 60:
        return None
    resp = {c: response(c, j) for c in CAMS}
    qb = None
    if a.bg_weight > 0:   # road points of the map without markings (in / out scoring)
        Bw = MK.background(x, y, a.rmax + 5, 3000, np.random.default_rng(j))
        if len(Bw):
            qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
            qb = qb[road_level(qb, tol=0.15, slope=0.02) & (qb[:, 0] > 1.0) & (qb[:, 0] < a.rmax) & (np.abs(qb[:, 1]) < a.lat)]
            if len(qb) < 60:
                qb = None

    def sc2(sh, rot=0.0):
        def tf(pts):
            p = pts - sh
            if rot:
                ct, st_ = np.cos(-rot), np.sin(-rot)
                p = np.c_[ct * p[:, 0] - st_ * p[:, 1], st_ * p[:, 0] + ct * p[:, 1], p[:, 2]]
            return p
        v = score(tf(q), resp)
        if qb is not None and np.isfinite(v):
            b = score(tf(qb), resp)
            v = v - a.bg_weight * (b if np.isfinite(b) else 0.0)
        return v
    offs = np.arange(-a.span, a.span + 1e-9, 0.1)
    g = np.full((len(offs), len(offs)), np.nan)
    for i, df in enumerate(offs):
        for jj, dl in enumerate(offs):
            g[i, jj] = sc2(np.array([df, dl, 0]))
    if not np.isfinite(g).any():
        return None
    g = np.nan_to_num(g, nan=0.0)
    i, jj = np.unravel_index(np.argmax(g), g.shape)
    if i in (0, len(offs) - 1) or jj in (0, len(offs) - 1):
        return None

    def para(v, k):
        den = v[k - 1] - 2 * v[k] + v[k + 1]
        return float(np.clip(0.5 * (v[k - 1] - v[k + 1]) / den, -0.5, 0.5)) if den < 0 else 0.0
    df = offs[i] + 0.1 * para(g[:, jj], i)
    dl = offs[jj] + 0.1 * para(g[i, :], jj)
    sf = g[i, jj] - max(g[max(i - 3, 0), jj], g[min(i + 3, len(offs) - 1), jj])
    sl = g[i, jj] - max(g[i, max(jj - 3, 0)], g[i, min(jj + 3, len(offs) - 1)])
    dyaw, sy = 0.0, 0.0
    if a.yaw_search > 0:   # heading: rotate the map points about the vehicle origin (after the position shift)
        ys = np.radians(np.arange(-a.yaw_search, a.yaw_search + 1e-9, 0.1))
        gy = [sc2(np.array([df, dl, 0]), th) for th in ys]   # true heading = yaw + th <=> points rotate by -th
        gy = np.nan_to_num(np.array(gy))
        k = int(np.argmax(gy))
        if 0 < k < len(ys) - 1:
            den = gy[k - 1] - 2 * gy[k] + gy[k + 1]
            dyaw = float(ys[k] + (np.clip(0.5 * (gy[k - 1] - gy[k + 1]) / den, -0.5, 0.5) if den < 0 else 0) * np.radians(0.1))
            sy = gy[k] - max(gy[max(k - 3, 0)], gy[min(k + 3, len(ys) - 1)])
        else:
            sy = -1.0          # at the search edge: no yaw information
    c, s = np.cos(yaw), np.sin(yaw)        # true pose = pose + R(yaw) (df, dl)
    return dict(idx=j, ts=float(FT[j]), px=x, py=y, pyaw=yaw, mx=x + c * df - s * dl, my=y + s * df + c * dl, myaw=yaw + dyaw,
                dyaw=dyaw, sharp_y=sy, fwd=df, lat=dl, peak=g[i, jj], base=g[len(offs) // 2, len(offs) // 2], sharp_f=sf,
                sharp_l=sl, npts=len(q))


if __name__ == '__main__':
    FT = pd.read_csv(os.path.join(a.track_dir, '3cameras_ply', 'index.csv')).f_ts.values.astype(np.float64)
    lo, hi, st = (int(v) for v in a.frames.split(':'))
    frames = list(range(lo, len(TRS[0]) if hi < 0 else hi, st))
    with Pool(a.workers) as pool:
        R = [r for r in pool.imap(fix, frames, chunksize=4) if r is not None]
    D = pd.DataFrame(R)
    np.savez(a.out, **{k: D[k].values for k in D.columns}, ts_i=D.ts.values.astype(np.int64))
    print(f'{T}: {len(D)} near-range fixes of {len(frames)} frames; fwd median {D.fwd.median():+.3f} lat median {D.lat.median():+.3f}, '
          f'gain median {(D.peak - D.base).median():.3f}, sharp f/l {D.sharp_f.median():.3f}/{D.sharp_l.median():.3f} -> {a.out}')
