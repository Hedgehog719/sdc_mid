#!/usr/bin/env python3
"""Position fixes from the camera point clouds (3cameras_ply, MapAnything), gated by the IMU.

The clouds are per-frame model predictions: scale (from the cloud's ground, camera 1.70 m) jitters by +-7 % and the
predicted camera frame rotates by up to +-2.7 deg between frames. A cloud is used only when it agrees with the IMU:
its rotation relative to the clouds 'step' frames before and after (2D ICP of raised structure) matches the IMU yaw
change within --gate_yaw deg, and its scale is within --gate_scale of the neighbours' median. Accepted clouds are
rotated by the camera yaw mount and placed with the trajectory yaw (the IMU decides the heading); a grid search
over (fwd, lat) scores the share of cloud structure points within --tol of map structure (both: points 0.3-6 m above
their local ground). Output npz like the other fix files (idx, ts, mx, my, myaw, peak, sharp_f, sharp_l, fwd, lat).

usage: cam_cloud_fix.py <track_dir> <traj.csv> <out.npz> --tag clean16 [--frames 0:-1:3 --workers 4]
"""
import argparse
import os
import sys
from multiprocessing import Pool

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'work', 'cam3d'))
from icp import read_ply, voxel  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj')
ap.add_argument('out')
ap.add_argument('--tag', default='')
ap.add_argument('--frames', default='0:-1:3')
ap.add_argument('--step', type=int, default=3)
ap.add_argument('--gate_yaw', type=float, default=0.3)
ap.add_argument('--gate_scale', type=float, default=0.04)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--cam_h', type=float, default=1.70)
ap.add_argument('--rmax', type=float, default=20.0)
ap.add_argument('--span', type=float, default=1.2)
ap.add_argument('--res', type=float, default=0.1)
ap.add_argument('--tol', type=float, default=0.15)
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--scale_win', type=int, default=30, help='+-frames of the median used as the cloud scale (the '
                'model scale drifts slowly, ~2 %% between frames; the per-frame ground estimate jitters ~6 %%); 0 = per frame')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TR = pd.read_csv(a.traj)
IDX = pd.read_csv(os.path.join(a.track_dir, '3cameras_ply', 'index.csv'))
FT = IDX.f_ts.values.astype(np.float64)
im = pd.read_csv(os.path.join(a.track_dir, 'sensors', 'imu.csv'))
IY = np.interp(FT, im.stamp.values.astype(np.float64),
               np.unwrap(Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values).as_euler('ZYX')[:, 0]))
sfx = f'_{a.tag}' if a.tag else ''
MAP = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'), mmap_mode='r')
cm, sm = np.cos(np.radians(a.yaw_mount)), np.sin(np.radians(a.yaw_mount))


def ground(k):
    """Ground plane of raw cloud k (camera frame, model units): (pa, pb, pc) or None."""
    Q = read_ply(os.path.join(a.track_dir, '3cameras_ply', IDX.ply[k]))
    near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
    if near.sum() < 100:
        return None
    G = Q[near]
    z0 = np.percentile(G[:, 2], 20)
    gg = G[np.abs(G[:, 2] - z0) < 0.25]
    return np.linalg.lstsq(np.c_[gg[:, 0], gg[:, 1], np.ones(len(gg))], gg[:, 2], rcond=None)[0]


SMOOTH = {}


def cloud(k):
    """Raised structure of cloud k in the vehicle frame (2D + height), its scale; None if no ground."""
    Q = read_ply(os.path.join(a.track_dir, '3cameras_ply', IDX.ply[k]))
    near = (Q[:, 0] > 3) & (Q[:, 0] < 15) & (np.abs(Q[:, 1]) < 3)
    if near.sum() < 100:
        return None
    G = Q[near]
    z0 = np.percentile(G[:, 2], 20)
    gg = G[np.abs(G[:, 2] - z0) < 0.25]
    pa, pb, pc = np.linalg.lstsq(np.c_[gg[:, 0], gg[:, 1], np.ones(len(gg))], gg[:, 2], rcond=None)[0]
    s = SMOOTH.get(k, a.cam_h / max(-pc, 0.5))
    hz = (Q[:, 2] - (pa * Q[:, 0] + pb * Q[:, 1] + pc)) * s
    r = np.hypot(Q[:, 0], Q[:, 1]) * s
    m = (r > 3) & (r < a.rmax) & (hz > 0.3) & (hz < 6)
    P = Q[m, :2] * s
    P = np.c_[cm * P[:, 0] - sm * P[:, 1], sm * P[:, 0] + cm * P[:, 1], hz[m]]   # camera mount yaw -> vehicle
    return voxel(P, 0.15), s


def icp_yaw(A, B, th0, t0):
    """2D rigid ICP of A onto B; returns the rotation (rad)."""
    tree = cKDTree(B[:, :2])
    th, t = th0, t0.copy()
    for it in range(20):
        c, s = np.cos(th), np.sin(th)
        W = A[:, :2] @ np.array([[c, s], [-s, c]]) + t
        d, j = tree.query(W, distance_upper_bound=0.8 if it < 8 else 0.3)
        ok = np.isfinite(d)
        if ok.sum() < 50:
            return None
        p, q = A[ok, :2], B[j[ok], :2]
        mp, mq = p.mean(0), q.mean(0)
        U, _, Vt = np.linalg.svd((p - mp).T @ (q - mq))
        Rm = (U @ Vt).T
        if np.linalg.det(Rm) < 0:
            Vt[1] *= -1
            Rm = (U @ Vt).T
        th = np.arctan2(Rm[1, 0], Rm[0, 0])
        t = mq - mp @ Rm.T
    return th


def map_struct(x, y, R=28):
    m = (np.abs(MAP[:, 0] - x) < R) & (np.abs(MAP[:, 1] - y) < R)
    L = voxel(np.asarray(MAP[m], dtype=np.float64), 0.1)
    k = np.floor(L[:, :2] / 2.0).astype(np.int64)
    key = k[:, 0] * 100000 + k[:, 1]
    o = np.lexsort((L[:, 2], key))
    u, f, n = np.unique(key[o], return_index=True, return_counts=True)
    gz = np.empty(len(L))
    gz[o] = np.repeat(L[o][:, 2][f + (0.1 * (n - 1)).astype(int)], n)
    hz = L[:, 2] - gz
    S = L[(hz > 0.3) & (hz < 6.0)].copy()
    S[:, 2] = hz[(hz > 0.3) & (hz < 6.0)]
    return S


def work(k):
    n = len(TR)
    if k - a.step < 0 or k + a.step >= min(n, len(IDX)):
        return None
    cs = [cloud(j) for j in (k - a.step, k, k + a.step)]
    if any(c is None for c in cs):
        return None
    (Pm, sm_), (P0, s0), (Pp, sp) = cs
    dist_m = np.hypot(TR.x[k] - TR.x[k - a.step], TR.y[k] - TR.y[k - a.step])
    dist_p = np.hypot(TR.x[k + a.step] - TR.x[k], TR.y[k + a.step] - TR.y[k])
    th_m = icp_yaw(P0, Pm, IY[k] - IY[k - a.step], np.array([dist_m, 0.0]))
    th_p = icp_yaw(Pp, P0, IY[k + a.step] - IY[k], np.array([dist_p, 0.0]))
    if th_m is None or th_p is None:
        return None
    e_m = np.degrees(th_m - (IY[k] - IY[k - a.step]))
    e_p = np.degrees(th_p - (IY[k + a.step] - IY[k]))
    e_s = abs(s0 - np.median([sm_, sp])) / s0
    accept = abs(e_m) < a.gate_yaw and abs(e_p) < a.gate_yaw and e_s < a.gate_scale
    row = dict(idx=k, e_m=e_m, e_p=e_p, e_s=e_s, accept=accept)
    if not accept:
        return row
    x, y, yaw = TR.x[k], TR.y[k], TR.yaw[k]
    tree = cKDTree(map_struct(x, y))
    c, s = np.cos(yaw), np.sin(yaw)
    W0 = np.stack([c * P0[:, 0] - s * P0[:, 1], s * P0[:, 0] + c * P0[:, 1], P0[:, 2]], 1)
    offs = np.arange(-a.span, a.span + 1e-9, a.res)
    g = np.zeros((len(offs), len(offs)))
    for i, f in enumerate(offs):
        for j, l in enumerate(offs):
            d, _ = tree.query(W0 + [x + c * f - s * l, y + s * f + c * l, 0], distance_upper_bound=a.tol)
            g[i, j] = np.isfinite(d).mean()
    i, j = np.unravel_index(np.argmax(g), g.shape)

    def para(v, q):  # sub-cell parabola along one axis
        if 0 < q < len(v) - 1:
            den = v[q - 1] - 2 * v[q] + v[q + 1]
            return float(np.clip(0.5 * (v[q - 1] - v[q + 1]) / den, -0.5, 0.5)) if den < 0 else 0.0
        return 0.0
    f0 = offs[i] + para(g[:, j], i) * a.res
    l0 = offs[j] + para(g[i, :], j) * a.res
    k3 = int(round(0.3 / a.res))
    sf = g[i, j] - max(g[max(i - k3, 0), j], g[min(i + k3, len(offs) - 1), j])
    sl = g[i, j] - max(g[i, max(j - k3, 0)], g[i, min(j + k3, len(offs) - 1)])
    edge = i in (0, len(offs) - 1) or j in (0, len(offs) - 1)
    row.update(mx=x + c * f0 - s * l0, my=y + s * f0 + c * l0, myaw=yaw, fwd=f0, lat=l0, peak=g[i, j], sharp_f=sf,
               sharp_l=sl, edge=edge, scale=s0)
    return row


if __name__ == '__main__':
    lo, hi, st = (int(v) for v in a.frames.split(':'))
    frames = list(range(lo, len(TR) if hi < 0 else hi, st))
    if a.scale_win > 0:   # temporally smoothed scale from the per-frame ground heights
        allf = list(range(0, min(len(TR), len(IDX)), st))
        with Pool(a.workers) as pool:
            gp = pool.map(ground, allf, chunksize=8)
        fr_ok = np.array([f for f, g_ in zip(allf, gp) if g_ is not None])
        sc = np.array([a.cam_h / max(-g_[2], 0.5) for g_ in gp if g_ is not None])
        for f in range(min(len(TR), len(IDX))):
            m_ = np.abs(fr_ok - f) <= a.scale_win
            if m_.any():
                SMOOTH[f] = float(np.median(sc[m_]))
        print(f'scale: per-frame spread {np.std(np.log(sc)) * 100:.1f} %, smoothed over +-{a.scale_win} frames')
    with Pool(a.workers) as pool:
        rows = [r for r in pool.imap(work, frames, chunksize=4) if r is not None]
    R = pd.DataFrame(rows)
    acc = R[R.accept & ~R.get('edge', pd.Series(False, index=R.index)).fillna(True).astype(bool)].copy()
    print(f'{T}: {len(R)} clouds checked, {int(R.accept.sum())} agree with the IMU (|dyaw| < {a.gate_yaw} deg both sides, '
          f'scale < {a.gate_scale * 100:.0f} %), {len(acc)} fixes not at the search edge')
    print(f'   IMU disagreement |dyaw| median {np.median(np.abs(np.r_[R.e_m, R.e_p])):.2f} deg, scale jump median {np.median(R.e_s) * 100:.1f} %')
    ts = FT[acc.idx.values.astype(int)]
    np.savez(a.out, idx=acc.idx.values.astype(int), ts=ts, mx=acc.mx.values, my=acc.my.values, myaw=acc.myaw.values,
             peak=acc.peak.values, sharp_f=acc.sharp_f.values, sharp_l=acc.sharp_l.values, fwd=acc.fwd.values,
             lat=acc.lat.values, scale=acc.scale.values)
    R.to_csv(a.out[:-4] + '_qc.csv', index=False)
    print('saved', a.out, len(acc))
