#!/usr/bin/env python3
"""Map-to-image localization with pole landmarks (complements road markings in the forward direction).

Poles from the LiDAR map (poles.py) are projected into the three fisheye cameras for a grid of (forward, lateral)
offsets around the prior pose. Each projected pole is scored by the image edge energy across its projected
direction (structure tensor, so thin poles respond at their centre); YOLO road users are masked (pedestrians are
vertical too). The pole azimuth depends on the vehicle position but hardly on camera pitch, so this constrains the
along-track position that lane markings leave ambiguous.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import GroundRaster  # noqa: E402
from camera_model import Cameras  # noqa: E402
from common import ply_frames  # noqa: E402

A = None


def edge_tensor(img, mask):
    g = cv2.GaussianBlur(img.astype(np.float32), (0, 0), 1.0)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    if mask is not None:
        gx[mask] = 0
        gy[mask] = 0
    s = A.tensor_sigma
    Jxx = cv2.GaussianBlur(gx * gx, (0, 0), s)
    Jyy = cv2.GaussianBlur(gy * gy, (0, 0), s)
    Jxy = cv2.GaussianBlur(gx * gy, (0, 0), s)
    norm = float(np.mean(Jxx + Jyy)) + 1e-6
    return Jxx / norm, Jyy / norm, Jxy / norm


def load_frame(i):
    out = {}
    for c in CAMS:
        img = cv2.imread(os.path.join(IMGDIR[c], NAMES[c][i]), cv2.IMREAD_GRAYSCALE)
        mask = None
        if c in MASKS:
            m = np.unpackbits(MASKS[c][i], axis=1)[:, :img.shape[1] // 2]
            mask = cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            mask = cv2.dilate(mask.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
        if c == 'f' and HOOD is not None:
            mask = HOOD if mask is None else (mask | HOOD)
        out[c] = edge_tensor(img, mask)
    return out


def match_frame(i, prior):
    x, y, yaw = prior
    c, s = np.cos(yaw), np.sin(yaw)
    d = POLES[:, :2] - [x, y]
    u = c * d[:, 0] + s * d[:, 1]
    v = -s * d[:, 0] + c * d[:, 1]
    r = np.hypot(u, v)
    sel = (r > A.min_range) & (r < A.max_range)
    if sel.sum() < 1:
        return None
    P = POLES[sel]
    u, v = u[sel], v[sel]
    zv = GROUND.height(np.array([x]), np.array([y]))[0] + A.cam_height
    hs = np.linspace(0.6, 1.0, A.n_samples)[None, :] * np.minimum(P[:, 4:5], A.max_h)  # sample heights per pole
    zrel = P[:, 2:3] + hs - zv                                                         # (np, ns)
    tensors = load_frame(i)
    fw = np.arange(-A.max_fwd, A.max_fwd + 1e-6, A.step)
    lt = np.arange(-A.max_lat, A.max_lat + 1e-6, A.step)
    FW, LT = np.meshgrid(fw, lt, indexing='ij')
    nh = FW.size
    # hypothesis: vehicle moved by (fwd, lat) -> poles appear at (u - fwd, v - lat)
    pu = u[None, :, None] - FW.reshape(-1, 1, 1)
    pv = v[None, :, None] - LT.reshape(-1, 1, 1)
    pts = np.stack(np.broadcast_arrays(pu, pv, zrel[None]), -1).reshape(-1, 3)
    q = pts @ RM
    score = np.zeros((nh, len(P)))
    cnt = np.zeros((nh, len(P)))
    for cname in CAMS:
        uv, ok = CAM.project(cname, q)
        uv = uv.reshape(nh, len(P), A.n_samples, 2)
        ok = ok.reshape(nh, len(P), A.n_samples)
        du = np.gradient(uv, axis=2)
        nrm = np.stack([-du[..., 1], du[..., 0]], -1)
        nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True) + 1e-9
        Jxx, Jyy, Jxy = tensors[cname]
        hgt, wid = Jxx.shape
        xi = np.clip(uv[..., 0].astype(int), 0, wid - 1)
        yi = np.clip(uv[..., 1].astype(int), 0, hgt - 1)
        e = (Jxx[yi, xi] * nrm[..., 0] ** 2 + 2 * Jxy[yi, xi] * nrm[..., 0] * nrm[..., 1] + Jyy[yi, xi] * nrm[..., 1] ** 2)
        e = np.where(ok, e, 0)
        score += e.sum(-1)
        cnt += ok.sum(-1)
    vis = cnt > A.n_samples * 0.5
    per_pole = np.where(vis, score / np.maximum(cnt, 1), 0)
    # robust: cap each pole's contribution so one strong edge (building) cannot dominate
    per_pole = np.minimum(per_pole, A.cap)
    tot = per_pole.sum(1)  # summed over visible poles (more poles -> more evidence)
    return tot.reshape(FW.shape), int(vis.max(0).sum()) if vis.size else 0


def peak_of(grid, npl):
    fw = np.arange(-A.max_fwd, A.max_fwd + 1e-6, A.step)
    lt = np.arange(-A.max_lat, A.max_lat + 1e-6, A.step)
    FW, _ = np.meshgrid(fw, lt, indexing='ij')
    k = int(np.argmax(grid))
    best = grid.max()
    base = np.median(grid)
    kf, kl = np.unravel_index(k, FW.shape)
    # parabolic refinement
    def ref1(a, b, c_):
        den = a - 2 * b + c_
        return 0.5 * (a - c_) / den if den < 0 else 0.0
    dfw = ref1(grid[kf - 1, kl], grid[kf, kl], grid[kf + 1, kl]) if 0 < kf < len(fw) - 1 else 0.0
    dlt = ref1(grid[kf, kl - 1], grid[kf, kl], grid[kf, kl + 1]) if 0 < kl < len(lt) - 1 else 0.0
    # curvature-based sharpness along each axis
    sharp_f = best - 0.5 * (grid[max(kf - 3, 0), kl] + grid[min(kf + 3, len(fw) - 1), kl])
    sharp_l = best - 0.5 * (grid[kf, max(kl - 3, 0)] + grid[kf, min(kl + 3, len(lt) - 1)])
    return (fw[kf] + dfw * A.step, lt[kl] + dlt * A.step, best, best / (base + 1e-6), sharp_f, sharp_l, npl)


def worker(chunk):
    """Score grids per frame, then sum them over +-accum frames (prior error assumed slowly varying)."""
    out = []
    t0 = time.time()
    lo, hi = chunk[0], chunk[-1]
    grids = {}
    for j in range(max(0, lo - A.accum), min(len(PRIOR), hi + A.accum + 1), A.accum_step):
        r = match_frame(j, PRIOR[j])
        if r is not None:
            grids[j] = r
    for n, i in enumerate(chunk):
        x, y, yaw = PRIOR[i]
        acc, npl = None, 0
        for j, (g, nv) in grids.items():
            if abs(j - i) <= A.accum:
                acc = g.copy() if acc is None else acc + g
                npl += nv
        if acc is None or npl < A.min_poles:
            continue
        fwd, lat, best, ratio, sf, sl, npl = peak_of(acc, npl)
        c, s = np.cos(yaw), np.sin(yaw)
        out.append((i, x, y, yaw, x + c * fwd - s * lat, y + s * fwd + c * lat, best, ratio, sf, sl, npl, fwd, lat))
        if n % 50 == 0:
            print(f'[{chunk[0]}] frame {i} {time.time() - t0:.0f}s poles={npl} ratio={ratio:.2f} fwd={fwd:.2f} lat={lat:.2f}',
                  flush=True)
    return out


def main():
    global A, PRIOR, POLES, GROUND, CAM, CAMS, NAMES, IMGDIR, MASKS, HOOD, RM
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('prior_csv')
    ap.add_argument('output_npz')
    ap.add_argument('--poles', default='poles.npy', help='poles file in the track dir (e.g. poles_curated.npy)')
    ap.add_argument('--cam_height', type=float, default=1.5)
    ap.add_argument('--yaw_mount', type=float, default=-1.5)
    ap.add_argument('--min_range', type=float, default=4.0)
    ap.add_argument('--max_range', type=float, default=35.0)
    ap.add_argument('--min_poles', type=int, default=2)
    ap.add_argument('--max_h', type=float, default=5.0)
    ap.add_argument('--n_samples', type=int, default=12)
    ap.add_argument('--max_fwd', type=float, default=2.5)
    ap.add_argument('--max_lat', type=float, default=1.5)
    ap.add_argument('--step', type=float, default=0.1)
    ap.add_argument('--tensor_sigma', type=float, default=2.0)
    ap.add_argument('--cap', type=float, default=8.0)
    ap.add_argument('--accum', type=int, default=0, help='+-frames whose score grids are summed')
    ap.add_argument('--accum_step', type=int, default=3)
    ap.add_argument('--out_step', type=int, default=1)
    ap.add_argument('--prior_noise', type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument('--frames', default='')
    ap.add_argument('--workers', type=int, default=4)
    A = ap.parse_args()
    track = os.path.basename(os.path.normpath(A.track_dir))
    ref = pd.read_csv(A.prior_csv)
    n = len(ref)
    PRIOR = ref[['x', 'y', 'yaw']].values.copy()
    if A.prior_noise[0] > 0 or A.prior_noise[1] > 0:
        rng = np.random.default_rng(0)
        PRIOR[:, :2] += rng.normal(0, A.prior_noise[0], (n, 2))
        PRIOR[:, 2] += np.radians(rng.normal(0, A.prior_noise[1], n))
    POLES = np.load(os.path.join(A.track_dir, A.poles))
    GROUND = GroundRaster(np.load(os.path.join(A.track_dir, 'map.npy'), mmap_mode='r'))
    CAM = Cameras(os.path.join(A.track_dir, f'{track}_images', 'camera_info'))
    CAMS = ['fl', 'f', 'fr']
    IMGDIR = {c: os.path.join(A.track_dir, f'{track}_images', f'{track}_{c}_images') for c in CAMS}
    NAMES = {c: sorted(os.listdir(IMGDIR[c])) for c in CAMS}
    MASKS = {c: np.load(os.path.join(A.track_dir, 'masks', f'{c}.npz'))['masks'] for c in CAMS}
    hood = cv2.imread(os.path.join(A.track_dir, f'{track}_images', 'camera_info', 'gige_100_f_hdr_mask.png'))
    HOOD = hood[..., 0] > 127 if hood is not None else None
    RM = Rotation.from_euler('ZYX', [A.yaw_mount, 0, 0], degrees=True).as_matrix()
    # R_OPT is applied inside Cameras.project; RM is the rig mounting (same convention as bev.ImageBEV)
    lo, hi, st = 0, n, 1
    if A.frames:
        parts = [int(v) for v in A.frames.split(':')]
        lo, hi = parts[:2]
        st = parts[2] if len(parts) > 2 else 1
    frames = list(range(lo, hi, st))
    chunks = [frames[k:k + 40] for k in range(0, len(frames), 40)]
    with mp.get_context('fork').Pool(A.workers) as pool:
        res = pool.map(worker, chunks, chunksize=1)
    rows = np.array([r for c in res for r in c])
    ts, _ = ply_frames(os.path.join(A.track_dir, 'raw_ply'))
    keys = ['idx', 'px', 'py', 'pyaw', 'mx', 'my', 'peak', 'ratio', 'sharp_f', 'sharp_l', 'npoles', 'fwd', 'lat']
    o = {k: rows[:, j] for j, k in enumerate(keys)}
    o['idx'] = o['idx'].astype(int)
    o['ts'] = ts[o['idx']]
    o['myaw'] = o['pyaw']
    np.savez(A.output_npz, **o)
    print('saved', A.output_npz, len(rows))


if __name__ == '__main__':
    main()
