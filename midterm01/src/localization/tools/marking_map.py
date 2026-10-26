#!/usr/bin/env python3
"""Global road-marking map from the LiDAR point cloud (replaces the per-frame top-hat of the map intensity).

The old map-side marking response (bev.lane_response on the per-frame mean-intensity raster) turns single bright
returns into markings: a road cell holds only 2-4 map points, so one stray return lifts its mean, and dividing by
the local std (tiny on uniform asphalt) amplifies it. Here, once per track, on 50 m tiles along the driven corridor:
  1. ground layer: points within -0.15..+0.20 m of a smooth local ground (grey opening of the per-cell minimum)
  2. per 0.1 m cell the MEDIAN intensity of the ground points (a single stray return no longer counts)
  3. background = 1.5 m median; residual scaled by the robust noise of a 5 m neighbourhood (not the 3 m std of
     the old response, which is tiny on uniform asphalt)
  4. oriented line filter: max over 12 directions of a 0.7 m line average - lines, dashes, crosswalk bars and
     arrows survive, isolated speckle is averaged away
  5. hysteresis: weak cells (> lo) are kept when connected to strong ones (> hi), so broken lines stay whole;
     components smaller than min_area are dropped; inside them only cells bright themselves (z > cell) remain, so
     the marking keeps its true width (the line filter alone widens it ~0.25 m per side)
Output npz: tile keys, response (uint8, response x SCALE) and smooth ground height (float16) per tile.
MarkingMap samples it into the vehicle BEV (matcher) or returns 3D marking points (viewer red dots).

usage: marking_map.py <track_dir> <traj_csv> <out_npz> [--hi 3 --lo 1.5 --min_area 8]
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from bev import H, W, bev_grid, local_ground  # noqa: E402

TILE, RES_M = 50.0, 0.1
N = int(round(TILE / RES_M))
MARGIN = 20  # cells of context around each tile
SCALE = 40.0


def line_kernels(length=7, n=12):
    sz = (length + 2) | 1
    c = sz // 2
    ks = []
    for k in range(n):
        th = np.pi * k / n
        K = np.zeros((sz, sz), np.float32)
        dx, dy = np.cos(th) * (length - 1) / 2, np.sin(th) * (length - 1) / 2
        cv2.line(K, (int(round(c - dx)), int(round(c - dy))), (int(round(c + dx)), int(round(c + dy))), 1.0, 1)
        ks.append(K / K.sum())
    return ks


def process_tile(tx, ty, pts, inten, hi, lo, min_area, kernels, cell_thr=1.5, mode='mask'):
    x0, y0 = tx * TILE - MARGIN * RES_M, ty * TILE - MARGIN * RES_M
    n = N + 2 * MARGIN
    ci = np.floor((pts[:, 0] - x0) / RES_M).astype(np.int64)
    cj = np.floor((pts[:, 1] - y0) / RES_M).astype(np.int64)
    ok = (ci >= 0) & (ci < n) & (cj >= 0) & (cj < n)
    ci, cj = ci[ok], cj[ok]
    z, it = pts[ok, 2].astype(np.float64), np.asarray(inten)[ok].astype(np.float32)
    k = ci * n + cj
    zmin = np.full(n * n, np.inf)
    np.minimum.at(zmin, k, z)
    zmin = zmin.reshape(n, n)
    valid = np.isfinite(zmin)
    if valid.sum() < 1000:
        return None
    g = local_ground(np.where(valid, zmin, 0).astype(np.float32), valid, open_m=1.5, smooth_m=0.5)
    hrel = z - g[ci, cj]
    gp = (hrel > -0.15) & (hrel < 0.20)
    kk, ii = k[gp], it[gp]
    o = np.lexsort((ii, kk))
    kk, ii = kk[o], ii[o]
    uk, first, cnt = np.unique(kk, return_index=True, return_counts=True)
    I = np.zeros(n * n, np.float32)
    C = np.zeros(n * n, np.float32)
    I[uk] = ii[first + cnt // 2]
    C[uk] = cnt
    I, C = I.reshape(n, n), C.reshape(n, n)
    v = C > 0
    num = cv2.blur(np.where(v, I, 0).astype(np.float32), (3, 3))
    den = cv2.blur(v.astype(np.float32), (3, 3))
    If = np.where(v, I, np.where(den > 0, num / np.maximum(den, 1e-6), 0)).astype(np.float32)
    vf = v | (den > 0)
    bg = cv2.medianBlur(np.clip(If * 4, 0, 255).astype(np.uint8), 15).astype(np.float32) / 4
    r = np.where(vf, If - bg, 0).astype(np.float32)
    # robust noise scale over a 5 m neighbourhood: asphalt, concrete and vegetation differ, one tile-wide scale
    # suppresses markings in tiles with much clutter
    sig = np.maximum(1.4826 * cv2.medianBlur(np.clip(np.abs(r) * 8, 0, 255).astype(np.uint8), 51).astype(np.float32) / 8,
                     0.8)
    zz = np.clip(r / sig, -3, 8).astype(np.float32)
    # detection: oriented line average (lines / dashes / bars survive, speckle does not) + hysteresis
    e = np.max([cv2.filter2D(zz, -1, K) for K in kernels], axis=0)
    e[~vf] = 0
    weak = (e > lo).astype(np.uint8)
    nlab, lab, st, _ = cv2.connectedComponentsWithStats(weak, connectivity=8)
    has_strong = np.zeros(nlab, bool)
    has_strong[np.unique(lab[e > hi])] = True
    has_strong[0] = False
    keep = (has_strong & (st[:, cv2.CC_STAT_AREA] >= min_area))[lab]
    # shape: only cells that are bright themselves (the line filter widens markings by ~0.25 m per side)
    cellm = (keep & (zz > cell_thr)).astype(np.uint8)
    nlab, lab, st, _ = cv2.connectedComponentsWithStats(cellm, connectivity=8)
    cellm = (st[:, cv2.CC_STAT_AREA] >= 4)[lab] & (lab > 0)
    resp = np.where(cellm, np.clip(zz, 0, 6), 0) if mode == 'mask' else np.clip(e, 0, 6)
    s = slice(MARGIN, MARGIN + N)
    return (resp[s, s] * SCALE).astype(np.uint8), g[s, s].astype(np.float16), float(np.median(sig[vf]))


class MarkingMap:
    def __init__(self, path):
        z = np.load(path)
        self.keys = z['keys'].astype(int)
        self.index = {(int(a), int(b)): k for k, (a, b) in enumerate(self.keys)}
        self.resp, self.ground = z['resp'], z['ground']
        self.gx, self.gy = (v.ravel() for v in bev_grid())

    def _lookup(self, arr, wx, wy, default=0.0):
        tx, ty = np.floor(wx / TILE).astype(np.int64), np.floor(wy / TILE).astype(np.int64)
        ci = np.clip(np.floor((wx - tx * TILE) / RES_M).astype(np.int64), 0, N - 1)
        cj = np.clip(np.floor((wy - ty * TILE) / RES_M).astype(np.int64), 0, N - 1)
        out = np.full(len(wx), default, np.float32)
        ok = np.zeros(len(wx), bool)
        key = tx * 100003 + ty
        for kv in np.unique(key):
            m = key == kv
            t = self.index.get((int(tx[m][0]), int(ty[m][0])))
            if t is None:
                continue
            out[m] = arr[t][ci[m], cj[m]]
            ok[m] = True
        return out, ok

    def sample(self, wx, wy):
        """Bilinear marking response at world points (cell centres at (i + 0.5) * RES_M)."""
        fx, fy = wx / RES_M - 0.5, wy / RES_M - 0.5
        x0, y0 = np.floor(fx), np.floor(fy)
        ax, ay = (fx - x0).astype(np.float32), (fy - y0).astype(np.float32)
        acc = np.zeros(len(wx), np.float32)
        okall = np.zeros(len(wx), bool)
        for dx, dy, w in ((0, 0, (1 - ax) * (1 - ay)), (1, 0, ax * (1 - ay)), (0, 1, (1 - ax) * ay), (1, 1, ax * ay)):
            v, ok = self._lookup(self.resp, (x0 + dx + 0.5) * RES_M, (y0 + dy + 0.5) * RES_M)
            acc += w * v
            okall |= ok
        return acc / SCALE, okall

    def bev(self, pose_xy, yaw):
        """Marking response in the vehicle BEV raster (H, W) at pose (x, y, yaw)."""
        c, s = np.cos(yaw), np.sin(yaw)
        wx = pose_xy[0] + c * self.gx - s * self.gy
        wy = pose_xy[1] + s * self.gx + c * self.gy
        r, ok = self.sample(wx, wy)
        return r.reshape(H, W), ok.reshape(H, W)

    def height(self, wx, wy):
        g, ok = self._lookup(self.ground, wx, wy, np.nan)
        return g.astype(np.float64), ok

    def background(self, x, y, radius, n, rng):
        """World (x, y, z) of n random non-marking cells within radius (road-level filtering by the caller)."""
        out = []
        for (tx, ty), t in self.index.items():
            cx, cy = (tx + 0.5) * TILE, (ty + 0.5) * TILE
            if np.hypot(cx - x, cy - y) > radius + TILE * 0.71:
                continue
            ii, jj = np.nonzero(self.resp[t] == 0)
            k = rng.choice(len(ii), min(len(ii), 4 * n), replace=False)
            ii, jj = ii[k], jj[k]
            px, py = tx * TILE + (ii + 0.5) * RES_M, ty * TILE + (jj + 0.5) * RES_M
            m = np.hypot(px - x, py - y) < radius
            out.append(np.stack([px[m], py[m], self.ground[t][ii[m], jj[m]].astype(np.float64)], 1))
        P = np.concatenate(out) if out else np.zeros((0, 3))
        return P[rng.choice(len(P), min(len(P), 3 * n), replace=False)] if len(P) else P

    def points(self, x, y, radius, thr=0.5, with_w=False):
        """World (x, y, z) of marking cells (response > thr) within radius of (x, y); with_w: also the response."""
        out, wts = [], []
        for (tx, ty), t in self.index.items():
            cx, cy = (tx + 0.5) * TILE, (ty + 0.5) * TILE
            if np.hypot(cx - x, cy - y) > radius + TILE * 0.71:
                continue
            ii, jj = np.nonzero(self.resp[t] > thr * SCALE)
            if not len(ii):
                continue
            px, py = tx * TILE + (ii + 0.5) * RES_M, ty * TILE + (jj + 0.5) * RES_M
            m = np.hypot(px - x, py - y) < radius
            out.append(np.stack([px[m], py[m], self.ground[t][ii[m], jj[m]].astype(np.float64)], 1))
            wts.append(self.resp[t][ii[m], jj[m]].astype(np.float32) / SCALE)
        P = np.concatenate(out) if out else np.zeros((0, 3))
        if with_w:
            return P, (np.concatenate(wts) if wts else np.zeros(0, np.float32))
        return P


def rot_z_to(n):
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(v / s * np.arcsin(s)).as_matrix()


def vehicle_frame(P, pose, height_fn, cam_height=1.5):
    """World points -> vehicle frame (x fwd, y left, z up; camera reference at the origin, road -cam_height below
    it), with the road tilt under the vehicle from the map ground (as ImageBEV3D)."""
    from scipy.spatial.transform import Rotation
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    ox, oy = np.meshgrid(np.linspace(-4, 4, 9), np.linspace(-2, 2, 5))
    px, py = x + c * ox.ravel() - s * oy.ravel(), y + s * ox.ravel() + c * oy.ravel()
    pz, ok = height_fn(px, py)
    ok &= np.isfinite(pz)
    A = np.stack([px[ok] - x, py[ok] - y, np.ones(ok.sum())], 1)
    a, b, z0 = np.linalg.lstsq(A, pz[ok], rcond=None)[0]
    nrm = np.array([-a, -b, 1.0])
    R = rot_z_to(nrm / np.linalg.norm(nrm)) @ Rotation.from_euler('z', yaw).as_matrix()
    cam = np.array([x, y, z0]) + R @ np.array([0, 0, cam_height])
    return (P - cam) @ R


def road_level(q, cam_height=1.5, tol=0.4, slope=0.06):
    """Vehicle-frame points (vehicle_frame) lying on the road level: the corridor also holds bright cells on
    walls, roofs and raised ground beside the road, which are not road markings."""
    return np.abs(q[:, 2] + cam_height) < tol + slope * np.hypot(q[:, 0], q[:, 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv', help="driven path: tiles are built along it; 'ALL' = every occupied map tile (no trajectory)")
    ap.add_argument('out_npz')
    ap.add_argument('--hi', type=float, default=3.0)
    ap.add_argument('--lo', type=float, default=1.5)
    ap.add_argument('--min_area', type=int, default=8)
    ap.add_argument('--cell', type=float, default=1.5, help='z-score a cell itself needs inside a detected marking')
    ap.add_argument('--corridor', type=float, default=34.0)
    ap.add_argument('--mode', choices=['mask', 'soft'], default='mask',
                    help='mask: detected markings only (red dots); soft: continuous line response (matcher)')
    a = ap.parse_args()
    import pandas as pd
    from bev_match import load_map
    M = load_map(a.track_dir)
    tiles = set()
    if a.traj_csv == 'ALL':   # no trajectory (Track03, GNSS-free): every map tile with enough points
        tk, cnt = np.unique(np.floor(np.asarray(M.pts[::20, :2]) / TILE).astype(np.int64), axis=0, return_counts=True)
        tiles = {(int(x_), int(y_)) for (x_, y_), c_ in zip(tk, cnt) if c_ * 20 >= 1000}
        print(f'ALL: {len(tiles)} occupied tiles', flush=True)
    else:
        tr = pd.read_csv(a.traj_csv)
        for px, py in tr[['x', 'y']].values[::10]:
            for tx in range(int(np.floor((px - a.corridor) / TILE)), int(np.floor((px + a.corridor) / TILE)) + 1):
                for ty in range(int(np.floor((py - a.corridor) / TILE)), int(np.floor((py + a.corridor) / TILE)) + 1):
                    tiles.add((tx, ty))
    tiles = sorted(tiles)
    kernels = line_kernels()
    keys, resp, ground, sigs = [], [], [], []
    t0 = time.time()
    for n_, (tx, ty) in enumerate(tiles):
        cx, cy = (tx + 0.5) * TILE, (ty + 0.5) * TILE
        idx = M.query_idx(cx, cy, TILE * 0.71 + MARGIN * RES_M + 1)
        if len(idx) < 1000:
            continue
        r = process_tile(tx, ty, M.pts[idx], M.inten[idx], a.hi, a.lo, a.min_area, kernels, a.cell, a.mode)
        if r is None:
            continue
        keys.append((tx, ty))
        resp.append(r[0])
        ground.append(r[1])
        sigs.append(r[2])
        if n_ % 10 == 0:
            print(f'tile {n_ + 1}/{len(tiles)} ({tx},{ty}) points {len(idx)} noise sigma {r[2]:.2f} marking cells '
                  f'{(r[0] > 0).sum()} {time.time() - t0:.0f}s', flush=True)
    np.savez_compressed(a.out_npz, keys=np.array(keys), resp=np.array(resp), ground=np.array(ground),
                        sig=np.array(sigs), hi=a.hi, lo=a.lo, min_area=a.min_area)
    print(f'saved {a.out_npz}: {len(keys)} tiles, {sum((r > 0).sum() for r in resp)} marking cells')


if __name__ == '__main__':
    main()
