#!/usr/bin/env python3
"""Frame-wise registration of image point clouds to the full-resolution LiDAR map.

Per frame:
  1. prior pose: GNSS x/y, IMU yaw (offset-corrected), roll/pitch from the local map ground plane
     combined with the cloud's own ground plane, z = map ground + cloud camera height.
  2. dynamic-object / noise removal: neighbouring frames are moved into the current frame with the
     GNSS+IMU ego motion; points without support in enough neighbours (moving cars, depth noise) are dropped.
  3. coarse-to-fine GICP against the full-resolution map submap.
  4. map-consistency filter (drop points far from the map after alignment) + final GICP.
Outputs per-frame pose, GICP Hessian and quality stats to an .npz consumed by fuse.py.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

import cv2
import numpy as np
import small_gicp
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from common import Sensors, TiledMap, ply_frames, wrap, yaw_of  # noqa: E402
from plyio import read_ply_xyz  # noqa: E402
from camera_model import Cameras  # noqa: E402

ARGS = None
MAP = None
SENS = None
MASKS = None
CAMS = None
MASK_KERNEL = None


def fit_plane(p):
    c = p.mean(0)
    n = np.linalg.svd(p - c, full_matrices=False)[2][2]
    return (n if n[2] > 0 else -n), c


def ground_plane(p, th, iters=4):
    q = p
    for _ in range(iters):
        n, c = fit_plane(q)
        q = p[np.abs((p - c) @ n) < th]
        if len(q) < 10:
            break
    return n, c


def rot_z_to(n):
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, n)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    return Rotation.from_rotvec(v / s * np.arcsin(s)).as_matrix()


def mask_filter(i, p):
    """Drop points that project into a YOLO road-user mask in any camera (half-res masks, extra dilation)."""
    keep = np.ones(len(p), bool)
    for cam, masks in MASKS.items():
        uv, valid = CAMS.project(cam, p / ARGS.scale)
        if not valid.any():
            continue
        m = np.unpackbits(masks[i], axis=1)
        if MASK_KERNEL is not None:
            m = cv2.dilate(m, MASK_KERNEL)
        u = (uv[valid, 0] / 2).astype(int).clip(0, m.shape[1] - 1)
        v = (uv[valid, 1] / 2).astype(int).clip(0, m.shape[0] - 1)
        keep[np.flatnonzero(valid)[m[v, u] > 0]] = False
    return keep


def load_cloud(i, path):
    p = read_ply_xyz(path).astype(np.float64) * ARGS.scale
    r = np.linalg.norm(p, axis=1)
    p = p[(r > ARGS.min_range) & (r < ARGS.max_range)]
    if MASKS:
        p = p[mask_filter(i, p)]
    pc, _ = small_gicp.preprocess_points(p, downsampling_resolution=ARGS.src_voxel, num_threads=1)
    return pc.points()[:, :3].copy()


def prior(i, cloud):
    t = SENS_TS[i]
    x, y, z = SENS.gnss_xyz(t)[0]
    yaw = SENS.imu_yaw(t)[0]
    loc = MAP.query(x, y, 5.0).astype(np.float64)
    loc = loc[loc[:, 2] < z]
    if len(loc) > 50:
        loc = loc[loc[:, 2] < np.percentile(loc[:, 2], 40)]
        nm, cm = ground_plane(loc, 0.15)
        gz = cm[2] - (nm[0] * (x - cm[0]) + nm[1] * (y - cm[1])) / nm[2]
    else:
        nm, gz = np.array([0, 0, 1.0]), z - 2.1
    m = (cloud[:, 0] > 2) & (cloud[:, 0] < 15) & (np.abs(cloud[:, 1]) < 4) & (cloud[:, 2] < 0)
    if m.sum() > 50:
        ns, cs = ground_plane(cloud[m], 0.1)
        hs = -(ns @ cs)
    else:
        ns, hs = np.array([0, 0, 1.0]), 2.1
    P = np.eye(4)
    P[:3, :3] = rot_z_to(nm) @ Rotation.from_euler('z', yaw).as_matrix() @ rot_z_to(ns).T
    P[:3, 3] = [x, y, gz + hs]
    return P


def dynamic_filter(i, clouds, priors):
    """Keep points of frame i that are supported by >= min_support neighbouring frames after ego-motion."""
    p = clouds[i]
    Pi_inv = np.linalg.inv(priors[i])
    r = np.linalg.norm(p, axis=1)
    th = ARGS.dyn_th + ARGS.dyn_th_range * r
    support = np.zeros(len(p), np.int32)
    used = 0
    for off in ARGS.dyn_offsets:
        j = i + off
        if j not in clouds:
            continue
        T = Pi_inv @ priors[j]
        q = clouds[j] @ T[:3, :3].T + T[:3, 3]
        d, _ = cKDTree(q).query(p, k=1, distance_upper_bound=float(th.max()), workers=1)
        support += d < th
        used += 1
    if used == 0:
        return p, 1.0
    keep = support >= min(ARGS.dyn_min_support, used)
    return p[keep], keep.mean()


class Target:
    """Full-resolution map submap, rebuilt only when the vehicle leaves the central area."""

    def __init__(self):
        self.center = None

    def get(self, x, y):
        if self.center is None or np.hypot(x - self.center[0], y - self.center[1]) > ARGS.rebuild_dist:
            sub = MAP.query(x, y, ARGS.submap_radius).astype(np.float64)
            if ARGS.map_voxel > 0:
                self.pc, self.tree = small_gicp.preprocess_points(sub, downsampling_resolution=ARGS.map_voxel,
                                                                  num_neighbors=20, num_threads=ARGS.threads)
            else:
                self.pc = small_gicp.PointCloud(sub)
                self.tree = small_gicp.KdTree(self.pc, num_threads=ARGS.threads)
                small_gicp.estimate_covariances(self.pc, self.tree, num_neighbors=20, num_threads=ARGS.threads)
            self.center = (x, y)
        return self.pc, self.tree


def gicp(tgt, tree, src, T, mcds):
    spc = small_gicp.PointCloud(src)
    stree = small_gicp.KdTree(spc, num_threads=ARGS.threads)
    small_gicp.estimate_covariances(spc, stree, num_neighbors=20, num_threads=ARGS.threads)
    res = None
    for mcd in mcds:
        res = small_gicp.align(tgt, spc, tree, T, registration_type='GICP', max_correspondence_distance=mcd,
                               num_threads=ARGS.threads, max_iterations=50)
        T = res.T_target_source
    return T, res


def worker(chunk):
    global MAP, SENS, SENS_TS
    lo, hi = chunk
    pad = max(abs(o) for o in ARGS.dyn_offsets)
    ids = range(max(0, lo - pad), min(len(FILES), hi + pad))
    clouds, priors = {}, {}
    target = Target()
    out = {k: [] for k in ['idx', 'T', 'P', 'H', 'inlier', 'error', 'npts', 'dyn_keep', 'map_keep']}
    t0 = time.time()

    def ensure(j):
        if j not in clouds and 0 <= j < len(FILES):
            clouds[j] = load_cloud(j, FILES[j])
            priors[j] = prior(j, clouds[j])

    for i in range(lo, hi):
        for j in range(i - pad, i + pad + 1):
            ensure(j)
        for j in [k for k in clouds if k < i - pad]:
            del clouds[j], priors[j]
        P = priors[i]
        src, dyn_keep = dynamic_filter(i, clouds, priors) if ARGS.dyn_offsets else (clouds[i], 1.0)
        tgt, tree = target.get(P[0, 3], P[1, 3])
        T, res = gicp(tgt, tree, src, P, ARGS.mcds)
        # map-consistency: remove points the map cannot explain, then refine
        q = src @ T[:3, :3].T + T[:3, 3]
        if ARGS.map_th > 0:
            nn = tree.batch_nearest_neighbor_search(q, num_threads=ARGS.threads)
            dist = np.sqrt(np.asarray(nn[1]))
            keep = dist < ARGS.map_th
            map_keep = keep.mean()
            if keep.sum() > 200:
                T, res = gicp(tgt, tree, src[keep], T, ARGS.final_mcds)
                src = src[keep]
        else:
            map_keep = 1.0
        out['idx'].append(i)
        out['T'].append(T)
        out['P'].append(P)
        out['H'].append(np.asarray(res.H))
        out['inlier'].append(res.num_inliers / max(len(src), 1))
        out['error'].append(res.error / max(res.num_inliers, 1))
        out['npts'].append(len(src))
        out['dyn_keep'].append(dyn_keep)
        out['map_keep'].append(map_keep)
        if (i - lo) % 50 == 0:
            d = T[:3, 3] - P[:3, 3]
            print(f'[{lo}-{hi}] frame {i} {time.time() - t0:.0f}s dyn_keep={dyn_keep:.2f} map_keep={map_keep:.2f} '
                  f'inl={out["inlier"][-1]:.2f} d=({d[0]:.2f},{d[1]:.2f},{d[2]:.2f}) '
                  f'dyaw={np.degrees(wrap(yaw_of(T) - yaw_of(P))):.2f}', flush=True)
    return {k: np.array(v) for k, v in out.items()}


def init_worker():
    global MAP, SENS, SENS_TS, MASKS, CAMS, MASK_KERNEL
    MASKS = {}
    if ARGS.masks:
        track = os.path.basename(os.path.normpath(ARGS.track_dir))
        CAMS = Cameras(os.path.join(ARGS.track_dir, f'{track}_images', 'camera_info'))
        if ARGS.mask_dilate > 0:
            MASK_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * ARGS.mask_dilate + 1,) * 2)
        for cam in ['f', 'fl', 'fr']:
            MASKS[cam] = np.load(os.path.join(ARGS.track_dir, 'masks', f'{cam}.npz'))['masks']
    MAP = TiledMap(ARGS.map_npy)
    SENS = Sensors(ARGS.track_dir)
    SENS_TS = TS


def main():
    global ARGS, FILES, TS
    parser = argparse.ArgumentParser()
    parser.add_argument('track_dir')
    parser.add_argument('output_npz')
    parser.add_argument('--ply_dir', default='3cameras_ply/ply')
    parser.add_argument('--map_npy', default=None)
    parser.add_argument('--scale', type=float, default=1.0)
    parser.add_argument('--min_range', type=float, default=2.5)
    parser.add_argument('--max_range', type=float, default=60.0)
    parser.add_argument('--src_voxel', type=float, default=0.3)
    parser.add_argument('--map_voxel', type=float, default=0.0, help='0 = full resolution map')
    parser.add_argument('--submap_radius', type=float, default=90.0)
    parser.add_argument('--rebuild_dist', type=float, default=15.0)
    parser.add_argument('--mcds', type=float, nargs='+', default=[4.0, 2.0, 1.0])
    parser.add_argument('--final_mcds', type=float, nargs='+', default=[1.0, 0.5])
    parser.add_argument('--map_th', type=float, default=1.0)
    parser.add_argument('--dyn_offsets', type=int, nargs='*', default=[-8, -4, 4, 8])
    parser.add_argument('--dyn_th', type=float, default=0.4)
    parser.add_argument('--dyn_th_range', type=float, default=0.02)
    parser.add_argument('--dyn_min_support', type=int, default=2)
    parser.add_argument('--masks', action='store_true', help='remove YOLO road-user pixels (needs masks/)')
    parser.add_argument('--mask_dilate', type=int, default=5, help='extra dilation in half-res pixels')
    parser.add_argument('--frames', type=str, default='', help='lo:hi[:step] subset for quick tests')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--threads', type=int, default=6)
    parser.add_argument('--chunk', type=int, default=100)
    ARGS = parser.parse_args()
    ARGS.map_npy = ARGS.map_npy or os.path.join(ARGS.track_dir, 'map.npy')
    TS, FILES = ply_frames(os.path.join(ARGS.track_dir, ARGS.ply_dir))
    lo, hi, step = 0, len(FILES), None
    if ARGS.frames:
        parts = [int(v) for v in ARGS.frames.split(':')]
        lo, hi = parts[0], parts[1]
        step = parts[2] if len(parts) > 2 else None
    if step:  # sparse test: tiny chunks spread over the track
        chunks = [(i, min(i + 1, hi)) for i in range(lo, hi, step)]
    else:
        chunks = [(i, min(i + ARGS.chunk, hi)) for i in range(lo, hi, ARGS.chunk)]
    with mp.get_context('fork').Pool(ARGS.workers, initializer=init_worker) as pool:
        results = pool.map(worker, chunks, chunksize=1)
    merged = {k: np.concatenate([r[k] for r in results]) for k in results[0]}
    merged['ts'] = TS[merged['idx']]
    np.savez(ARGS.output_npz, **merged)
    print('saved', ARGS.output_npz, len(merged['idx']))


if __name__ == '__main__':
    main()
