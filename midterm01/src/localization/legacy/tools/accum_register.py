#!/usr/bin/env python3
"""Multi-frame accumulated-submap registration.

Single image clouds are too noisy/distorted for reliable map matching. Here the filtered clouds of a window
of frames (+-W) are stacked into the window-centre frame using the smooth relative motion of a fused
trajectory (fuse.py CSV: x, y, yaw; tilt/z from register.py priors), then the accumulated cloud is GICP-aligned
to the full-resolution map. One correction per window centre (every --stride frames) is written to an .npz
with the same keys as register.py so fuse.py can consume it.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

import numpy as np
import pandas as pd
import small_gicp
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
import register as R  # noqa: E402
from common import Sensors, TiledMap, ply_frames, wrap, yaw_of  # noqa: E402

A = None


def poses_from_traj(prior_P, traj):
    """Replace x, y, yaw of the register.py priors with the fused trajectory (keep tilt and z)."""
    P = prior_P.copy()
    for k in range(len(P)):
        dyaw = traj.yaw.values[k] - yaw_of(P[k])
        P[k, :3, :3] = Rotation.from_euler('z', dyaw).as_matrix() @ P[k, :3, :3]
        P[k, 0, 3] = traj.x.values[k]
        P[k, 1, 3] = traj.y.values[k]
    return P


def worker(centres):
    out = {k: [] for k in ['idx', 'T', 'P', 'H', 'inlier', 'error', 'npts', 'dyn_keep', 'map_keep']}
    clouds = {}
    target = R.Target()
    t0 = time.time()
    pad = max(abs(o) for o in R.ARGS.dyn_offsets) if R.ARGS.dyn_offsets else 0
    for c in centres:
        lo, hi = max(0, c - A.window), min(len(R.FILES), c + A.window + 1)
        for j in range(lo - pad, hi + pad):
            if 0 <= j < len(R.FILES) and j not in clouds:
                clouds[j] = R.load_cloud(j, R.FILES[j])
        for j in [k for k in clouds if k < lo - pad]:
            del clouds[j]
        priors = {j: POSES[j] for j in clouds}
        Pc = POSES[c]
        Pc_inv = np.linalg.inv(Pc)
        stack = []
        for j in range(lo, hi, A.frame_step):
            p, _ = R.dynamic_filter(j, clouds, priors) if R.ARGS.dyn_offsets else (clouds[j], 1.0)
            p = p[np.linalg.norm(p, axis=1) < A.accum_range]
            T = Pc_inv @ POSES[j]
            stack.append(p @ T[:3, :3].T + T[:3, 3])
        src = np.concatenate(stack)
        src = small_gicp.voxelgrid_sampling(small_gicp.PointCloud(src), A.accum_voxel).points()[:, :3].copy()
        tgt, tree = target.get(Pc[0, 3], Pc[1, 3])
        T, res = R.gicp(tgt, tree, src, Pc, R.ARGS.mcds)
        q = src @ T[:3, :3].T + T[:3, 3]
        nn = tree.batch_nearest_neighbor_search(q, num_threads=R.ARGS.threads)
        keep = np.sqrt(np.asarray(nn[1])) < R.ARGS.map_th
        if keep.sum() > 500:
            T, res = R.gicp(tgt, tree, src[keep], T, R.ARGS.final_mcds)
        out['idx'].append(c)
        out['T'].append(T)
        out['P'].append(Pc)
        out['H'].append(np.asarray(res.H))
        out['inlier'].append(res.num_inliers / max(keep.sum(), 1))
        out['error'].append(res.error / max(res.num_inliers, 1))
        out['npts'].append(len(src))
        out['dyn_keep'].append(1.0)
        out['map_keep'].append(keep.mean())
        d = T[:3, 3] - Pc[:3, 3]
        yw = yaw_of(Pc)
        print(f'centre {c} {time.time() - t0:.0f}s pts={len(src)} map_keep={keep.mean():.2f} '
              f'fwd={np.cos(yw) * d[0] + np.sin(yw) * d[1]:.2f} lat={-np.sin(yw) * d[0] + np.cos(yw) * d[1]:.2f} '
              f'dyaw={np.degrees(wrap(yaw_of(T) - yw)):.2f}', flush=True)
    return {k: np.array(v) for k, v in out.items()}


def init_worker():
    R.init_worker()


def main():
    global A, POSES
    p = argparse.ArgumentParser()
    p.add_argument('track_dir')
    p.add_argument('traj_csv', help='fused trajectory from fuse.py')
    p.add_argument('prior_npz', help='register.py output (for per-frame tilt/z priors)')
    p.add_argument('output_npz')
    p.add_argument('--window', type=int, default=30)
    p.add_argument('--frame_step', type=int, default=2)
    p.add_argument('--stride', type=int, default=10)
    p.add_argument('--accum_range', type=float, default=35.0)
    p.add_argument('--accum_voxel', type=float, default=0.25)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--threads', type=int, default=6)
    p.add_argument('--frames', default='')
    A, rest = p.parse_known_args()
    # register.py options (filters, GICP stages) parsed from the remaining args
    sys.argv = [sys.argv[0], A.track_dir, '/dev/null', '--masks', '--workers', str(A.workers),
                '--threads', str(A.threads)] + rest
    rp = R.argparse.ArgumentParser()
    # re-use register's parser definition by calling its main parser builder
    R_args = build_register_args(sys.argv[1:])
    R.ARGS = R_args
    R.TS, R.FILES = ply_frames(os.path.join(A.track_dir, R_args.ply_dir))
    z = np.load(A.prior_npz)
    order = np.argsort(z['idx'])
    traj = pd.read_csv(A.traj_csv)
    POSES = poses_from_traj(z['P'][order], traj)
    lo, hi = 0, len(R.FILES)
    if A.frames:
        lo, hi = [int(v) for v in A.frames.split(':')]
    centres = list(range(lo, hi, A.stride))
    chunks = [centres[i:i + 15] for i in range(0, len(centres), 15)]
    with mp.get_context('fork').Pool(A.workers, initializer=init_worker) as pool:
        results = pool.map(worker, chunks, chunksize=1)
    merged = {k: np.concatenate([r[k] for r in results]) for k in results[0]}
    merged['ts'] = R.TS[merged['idx']]
    np.savez(A.output_npz, **merged)
    print('saved', A.output_npz, len(merged['idx']))


def build_register_args(argv):
    """Parse register.py options without running it (mirrors register.main's parser)."""
    saved = sys.argv
    sys.argv = [saved[0]] + argv
    captured = {}
    real_pool = R.mp.get_context

    class _Stop(Exception):
        pass

    def fake_ctx(*_a, **_k):
        captured['args'] = R.ARGS
        raise _Stop()
    R.mp.get_context = fake_ctx
    try:
        R.main()
    except _Stop:
        pass
    finally:
        R.mp.get_context = real_pool
        sys.argv = saved
    return captured['args']


if __name__ == '__main__':
    main()
