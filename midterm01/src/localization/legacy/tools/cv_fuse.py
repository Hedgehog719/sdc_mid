#!/usr/bin/env python3
"""Block cross-validation of fuse.py settings on real data (no ground truth needed).

The track is cut into blocks of --block frames; fold k drops the map fixes (lanes, poles, kerb) of every other block,
fuses with the rest (same two-pass recipe as the submissions: pass 1 for the heading, pass 2 with --psi_csv), and
scores the held-out lane fixes: their lateral residual (the precise axis of a lane fix) and, for fixes with a sharp
forward peak (crosswalks / stop lines), the forward residual. The fix noise does not depend on the fusion setting,
so lower held-out residuals mean a better estimator - in particular where fixes are missing (the held-out blocks are
gaps), which is where the GNSS error model matters.

Usage: cv_fuse.py <track_dir> <name> --bev a.npz --pole b.npz --road c.npz --base "<common fuse args>" \
           --variant NAME "<extra fuse args>" [--variant ...]
"""
import argparse
import os
import subprocess
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('name')
ap.add_argument('--bev', required=True)
ap.add_argument('--pole', required=True)
ap.add_argument('--road', required=True)
ap.add_argument('--base', required=True, help='fuse args common to both passes (without --bev/--pole/--road/rts noise)')
ap.add_argument('--pass1', default='--rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1')
ap.add_argument('--pass2', default='--rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05')
ap.add_argument('--variant', nargs=2, action='append', metavar=('NAME', 'ARGS'), required=True)
ap.add_argument('--block', type=int, default=150)
ap.add_argument('--start', type=int, default=330, help='frames before this are not scored (start segment)')
a = ap.parse_args()

tmp = os.path.join(os.path.dirname(__file__), '..', 'work', 'cv')
os.makedirs(tmp, exist_ok=True)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
src = {'bev': dict(np.load(a.bev)), 'pole': dict(np.load(a.pole)), 'road': dict(np.load(a.road))}


def fold_files(k):
    out = {}
    for key, z in src.items():
        blk = (z['idx'] // a.block) % 2
        keep = blk != k
        f = os.path.join(tmp, f'{a.name}_{key}_f{k}.npz')
        np.savez(f, **{kk: v[keep] for kk, v in z.items()})
        out[key] = f
    return out


def run(args):
    r = subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), 'fuse.py'), a.track_dir] + args,
                       capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[-2000:])
        raise SystemExit('fuse failed')


def score(traj, z, test):
    ii = np.searchsorted(ts, z['ts'][test])
    yw = traj.yaw.values[ii]
    dx, dy = z['mx'][test] - traj.x.values[ii], z['my'][test] - traj.y.values[ii]
    fw = np.cos(yw) * dx + np.sin(yw) * dy
    lt = -np.sin(yw) * dx + np.cos(yw) * dy
    return fw, lt


def rob_rms(v):
    v = v[np.isfinite(v)]
    med = np.median(v)
    mad = 1.4826 * np.median(np.abs(v - med)) + 1e-6
    k = np.abs(v - med) < 3 * mad
    return np.sqrt(np.mean(v[k] ** 2)), med


zb = src['bev']
dd = np.abs(np.degrees(zb['dyaw']))
lim = zb['max_yaw'] if 'max_yaw' in zb else np.full(len(dd), dd.max())
okb = dd < lim - 0.01
good_lane = okb & (zb['peak'] >= np.percentile(zb['peak'], 30)) & (zb['idx'] >= a.start)
sharp = good_lane & (zb['sharp_f'] >= np.percentile(zb['sharp_f'][good_lane], 75)) if 'sharp_f' in zb else good_lane
results = {}
for vname, vargs in a.variant:
    lat_all, fwd_all = [], []
    for k in (0, 1):
        ff = fold_files(k)
        fix = ['--bev', ff['bev'], '--pole', ff['pole'], '--road', ff['road']]
        p1 = os.path.join(tmp, f'{a.name}_{vname}_f{k}_p1.csv')
        p2 = os.path.join(tmp, f'{a.name}_{vname}_f{k}.csv')
        run([p1] + a.base.split() + fix + a.pass1.split() + vargs.split())
        run([p2] + a.base.split() + fix + a.pass2.split() + ['--psi_csv', p1] + vargs.split())
        traj = pd.read_csv(p2)
        blk = (zb['idx'] // a.block) % 2
        test = (blk == k)
        _, lt = score(traj, zb, test & good_lane)
        fw, _ = score(traj, zb, test & sharp)
        lat_all.append(lt)
        fwd_all.append(fw)
    lr, lm = rob_rms(np.concatenate(lat_all))
    fr, fm = rob_rms(np.concatenate(fwd_all))
    results[vname] = (lr, lm, fr, fm)
    print(f'{vname:24s} held-out lane fixes: lateral rms {lr:.3f} m (median {lm:+.3f}) | sharp-forward rms {fr:.3f} m '
          f'(median {fm:+.3f})', flush=True)
