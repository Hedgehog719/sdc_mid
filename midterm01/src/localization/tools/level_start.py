#!/usr/bin/env python3
"""Drop map scans near the track start whose ground is not at the height of the road further on.

Assumption (the street is flat there): the road at the start lies at the height measured on a trusted stretch
(ref frames, e.g. Track1 210-330). After clean_map.py the start still holds scans recorded just before the stop
(+0.35 m) and just after it (SLAM height still settling, up to -0.6 m); scans whose own ground (mode of the
height residuals in the road corridor) is offset by more than tol are removed.

Reads map_<tag_in>.npy, writes map_<tag_out>.npy (and intensities) in the track dir.
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree


def level_pitch(a, P, I, xyz, fix_path, z_ref):
    """Per block: fit height offset + slope along the path to its own-lane road points (lowest layer per 1 m),
    referred to the level road at z_ref, and subtract it (pitch / height error of scans after the stop)."""
    sp = np.r_[0, np.cumsum(np.hypot(*np.diff(fix_path, axis=0).T))]
    tree = cKDTree(fix_path)
    d, j = tree.query(xyz[:, :2], distance_upper_bound=40.0)
    near = np.isfinite(d)
    jj = np.where(near, j, 0)
    hx, hy = np.gradient(fix_path[:, 0]), np.gradient(fix_path[:, 1])
    n = np.hypot(hx, hy)
    hx, hy = hx / n, hy / n
    lat = -hy[jj] * (xyz[:, 0] - fix_path[jj, 0]) + hx[jj] * (xyz[:, 1] - fix_path[jj, 1])
    sa = sp[jj] + hx[jj] * (xyz[:, 0] - fix_path[jj, 0]) + hy[jj] * (xyz[:, 1] - fix_path[jj, 1])
    strip = near & (np.abs(lat) < a.strip) & (d < a.strip + 0.5)
    zc = xyz[:, 2].copy()
    K = a.block
    nfix = 0
    for b in range((len(P) + K - 1) // K):
        sl = slice(b * K, min((b + 1) * K, len(P)))
        m = strip[sl]
        if m.sum() < 300:
            continue
        ss, zz = sa[sl][m], xyz[sl][m, 2] - z_ref
        keep = np.abs(zz) < 1.0
        ss, zz = ss[keep], zz[keep]
        if len(ss) < 300:
            continue
        bins = np.floor(ss).astype(int)
        ub = np.unique(bins)
        bs, bz = [], []
        for u in ub:
            zb = zz[bins == u]
            if len(zb) >= 10:
                bs.append(u + 0.5)
                bz.append(np.percentile(zb, 20))
        bs, bz = np.array(bs), np.array(bz)
        if len(bs) < 5 or np.ptp(bs) < 5:
            continue
        w = np.ones(len(bs), bool)
        for _ in range(3):
            coef = np.polyfit(bs[w], bz[w], 1)
            rr = bz - np.polyval(coef, bs)
            w = np.abs(rr) < max(2.5 * np.median(np.abs(rr[w])), 0.05)
        lo_s, hi_s = bs.min() - 10, bs.max() + 10
        corr_at = lambda q: np.polyval(coef, np.clip(q, lo_s, hi_s))  # noqa: E731
        span = np.abs(corr_at(np.array([bs.min(), bs.max()])))
        if span.max() <= a.tol:
            continue
        blk_s = np.where(near[sl], sa[sl], np.clip(sa[sl], lo_s, hi_s))
        zc[sl] = xyz[sl][:, 2] - corr_at(blk_s)
        nfix += 1
        print(f'  block {b * K / 1e6:7.3f}M  s {bs.min():5.1f}..{bs.max():5.1f} m  offset {coef[1] + coef[0] * bs.mean():+.2f} m '
              f'slope {np.degrees(np.arctan(coef[0])):+.2f} deg -> corrected')
    print(f'pitch-corrected {nfix} blocks')
    out = np.asarray(P).copy()
    out[:, 2] = zc.astype(out.dtype)
    np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), out)
    np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv')
    ap.add_argument('--tag_in', default='clean')
    ap.add_argument('--tag_out', default='clean3')
    ap.add_argument('--ref_frames', default='210:330')
    ap.add_argument('--fix_frames', default='0:210')
    ap.add_argument('--ahead', type=float, default=30.0, help='m of road beyond the fix frames that is checked')
    ap.add_argument('--corridor', type=float, default=8.0, help='half width (m) of the road corridor')
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--block', type=int, default=20000)
    ap.add_argument('--tilt', action='store_true', help='correct: also remove the block ground tilt (ill-posed for sweep sectors)')
    ap.add_argument('--strip', type=float, default=1.5, help='pitch mode: half width (m) of the own-lane strip')
    ap.add_argument('--mode', choices=['remove', 'correct', 'pitch'], default='pitch',
                    help='correct: subtract the block\'s ground plane offset (keeps coverage); remove: drop the block')
    a = ap.parse_args()
    P = np.load(os.path.join(a.track_dir, f'map_{a.tag_in}.npy'), mmap_mode='r')
    I = np.load(os.path.join(a.track_dir, f'map_intensity_{a.tag_in}.npy'), mmap_mode='r')
    tr = pd.read_csv(a.traj_csv)
    x, y = tr.x.values, tr.y.values
    s = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    r0, r1 = [int(v) for v in a.ref_frames.split(':')]
    f0, f1 = [int(v) for v in a.fix_frames.split(':')]
    # dense path samples of the reference stretch and of the checked stretch (fix frames + ahead)
    def path(sa, sb):
        ss = np.arange(sa, sb, 0.5)
        return np.stack([np.interp(ss, s, x), np.interp(ss, s, y)], 1)
    ref_path = path(s[r0], s[r1])
    fix_path = path(max(s[f0] - 5, 0), min(s[f1] + a.ahead, s[-1]))
    xyz = np.asarray(P, dtype=np.float64)
    tree_ref, tree_fix = cKDTree(ref_path), cKDTree(fix_path)
    # reference road height: low layer of the lane strip along the trusted stretch
    d, _ = tree_ref.query(xyz[:, :2], distance_upper_bound=1.5)
    zr = xyz[np.isfinite(d), 2]
    lo = np.percentile(zr, 5)
    z_ref = float(np.median(zr[zr < lo + 0.3]))
    print(f'reference road height (frames {r0}-{r1}): {z_ref:.3f} m from {len(zr)} points')
    if a.mode == 'pitch':
        return level_pitch(a, P, I, xyz, fix_path, z_ref)
    d, _ = tree_fix.query(xyz[:, :2], distance_upper_bound=a.corridor)
    inc = np.isfinite(d)
    K = a.block
    keep = np.ones(len(P), bool)
    zc = xyz[:, 2].copy()
    bins = np.arange(-0.8, 0.8001, 0.02)
    nrem = 0
    for b in range((len(P) + K - 1) // K):
        sl = slice(b * K, min((b + 1) * K, len(P)))
        m = inc[sl]
        if m.sum() < 200:
            continue
        q = xyz[sl][m]
        r = q[:, 2] - z_ref
        w = np.abs(r) < 0.8
        if w.sum() < 200:
            continue
        h, _ = np.histogram(r[w], bins)
        h = np.convolve(h, np.ones(5), 'same')
        off = 0.5 * (bins[np.argmax(h)] + bins[np.argmax(h) + 1])
        if abs(off) <= a.tol:
            continue
        nrem += 1
        if a.mode == 'remove':
            keep[sl] = False
            print(f'  block {b * K / 1e6:7.3f}M ground offset {off:+.2f} m -> removed')
            continue
        # ground plane of this block (points near its own ground mode), referred to the flat road at z_ref
        g = np.abs(r - off) < 0.15
        A = np.stack([np.ones(g.sum()), q[g, 0] - q[g, 0].mean(), q[g, 1] - q[g, 1].mean()], 1)
        coef = np.linalg.lstsq(A, r[g], rcond=None)[0] if a.tilt else np.array([np.median(r[g]), 0.0, 0.0])
        blk = xyz[sl]
        zc[sl] = blk[:, 2] - (coef[0] + coef[1] * (blk[:, 0] - q[g, 0].mean()) + coef[2] * (blk[:, 1] - q[g, 1].mean()))
        print(f'  block {b * K / 1e6:7.3f}M ground offset {off:+.2f} m, tilt {np.degrees(np.arctan(np.hypot(coef[1], coef[2]))):.2f} deg -> corrected')
    print(f'{a.mode}: {nrem} blocks')
    out = np.asarray(P).copy()
    out[:, 2] = zc.astype(out.dtype)
    np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), out[keep])
    np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I)[keep])


if __name__ == '__main__':
    main()
