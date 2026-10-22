#!/usr/bin/env python3
"""Remove the scans recorded while the mapping vehicle stood still from the LiDAR map.

The map files store the accumulated scans in recording order (each ~20k-point block is part of one sweep,
blocks move along the mapping drive). Both maps begin with the mapping vehicle stopped at the track start
for a long time (Track1: idx 9.6M-20.4M, 18 % of the map; Track2: idx 0-17.2M, 34 %). Those scans are
(1) dense copies of the vehicles queued in front, and (2) affected by the SLAM height / attitude drift while
stationary (Track1: ground of successive scans sinks by 0.7 m and tilts by several degrees; approach and
departure differ by 0.4 m). Stacked at the start they make a fake dip, fake kerbs and obstacles on a flat road.
The moving scans before / after cover the same area consistently.

Writes <track>/map_<tag>.npy and map_intensity_<tag>.npy (same order); tools pick them up with SDC_MAP_TAG=<tag>.
"""
import argparse
import os

import numpy as np
from scipy.ndimage import uniform_filter1d


def consistency_filter(P, keep, K, tol, iters, cell=1.0):
    """Drop blocks whose ground is vertically offset from the consensus ground of all kept blocks.

    Consensus per 1 m cell: median z of the cell's lowest layer (points within 0.8 m above its 5th percentile),
    i.e. the majority ground layer. Block offset: mode of its residuals within +-0.8 m (its own ground)."""
    xyz = np.asarray(P, dtype=np.float32)
    ix = np.floor(xyz[:, 0] / cell).astype(np.int64)
    iy = np.floor(xyz[:, 1] / cell).astype(np.int64)
    key = (ix - ix.min()) * (iy.max() - iy.min() + 1) + (iy - iy.min())
    z = xyz[:, 2]
    nb = (len(P) + K - 1) // K
    bins = np.arange(-0.8, 0.8001, 0.02)
    for it in range(iters):
        sel = np.flatnonzero(keep)
        o = sel[np.argsort(key[sel], kind='stable')]
        ks = key[o]
        u, st = np.unique(ks, return_index=True)
        en = np.r_[st[1:], len(ks)]
        zs = z[o]
        ref_u = np.full(len(u), np.nan, np.float32)
        for n, (a0, b0) in enumerate(zip(st, en)):
            if b0 - a0 < 5:
                continue
            zz = zs[a0:b0]
            lo = np.percentile(zz, 5)
            ref_u[n] = np.median(zz[zz < lo + 0.8])
        pos = np.searchsorted(u, key)
        pos = np.clip(pos, 0, len(u) - 1)
        ref = np.where(u[pos] == key, ref_u[pos], np.nan)
        removed = 0
        for b in range(nb):
            a0, b0 = b * K, min((b + 1) * K, len(P))
            if not keep[a0:b0].any():
                continue
            r = z[a0:b0] - ref[a0:b0]
            r = r[np.isfinite(r) & (np.abs(r) < 0.8)]
            if len(r) < 200:
                continue
            h, _ = np.histogram(r, bins)
            h = np.convolve(h, np.ones(5), 'same')
            off = 0.5 * (bins[np.argmax(h)] + bins[np.argmax(h) + 1])
            if abs(off) > tol:
                keep[a0:b0] = False
                removed += 1
        print(f'consistency pass {it + 1}: removed {removed} blocks ({removed * K / len(P) * 100:.1f} %)')
        if removed == 0:
            break
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('--tag', default='clean')
    ap.add_argument('--block', type=int, default=20000, help='points per block (part of one sweep)')
    ap.add_argument('--smooth', type=int, default=15, help='blocks averaged for the scan centre (one full sweep+)')
    ap.add_argument('--win', type=int, default=40, help='blocks in the stationarity window')
    ap.add_argument('--move', type=float, default=2.0, help='max centre motion (m) within the window = stationary')
    ap.add_argument('--min_len', type=float, default=1.0e6, help='min points of a stationary period to remove')
    ap.add_argument('--margin', type=float, default=0.4e6, help='points removed before / after each period')
    ap.add_argument('--consistency', type=float, default=0.12,
                    help='>0: also drop blocks whose ground is offset from the consensus ground by more (m)')
    ap.add_argument('--iters', type=int, default=2)
    a = ap.parse_args()
    P = np.load(os.path.join(a.track_dir, 'map.npy'), mmap_mode='r')
    I = np.load(os.path.join(a.track_dir, 'map_intensity.npy'), mmap_mode='r')
    K = a.block
    nb = (len(P) + K - 1) // K
    cen = np.array([np.median(np.asarray(P[k * K:(k + 1) * K, :2], dtype=np.float64), 0) for k in range(nb)])
    cs = uniform_filter1d(cen, a.smooth, axis=0, mode='nearest')
    h = a.win // 2
    mv = np.array([np.hypot(*np.ptp(cs[max(0, i - h):i + h + 1], 0)) for i in range(nb)])
    still = mv < a.move
    keep = np.ones(len(P), bool)
    i = 0
    while i < nb:
        if not still[i]:
            i += 1
            continue
        j = i
        while j < nb and still[j]:
            j += 1
        lo, hi = i * K, min(j * K, len(P))
        if hi - lo >= a.min_len:
            lo2, hi2 = max(0, int(lo - a.margin)), min(len(P), int(hi + a.margin))
            keep[lo2:hi2] = False
            c = cs[i:j].mean(0)
            print(f'stationary {lo / 1e6:6.2f}M-{hi / 1e6:6.2f}M at ({c[0]:.1f}, {c[1]:.1f}) -> removed '
                  f'{lo2 / 1e6:.2f}M-{hi2 / 1e6:.2f}M ({(hi2 - lo2) / len(P) * 100:.1f} %)')
        i = j
    if a.consistency > 0:
        keep = consistency_filter(P, keep, K, a.consistency, a.iters)
    out = os.path.join(a.track_dir, f'map_{a.tag}.npy')
    np.save(out, np.asarray(P)[keep])
    np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag}.npy'), np.asarray(I)[keep])
    print(f'kept {keep.sum()} / {len(P)} points -> {out}')


if __name__ == '__main__':
    main()
