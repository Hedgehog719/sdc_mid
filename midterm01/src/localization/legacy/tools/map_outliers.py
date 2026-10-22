#!/usr/bin/env python3
"""Drop isolated map points (radius-outlier removal on a voxel grid).

A point is kept when its voxel and the 26 neighbouring voxels (voxel size v, i.e. a ~3v cube) hold at least
min_pts points. Single stray returns (multipath, dust, glass reflections, moving objects seen once) otherwise
punch holes in the slope-based road surface and create fake kerbs / obstacles.

Reads map_<tag_in>.npy, writes map_<tag_out>.npy and intensities (same order).
"""
import argparse
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('--tag_in', default='clean')
    ap.add_argument('--tag_out', default='clean_sor')
    ap.add_argument('--voxel', type=float, default=0.2)
    ap.add_argument('--min_pts', type=int, default=4, help='points required in the 3x3x3 voxel neighbourhood')
    a = ap.parse_args()
    sfx = f'_{a.tag_in}' if a.tag_in else ''
    P = np.load(os.path.join(a.track_dir, f'map{sfx}.npy'), mmap_mode='r')
    I = np.load(os.path.join(a.track_dir, f'map_intensity{sfx}.npy'), mmap_mode='r')
    q = np.floor(np.asarray(P, dtype=np.float64) / a.voxel).astype(np.int64)
    q -= q.min(0)
    span = q.max(0) + 3
    q += 1
    key = (q[:, 0] * span[1] + q[:, 1]) * span[2] + q[:, 2]
    del q
    u, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    print(f'{len(P)} points in {len(u)} voxels')
    nb = np.zeros(len(u), np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                k2 = u + (dx * span[1] + dy) * span[2] + dz
                pos = np.clip(np.searchsorted(u, k2), 0, len(u) - 1)
                hit = u[pos] == k2
                nb[hit] += cnt[pos[hit]]
    keep = nb[inv] >= a.min_pts
    print(f'removed {(~keep).sum()} isolated points ({(~keep).mean() * 100:.2f} %)')
    np.save(os.path.join(a.track_dir, f'map_{a.tag_out}.npy'), np.asarray(P)[keep])
    np.save(os.path.join(a.track_dir, f'map_intensity_{a.tag_out}.npy'), np.asarray(I)[keep])


if __name__ == '__main__':
    main()
