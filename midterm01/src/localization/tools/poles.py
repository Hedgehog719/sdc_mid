#!/usr/bin/env python3
"""Extract pole-like landmarks (street lights, sign / traffic-light poles, utility poles, tree trunks) from the map.

Height above ground (ground raster) is binned in 0.5 m slices per 0.2 m XY cell. A pole cell is occupied in most
slices between 1 and 4 m, and is isolated: the ring 0.6-1.5 m around it is mostly empty between 1 and 3 m
(rejects walls, bushes, parked vehicles). Pole cells are clustered into landmarks (x, y, z_base, z_top, height).
"""
import argparse
import os
import sys

import cv2
import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(__file__))
from bev import GroundRaster  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('--res', type=float, default=0.2)
    ap.add_argument('--min_slices', type=int, default=5, help='of the 6 slices between 1 and 4 m')
    ap.add_argument('--ring_max', type=float, default=0.12, help='max occupied fraction of the isolation ring')
    a = ap.parse_args()
    xyz = np.load(os.path.join(a.track_dir, 'map.npy'), mmap_mode='r')
    G = GroundRaster(xyz)
    res = a.res
    x0, y0 = G.x0, G.y0
    nx = int(G.nx * G.res / res) + 1
    ny = int(G.ny * G.res / res) + 1
    bits = np.zeros(nx * ny, np.uint16)
    top = np.zeros(nx * ny, np.float32)
    step = 5_000_000
    for s in range(0, len(xyz), step):
        p = np.asarray(xyz[s:s + step], np.float64)
        hag = p[:, 2] - G.height(p[:, 0], p[:, 1])
        m = (hag > 0.5) & (hag < 8.0)
        p, hag = p[m], hag[m]
        k = ((p[:, 0] - x0) / res).astype(np.int64) * ny + ((p[:, 1] - y0) / res).astype(np.int64)
        sl = np.clip(((hag - 0.5) / 0.5).astype(np.int64), 0, 14)
        np.bitwise_or.at(bits, k, (1 << sl).astype(np.uint16))
        np.maximum.at(top, k, hag.astype(np.float32))
    bits = bits.reshape(nx, ny)
    top = top.reshape(nx, ny)
    # slices: index i covers hag 0.5+0.5i .. 1.0+0.5i ; 1-4 m -> i = 1..6 ; 1-3 m -> i = 1..4
    occ = [((bits >> i) & 1).astype(np.uint8) for i in range(15)]
    n14 = sum(occ[i] for i in range(1, 7))
    reaches_ground = (occ[0] | occ[1]).astype(bool)  # 0.5-1.5 m occupied: excludes hanging signboards / awnings
    cand = (n14 >= a.min_slices) & reaches_ground
    # isolation ring on the union of the 1-3 m slices
    mid = np.clip(sum(occ[i] for i in range(1, 5)), 0, 1).astype(np.float32)
    r_in, r_out = int(round(0.6 / res)), int(round(1.5 / res))
    yy, xx = np.mgrid[-r_out:r_out + 1, -r_out:r_out + 1]
    rr = np.hypot(xx, yy)
    ring = ((rr >= r_in) & (rr <= r_out)).astype(np.float32)
    ring /= ring.sum()
    ring_occ = cv2.filter2D(mid, -1, ring, borderType=cv2.BORDER_CONSTANT)
    pole = cand & (ring_occ < a.ring_max)
    lab, n = ndimage.label(pole, structure=np.ones((3, 3)))
    idx = np.arange(1, n + 1)
    size = ndimage.sum(pole, lab, idx)
    cx = ndimage.mean(np.arange(nx)[:, None] * np.ones((1, ny)), lab, idx) if False else None
    coms = np.array(ndimage.center_of_mass(pole, lab, idx))
    tops = ndimage.maximum(top, lab, idx)
    keep = (size <= 9)  # footprint <= ~0.36 m^2 (thin)
    px = x0 + (coms[keep, 0] + 0.5) * res
    py = y0 + (coms[keep, 1] + 0.5) * res
    zb = G.height(px, py)
    out = np.stack([px, py, zb, zb + tops[keep], tops[keep]], 1)
    np.save(os.path.join(a.track_dir, 'poles.npy'), out.astype(np.float64))
    print(f'pole candidates {n}, kept {len(out)}; height median {np.median(out[:, 4]):.1f} m '
          f'(p10 {np.percentile(out[:, 4], 10):.1f}, p90 {np.percentile(out[:, 4], 90):.1f})')


if __name__ == '__main__':
    main()
