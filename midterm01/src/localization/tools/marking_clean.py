#!/usr/bin/env python3
"""Shape filter for a marking map (marking_map.py npz), map-only (no trajectory): keeps connected marking components
that are elongated (lines, dashes, crosswalk bars: major axis >= MIN_LEN m and length / width >= MIN_ELONG) or large
(arrows, text: area >= MIN_AREA m^2), drops small compact blobs (paving patterns on raised pavements beside the
road pass the ground-layer test at the Track01 / Track02 starts and look like speckle in the matcher's map).
usage: marking_clean.py <in.npz> <out.npz> [MIN_LEN MIN_ELONG MIN_AREA]"""
import sys

import cv2
import numpy as np

RES_M = 0.1


def clean(resp, min_len, min_elong, min_area):
    m = (resp > 0).astype(np.uint8)
    nlab, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    keep = np.zeros(nlab, bool)
    for i in range(1, nlab):
        area = st[i, cv2.CC_STAT_AREA]
        if area * RES_M ** 2 >= min_area:
            keep[i] = True
            continue
        if area < 3:
            continue
        ys, xs = np.nonzero(lab[st[i, cv2.CC_STAT_TOP]:st[i, cv2.CC_STAT_TOP] + st[i, cv2.CC_STAT_HEIGHT],
                                st[i, cv2.CC_STAT_LEFT]:st[i, cv2.CC_STAT_LEFT] + st[i, cv2.CC_STAT_WIDTH]] == i)
        c = np.cov(np.stack([xs, ys]).astype(np.float64)) + np.eye(2) / 12.0
        ev = np.sort(np.linalg.eigvalsh(c))[::-1]
        length, width = np.sqrt(12 * ev[0]) * RES_M, np.sqrt(12 * ev[1]) * RES_M
        keep[i] = length >= min_len and length / max(width, 1e-3) >= min_elong
    return np.where(keep[lab], resp, 0).astype(resp.dtype)


if __name__ == '__main__':
    src, dst = sys.argv[1], sys.argv[2]
    min_len, min_elong, min_area = (float(v) for v in sys.argv[3:6]) if len(sys.argv) > 5 else (1.0, 3.0, 0.5)
    z = dict(np.load(src))
    before = sum(int((r > 0).sum()) for r in z['resp'])
    z['resp'] = np.array([clean(r, min_len, min_elong, min_area) for r in z['resp']])
    after = sum(int((r > 0).sum()) for r in z['resp'])
    np.savez_compressed(dst, **z)
    print(f'{dst}: marking cells {before} -> {after} ({100 * after / max(before, 1):.0f} %)')
