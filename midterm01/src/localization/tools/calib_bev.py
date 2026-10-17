#!/usr/bin/env python3
"""Calibrate the camera-rig mounting (height, pitch, roll, yaw) for the BEV map-to-image matcher.

With a reference trajectory (pseudo-GT), the map lane raster is rendered at the reference pose; for each
candidate calibration the image IPM lane raster is correlated with it (small x/y shift search, NCC).
The calibration maximising the mean peak NCC (with peaks near zero shift) is selected.
"""
import argparse
import itertools
import os
import sys

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from bev import H, RES, W, ImageBEV, lane_response, map_bev  # noqa: E402
from common import TiledMap  # noqa: E402


def ncc_shift(img, ref, max_shift_px):
    """Peak NCC of img against ref over integer shifts within +-max_shift_px; returns (peak, dr, dc)."""
    p = max_shift_px
    padded = cv2.copyMakeBorder(ref, p, p, p, p, cv2.BORDER_CONSTANT, value=0)
    tpl = img[p:-p, p:-p] if False else img
    res = cv2.matchTemplate(padded.astype(np.float32), tpl.astype(np.float32), cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(res)
    return mx, loc[1] - p, loc[0] - p, res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('ref_csv')
    ap.add_argument('--frames', default='200:3200:150')
    ap.add_argument('--heights', type=float, nargs='+', default=[1.8, 2.0, 2.2, 2.4])
    ap.add_argument('--pitches', type=float, nargs='+', default=[-3, -2, -1, 0, 1, 2, 3])
    ap.add_argument('--rolls', type=float, nargs='+', default=[-1.5, 0, 1.5])
    ap.add_argument('--yaws', type=float, nargs='+', default=[-1.5, 0, 1.5])
    a = ap.parse_args()
    lo, hi, st = [int(v) for v in a.frames.split(':')]
    frames = list(range(lo, hi, st))
    ref = pd.read_csv(a.ref_csv)
    M = TiledMap(os.path.join(a.track_dir, 'map.npy'))
    inten = np.load(os.path.join(a.track_dir, 'map_intensity.npy'))
    # TiledMap reorders points; keep intensity aligned by recomputing the same order
    keys = np.floor(np.load(os.path.join(a.track_dir, 'map.npy'), mmap_mode='r')[:, :2] / M.tile).astype(np.int64)
    k = (keys[:, 0] - M.kx0) * 100000 + (keys[:, 1] - M.ky0)
    inten = inten[np.argsort(k, kind='stable')]
    del keys, k
    maps = {}
    for i in frames:
        x, y, yaw = ref.x[i], ref.y[i], ref.yaw[i]
        idx = M.query_idx(x, y, 40.0)
        mg, road = map_bev(M.pts[idx], inten[idx], np.array([x, y]), yaw)
        maps[i] = (lane_response(mg, road), road)
    B = ImageBEV(a.track_dir)
    best = None
    for h, pt, rl, yw in itertools.product(a.heights, a.pitches, a.rolls, a.yaws):
        B.set_calib(h, pt, rl, yw)
        scores, shifts = [], []
        for i in frames:
            g, v = B.render(i)
            ref_resp, road = maps[i]
            resp = lane_response(g, v & road)
            pk, dr, dc, _ = ncc_shift(resp, ref_resp, int(1.5 / RES))
            scores.append(pk)
            shifts.append((dr * RES, dc * RES))
        sc = np.mean(scores)
        sh = np.sqrt(np.mean(np.square(shifts), 0))
        print(f'h={h:.2f} pitch={pt:+.1f} roll={rl:+.1f} yaw={yw:+.1f}  ncc={sc:.3f}  shift rms fwd {sh[0]:.2f} lat {sh[1]:.2f}', flush=True)
        if best is None or sc > best[0]:
            best = (sc, h, pt, rl, yw)
    print('BEST', best)


if __name__ == '__main__':
    main()
