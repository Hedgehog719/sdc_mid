#!/usr/bin/env python3
"""Per camera and per distance band: the BEV shift that best aligns the camera's lane-marking response with the map.

A correct calibration gives the same shift (the trajectory error) for every camera and band. A shift that changes
with the band, or differs between cameras that see the same marking, is a calibration-geometry error (camera
position / height / roll) that a single rotation cannot absorb - e.g. a marking that joins far ahead but splits
near the car. Bands are BEV forward distance (m, from the front camera); output dx (forward) / dy (left) in m =
how far the camera's BEV must move to match the map (no capture-time sync: read dy, dx is speed-dependent).
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from bev import RES, ImageBEV, bev_grid, lane_response, map_bev  # noqa: E402
from bev_match import load_map  # noqa: E402
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--frames', default='330:-1:30')
ap.add_argument('--show', type=int, nargs='*', default=[], help='also print these frames individually')
ap.add_argument('--bands', default='-5:2,2:6,6:10,10:16,16:24')
ap.add_argument('--dump', default='', help='npz with every per-frame measurement')
a = ap.parse_args()
ref = pd.read_csv(a.traj_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
tt = (ts - ts[0]) * 1e-9
spd = np.hypot(np.gradient(ref.x, tt), np.gradient(ref.y, tt))
lo, hi, st = (int(v) for v in a.frames.split(':'))
hi = len(ref) if hi < 0 else hi
frames = sorted(set([i for i in range(lo, hi, st) if spd[i] > 3] + a.show))
bands = [tuple(float(v) for v in b.split(':')) for b in a.bands.split(',')]
M = load_map(a.track_dir)
B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
xs, ys = bev_grid()
P = 15
rows_out = []
res = {c: {b: [] for b in bands} for c in B.cams}
for i in frames:
    x, y, yaw = ref.x[i], ref.y[i], ref.yaw[i]
    idx = M.query_idx(x, y, 42.0)
    mg, road = map_bev(M.pts[idx], M.inten[idx], np.array([x, y]), yaw)
    lr = cv2.GaussianBlur(lane_response(mg, road), (0, 0), 1.5)
    refp = cv2.copyMakeBorder(lr, P, P, P, P, cv2.BORDER_CONSTANT, value=0).astype(np.float32)
    roadd = cv2.dilate(road.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    line = [f'frame {i:5d}']
    for c in B.cams:
        img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][i]),
                         cv2.IMREAD_GRAYSCALE)
        bad = B.ego_bad(c, img.shape).copy()
        if c in B.masks:
            m_ = np.unpackbits(B.masks[c][i], axis=1)[:, :img.shape[1] // 2]
            bad |= cv2.resize(m_, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        mx, my, v = B.maps[c]
        g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
        b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
        ok = v & (b == 0)
        if ok.sum() < 500:
            continue
        resp = cv2.GaussianBlur(lane_response(g, ok), (0, 0), 1.5)
        okr = (cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0) & roadd
        for bd in bands:
            m = okr & (xs >= bd[0]) & (xs < bd[1])
            if m.sum() < 300:
                continue
            rows, cols = np.nonzero(m)
            r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
            tpl = np.where(m, resp, 0)[r0:r1, c0:c1].astype(np.float32)
            win = refp[r0:r1 + 2 * P, c0:c1 + 2 * P]
            if tpl.std() < 1e-3 or win[P:-P, P:-P].std() < 1e-3:
                continue
            cc = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
            dr, dc = np.unravel_index(np.argmax(cc), cc.shape)
            if cc[dr, dc] < 0.25 or dr in (0, 2 * P) or dc in (0, 2 * P):
                continue
            dx, dy = -(dr - P) * RES, -(dc - P) * RES
            res[c][bd].append((dx, dy, float(cc[dr, dc])))
            rows_out.append((i, B.cams.index(c), bands.index(bd), dx, dy, float(cc[dr, dc])))
            if i in a.show:
                line.append(f'{c}[{bd[0]:g},{bd[1]:g}] dx {dx:+.1f} dy {dy:+.1f} ncc {cc[dr, dc]:.2f}')
    if i in a.show:
        print('  '.join(line), flush=True)
print(f'{len(frames)} frames; median shift camera BEV -> map (dx forward, dy left, m), n, mean NCC')
for c in B.cams:
    print(' ', c, ' | '.join(f'x {bd[0]:g}..{bd[1]:g}: ' + (f'dy {np.median([r[1] for r in res[c][bd]]):+.2f} dx {np.median([r[0] for r in res[c][bd]]):+.2f} (n {len(res[c][bd])})' if len(res[c][bd]) >= 5 else '-') for bd in bands))
if a.dump:
    r = np.array(rows_out)
    np.savez(a.dump, frame=r[:, 0], cam=r[:, 1], band=r[:, 2], dx=r[:, 3], dy=r[:, 4], ncc=r[:, 5], cams=np.array(B.cams), bands=np.array(bands))
