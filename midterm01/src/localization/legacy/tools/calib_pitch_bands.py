#!/usr/bin/env python3
"""Front-camera pitch from BEV band consistency (trajectory-independent).

A pitch error stretches the IPM with distance: far markings land too far (or too near) while near ones barely move.
The forward offset that aligns the camera's lane response with the map is measured separately in a near and a far
band; their difference cancels the trajectory error (common to both bands) and is zero for a correct pitch. It is
measured for a grid of rig pitch offsets, so the zero crossing gives the static pitch, and its regression on the
longitudinal acceleration checks the dynamic (suspension) pitch model.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

sys.path.insert(0, os.path.dirname(__file__))
from bev import RES, ImageBEV, bev_grid, lane_response, map_bev  # noqa: E402
from bev_match import load_map  # noqa: E402
from common import ply_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('--cam', default='f')
ap.add_argument('--frames', default='330:-1:10')
ap.add_argument('--pitch', default='-0.4,-0.2,0,0.2,0.4,0.6,0.8', help='rig pitch offsets (deg) to test')
ap.add_argument('--near', default='4:10')
ap.add_argument('--far', default='14:26')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
a = ap.parse_args()
ref = pd.read_csv(a.traj_csv)
ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
tt = (ts - ts[0]) * 1e-9
spd = gaussian_filter1d(np.hypot(np.gradient(ref.x, tt), np.gradient(ref.y, tt)), 14)
acc = gaussian_filter1d(np.gradient(spd, tt), 14)
lo, hi, st = (int(v) for v in a.frames.split(':'))
frames = [i for i in range(lo, len(ref) if hi < 0 else hi, st) if spd[i] > 3]
pitches = [float(v) for v in a.pitch.split(',')]
bands = {'near': tuple(float(v) for v in a.near.split(':')), 'far': tuple(float(v) for v in a.far.split(':'))}
M = load_map(a.track_dir)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount, cams=(a.cam,))
xs, _ = bev_grid()
P = 15
refs, imgs = {}, {}
for i in frames:
    x, y, yaw = ref.x[i], ref.y[i], ref.yaw[i]
    idx = M.query_idx(x, y, 42.0)
    mg, road = map_bev(M.pts[idx], M.inten[idx], np.array([x, y]), yaw)
    lr = cv2.GaussianBlur(lane_response(mg, road), (0, 0), 1.5)
    refs[i] = (cv2.copyMakeBorder(lr, P, P, P, P, cv2.BORDER_CONSTANT, value=0).astype(np.float32),
               cv2.dilate(road.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0)


def subpix(c, j, k):
    if 0 < j < c.shape[0] - 1:
        a0, a1, a2 = c[j - 1, k], c[j, k], c[j + 1, k]
        den = a0 - 2 * a1 + a2
        return j + (0.5 * (a0 - a2) / den if den < 0 else 0.0)
    return float(j)


out = {}
for pt in pitches:
    B.set_calib(1.5, pt, 0.0, a.yaw_mount)
    rows = []
    for i in frames:
        refp, roadd = refs[i]
        resp, ok = B.render_resp(i, lane_response)
        resp = cv2.GaussianBlur(resp, (0, 0), 1.5)
        okr = (cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0) & roadd
        dx = {}
        for name, (b0, b1) in bands.items():
            m = okr & (xs >= b0) & (xs < b1)
            if m.sum() < 300:
                break
            rr, cc_ = np.nonzero(m)
            r0, r1, c0, c1 = rr.min(), rr.max() + 1, cc_.min(), cc_.max() + 1
            tpl = np.where(m, resp, 0)[r0:r1, c0:c1].astype(np.float32)
            win = refp[r0:r1 + 2 * P, c0:c1 + 2 * P]
            if tpl.std() < 1e-3:
                break
            c = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
            j, k = np.unravel_index(np.argmax(c), c.shape)
            if j in (0, 2 * P) or k in (0, 2 * P):
                break
            dx[name] = (-(subpix(c, j, k) - P) * RES, float(c[j, k]))
        if len(dx) == 2:
            rows.append((i, dx['near'][0], dx['far'][0], acc[i], dx['near'][1], dx['far'][1]))
    out[pt] = {int(r[0]): r for r in rows}
    print(f'pitch {pt:+.2f} done ({len(rows)} frames)', flush=True)
common = sorted(set.intersection(*[set(v) for v in out.values()]))
ok = [i for i in common if min(out[p][i][4] for p in pitches) > 0.15 and min(out[p][i][5] for p in pitches) > 0.15]
print(f'{len(common)} frames measured at every pitch, {len(ok)} with both bands matched (NCC > 0.15) at every pitch')
for pt in pitches:
    r = np.array([out[pt][i] for i in ok])
    d = r[:, 2] - r[:, 1]
    k = np.abs(d - np.median(d)) < 1.0
    sl = np.polyfit(r[k, 3], d[k], 1)[0]
    print(f'pitch {pt:+.2f}: near dx {np.median(r[:, 1]):+.2f} far dx {np.median(r[:, 2]):+.2f} far-near {np.median(d):+.3f} m '
          f'| NCC near {r[:, 4].mean():.4f} far {r[:, 5].mean():.4f} | far-near vs accel {sl:+.3f} m per m/s2')
