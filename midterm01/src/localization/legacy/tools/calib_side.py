#!/usr/bin/env python3
"""Per-camera rotation calibration (yaw, pitch, roll) against the LiDAR map, one camera at a time.

On a track with an accurate reference trajectory, each camera's own BEV marking response (lane_response inside
that camera's footprint, extended BEV incl. the area beside the car) is correlated at zero shift with the map
marking raster; a coordinate descent over (yaw, pitch, roll) offsets maximises the mean NCC over many frames.
Result: work/cam_rot_fix.json, read by camera_model.Cameras (applied in every projection).
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from bev import H, W, ImageBEV, lane_response, map_bev  # noqa: E402
from bev_match import load_map  # noqa: E402
from common import ply_frames  # noqa: E402


BLUR = 3.0  # px (0.3 m)
SHIFT = 10  # px (1 m) search for the peak NCC


def cam_maps(B, cam):
    from scipy.spatial.transform import Rotation  # noqa: F401
    h, p, r, y = B.calib
    B.set_calib(h, p, r, y)
    return B.maps[cam]


def score(B, cam, frames, cache):
    mx, my, v = cam_maps(B, cam)
    vals = []
    for i in frames:
        img, bad, ref, road = cache[i][cam], cache[i][cam + '_bad'], cache[i]['ref'], cache[i]['road']
        g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
        b = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
        ok = v & (b == 0)
        if ok.sum() < 500:
            continue
        resp = lane_response(g, ok)
        m = (cv2.erode(ok.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0) & road
        if m.sum() < 300 or resp[m].std() < 1e-3:
            continue
        # peak NCC within +-1 m: a wrong rotation distorts the projected markings so no shift fits them well
        resp = np.where(m, resp, 0).astype(np.float32)
        rows, cols = np.nonzero(m)
        r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        tpl = resp[r0:r1, c0:c1]
        p = SHIFT
        refp = cv2.copyMakeBorder(ref, p, p, p, p, cv2.BORDER_CONSTANT, value=0).astype(np.float32)
        win = refp[r0:r1 + 2 * p, c0:c1 + 2 * p]
        if tpl.std() < 1e-4:
            continue
        res = cv2.matchTemplate(win, tpl, cv2.TM_CCOEFF_NORMED)
        vals.append(float(res.max()))
    return float(np.mean(vals)) if vals else -1.0, len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('ref_csv')
    ap.add_argument('--step', type=int, default=60)
    ap.add_argument('--cams', nargs='+', default=['fl', 'fr', 'f'])
    ap.add_argument('--eval', default='', help='only score: json of per-camera rot fixes (or "none")')
    ap.add_argument('--out', default='', help='output json (default work/cam_rot_fix.json, the active calibration)')
    a = ap.parse_args()
    ref = pd.read_csv(a.ref_csv)
    ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
    tt = (ts - ts[0]) * 1e-9
    spd = np.hypot(np.gradient(ref.x, tt), np.gradient(ref.y, tt))
    frames = [i for i in range(60, len(ref), a.step) if spd[i] > 3]
    M = load_map(a.track_dir)
    B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
    C = B.C
    C.rot_fix = {}  # start from the current defaults (yaw_fix) explicitly
    start = {c: (C.yaw_fix.get(c, 0.0), 0.0, 0.0) for c in a.cams}
    cache = {}
    for i in frames:
        x, y, yaw = ref.x[i], ref.y[i], ref.yaw[i]
        idx = M.query_idx(x, y, 42.0)
        mg, road = map_bev(M.pts[idx], M.inten[idx], np.array([x, y]), yaw)
        lr = lane_response(mg, road)
        d = {'ref': lr, 'ref_blur': cv2.GaussianBlur(lr, (0, 0), BLUR),
             'road': cv2.dilate(road.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0}
        for c in a.cams:
            img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][i]), cv2.IMREAD_GRAYSCALE)
            bad = B.ego_bad(c, img.shape).copy()
            if c in B.masks:
                m = np.unpackbits(B.masks[c][i], axis=1)[:, :img.shape[1] // 2]
                bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
            d[c], d[c + '_bad'] = img, bad
        cache[i] = d
    print(f'{len(frames)} frames', flush=True)
    if a.eval:
        fixes = {} if a.eval == 'none' else json.load(open(a.eval))
        for c in a.cams:
            C.rot_fix[c] = tuple(fixes.get(c, start[c]))
            sc, n = score(B, c, frames, cache)
            print(f'   {c}: rot {C.rot_fix[c]} -> peak NCC {sc:.4f} (n={n})')
        return
    result = {}
    for c in a.cams:
        cur = list(start[c])
        C.rot_fix[c] = tuple(cur)
        best, n = score(B, c, frames, cache)
        print(f'{c}: start yaw/pitch/roll {cur} -> NCC {best:.4f} (n={n})', flush=True)
        for step in [1.0, 0.5, 0.25]:
            improved = True
            while improved:
                improved = False
                for k in range(3):
                    for dlt in (-step, step):
                        cand = list(cur)
                        cand[k] += dlt
                        C.rot_fix[c] = tuple(cand)
                        sc, _ = score(B, c, frames, cache)
                        if sc > best + 1e-4:
                            best, cur, improved = sc, cand, True
                C.rot_fix[c] = tuple(cur)
            print(f'   step {step}: yaw {cur[0]:+.2f} pitch {cur[1]:+.2f} roll {cur[2]:+.2f} -> NCC {best:.4f}', flush=True)
        result[c] = [round(v, 3) for v in cur]
        C.rot_fix[c] = tuple(cur)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'work', 'cam_rot_fix.json')
    json.dump(result, open(out, 'w'), indent=1)
    print('saved', out, result)


if __name__ == '__main__':
    main()
