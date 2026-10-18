#!/usr/bin/env python3
"""Precompute keyframe overlays for the web viewer: map features projected into the three cameras with a trajectory.

Red dots: map road markings (reflectivity); yellow: map road boundary (kerbs / islands, row scan); cyan: kerb steps;
green: curated poles. Plus a BEV panel: per-camera image marking response with the map markings (red) and road
boundary (yellow), and the segmentation road probability (blue tint).
Output: work/viewer/<Track>/kf_<i>.jpg, bev_<i>.jpg, frames.json
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import (H, RES, W, X_MAX, Y_MAX, ImageBEV, RoadProbBEV, bev_grid, lane_response, map_bev, map_curb,  # noqa: E402
                 map_road_scan)
from bev_match import load_map  # noqa: E402
from camera_model import Cameras  # noqa: E402


def to_vehicle(rc_mask, h):
    xs, ys = bev_grid()
    return np.stack([xs[rc_mask], ys[rc_mask], np.full(rc_mask.sum(), -h)], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv')
    ap.add_argument('--step', type=int, default=15)
    ap.add_argument('--label', default='')
    ap.add_argument('--variant', default='', help='render into work/viewer/<Track>_<variant> for A/B comparison')
    ap.add_argument('--ground_mode', default='plane', help='map road mask: plane | local | smooth | hybrid')
    ap.add_argument('--range', default='', help='lo:hi frames to (re)render; frames.json keeps all rendered frames')
    a = ap.parse_args()
    import bev as _bev
    _bev.GROUND_MODE = a.ground_mode
    track = os.path.basename(os.path.normpath(a.track_dir))
    out = os.path.join(os.path.dirname(__file__), '..', 'work', 'viewer', track + (f'_{a.variant}' if a.variant else ''))
    os.makedirs(out, exist_ok=True)
    tr = pd.read_csv(a.traj_csv)
    M = load_map(a.track_dir)
    B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
    R = RoadProbBEV(B)
    C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
    poles = np.load(os.path.join(a.track_dir, 'poles_curated.npy'))
    Rm = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
    cams = ['fl', 'f', 'fr']
    imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
    names = {c: sorted(os.listdir(imgdir[c])) for c in cams}
    frames = []
    lo, hi = (0, len(tr)) if not a.range else [int(v) for v in a.range.split(':')]
    for i in range(lo, min(hi, len(tr)), a.step):
        x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
        idx = M.query_idx(x, y, 42.0)
        pts, inten = M.pts[idx], M.inten[idx]
        mg, road = map_bev(pts, inten, np.array([x, y]), yaw)
        lanes = lane_response(mg, road) > 1.5
        rs = map_road_scan(pts.astype(np.float64), np.array([x, y]), yaw)
        edge = (rs > 0.5).astype(np.uint8)
        edge = (edge - cv2.erode(edge, np.ones((3, 3), np.uint8))) > 0
        curb = map_curb(pts.astype(np.float64), np.array([x, y]), yaw) > 0.03
        layers = [(to_vehicle(lanes, 1.5), (0, 0, 255), 1), (to_vehicle(curb, 1.5), (255, 255, 0), 1),
                  (to_vehicle(edge, 1.5), (0, 230, 255), 2)]
        c_, s_ = np.cos(yaw), np.sin(yaw)
        d = poles[:, :2] - [x, y]
        pu, pv = c_ * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c_ * d[:, 1]
        near = (np.hypot(pu, pv) < 40) & (pu > -5)
        strip = []
        for cam in cams:
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]))
            for P, col, rad in layers:
                if len(P) == 0:
                    continue
                P = P[::max(1, len(P) // 4000)]
                uv, ok = C.project(cam, P @ Rm)
                for u_, v_ in uv[ok].astype(int):
                    cv2.circle(img, (u_, v_), rad, col, -1)
            for k in np.flatnonzero(near):
                hs = np.linspace(0, min(poles[k, 4], 6), 12)
                Q = np.stack([np.full(12, pu[k]), np.full(12, pv[k]), hs - 1.5], 1) @ Rm
                uv, ok = C.project(cam, Q)
                for p0, p1, o0, o1 in zip(uv[:-1], uv[1:], ok[:-1], ok[1:]):
                    if o0 and o1:
                        cv2.line(img, tuple(p0.astype(int)), tuple(p1.astype(int)), (0, 255, 0), 3)
            strip.append(cv2.resize(img, (720, 464)))
        cv2.imwrite(os.path.join(out, f'kf_{i:05d}.jpg'), np.concatenate(strip, 1), [cv2.IMWRITE_JPEG_QUALITY, 80])
        # BEV panel
        resp, v = B.render_resp(i, lane_response)
        rp, rv = R.render(i)
        g = np.clip(resp / 2.0 * 255, 0, 255).astype(np.uint8)
        bev = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        bev[..., 0] = np.maximum(bev[..., 0], (np.where(rv, rp, 0) * 120).astype(np.uint8))
        bev[lanes] = (0, 0, 255)
        bev[edge] = (0, 230, 255)
        bev[curb & ~edge & ~lanes] = (255, 255, 0)
        bev = cv2.resize(bev, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(os.path.join(out, f'bev_{i:05d}.jpg'), bev, [cv2.IMWRITE_JPEG_QUALITY, 85])
        frames.append(int(i))
        if len(frames) % 20 == 0:
            print(track, len(frames), flush=True)
    fj = os.path.join(out, 'frames.json')
    if a.range and os.path.exists(fj):
        frames = sorted(set(frames) | set(json.load(open(fj))['frames']))
    json.dump({'track': track, 'frames': frames, 'label': a.label or os.path.basename(a.traj_csv)},
              open(os.path.join(out, 'frames.json'), 'w'))
    print('wrote', len(frames), 'keyframes to', out)


if __name__ == '__main__':
    main()
