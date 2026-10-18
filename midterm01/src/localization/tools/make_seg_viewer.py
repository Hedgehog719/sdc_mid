#!/usr/bin/env python3
"""Keyframes for judging the camera-side road / kerb segmentation.

Camera strip (front-left | front | front-right): road boundary contours of Mask2Former (blue), SegFormer-B2 (green)
and the per-frame Lab colour model (magenta), plus the map's slope-based road boundary projected with the
trajectory (yellow dots). BEV strip: B2 | M2F | colour | map smooth road, each with the map boundary in yellow.
Output: work/viewer/<Track>_seg/{kf,bev}_<i>.jpg + frames.json (served at /viewer/<Track>_seg).
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
from bev import H, W, ImageBEV, RoadProbBEV, bev_grid, map_road_smooth  # noqa: E402
from camera_model import Cameras  # noqa: E402
from common import TiledMap  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv')
    ap.add_argument('--step', type=int, default=30)
    a = ap.parse_args()
    track = os.path.basename(os.path.normpath(a.track_dir))
    out = os.path.join(os.path.dirname(__file__), '..', 'work', 'viewer', f'{track}_seg')
    os.makedirs(out, exist_ok=True)
    tr = pd.read_csv(a.traj_csv)
    M = TiledMap(os.path.join(a.track_dir, 'map.npy'))
    B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
    Rb, Rm, Rc = RoadProbBEV(B, 'road'), RoadProbBEV(B, 'road_m2f'), RoadProbBEV(B, 'road_m2f', 'seg_colour')
    segs = {'b2': np.load(os.path.join(a.track_dir, 'road', 'f.npz'))['road'].shape}
    seg_b2 = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in B.cams}
    seg_m2 = {c: np.load(os.path.join(a.track_dir, 'road_m2f', f'{c}.npz'))['road'] for c in B.cams}
    C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
    Rmnt = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
    xs, ys = bev_grid()
    frames = []
    for i in range(0, len(tr), a.step):
        x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
        rd, kn = map_road_smooth(M.query(x, y, 42).astype(np.float64), np.array([x, y]), yaw)
        rm = (rd > 0.5).astype(np.uint8)
        edge = (rm - cv2.erode(rm, np.ones((3, 3), np.uint8))) > 0
        P = np.stack([xs[edge], ys[edge], np.full(edge.sum(), -1.5)], 1)
        strip = []
        for c in ['fl', 'f', 'fr']:
            img = cv2.imread(os.path.join(B.track_dir, f'{track}_images', f'{track}_{c}_images', B.names[c][i]))
            hh, ww = img.shape[:2]
            for prob, col in [(cv2.resize(seg_b2[c][i], (ww, hh)) / 255.0, (0, 200, 0)),
                              (cv2.resize(seg_m2[c][i], (ww, hh)) / 255.0, (255, 80, 0)),
                              (Rc.colour_prob(i, c, None), (255, 0, 255))]:
                cs, _ = cv2.findContours((prob > 0.5).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cs = [q for q in cs if cv2.contourArea(q) > 2000]
                cv2.drawContours(img, cs, -1, col, 3)
            if len(P):
                uv, ok = C.project(c, P[::max(1, len(P) // 3000)] @ Rmnt)
                for u_, v_ in uv[ok].astype(int):
                    cv2.circle(img, (u_, v_), 3, (0, 230, 255), -1)
            strip.append(cv2.resize(img, (720, 464)))
        cv2.imwrite(os.path.join(out, f'kf_{i:05d}.jpg'), np.concatenate(strip, 1), [cv2.IMWRITE_JPEG_QUALITY, 80])
        panels = []
        for R in (Rb, Rm, Rc):
            p, v = R.render(i)
            g = cv2.cvtColor((np.where(v, p, 0) * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            g[edge] = (0, 230, 255)
            panels.append(g)
        mg = cv2.cvtColor((np.where(kn, rd, 0.3) * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        mg[edge] = (0, 230, 255)
        panels.append(mg)
        for k, t in enumerate(['B2', 'M2F', 'M2F+colour', 'map slope road']):
            cv2.putText(panels[k], t, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imwrite(os.path.join(out, f'bev_{i:05d}.jpg'), cv2.resize(np.concatenate(panels, 1), (W * 4 * 2 // 2 * 1, H)),
                    [cv2.IMWRITE_JPEG_QUALITY, 85])
        frames.append(int(i))
    json.dump({'track': track, 'frames': frames, 'label': 'camera road segmentation vs map slope road'},
              open(os.path.join(out, 'frames.json'), 'w'))
    print('wrote', len(frames), 'to', out)


if __name__ == '__main__':
    main()
