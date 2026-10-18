#!/usr/bin/env python3
"""Keyframes for judging the camera-side road / kerb segmentation (colour, no greyscale).

Camera strip (front-left | front | front-right): segmentation road boundary (blue, road class only), road users
(red outline: cars, scooters, people), map slope-road boundary projected with the trajectory (yellow dots).
BEV: colour inverse-perspective mosaic of the three cameras with the image road boundary (blue) and the map road
boundary (yellow); road users dimmed.
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


def colour_bev(B, i):
    acc = np.zeros((H, W, 3), np.float32)
    cnt = np.zeros((H, W), np.float32)
    for c in B.cams:
        img = cv2.imread(os.path.join(B.track_dir, f'{B.track}_images', f'{B.track}_{c}_images', B.names[c][i]))
        bad = np.zeros(img.shape[:2], np.uint8)
        bad |= B.ego_bad(c, bad.shape)
        mx, my, v = B.maps[c]
        g = cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(np.float32)
        bm = cv2.remap(bad, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
        ok = v & (bm == 0)
        acc[ok] += g[ok]
        cnt[ok] += 1
    out = np.where(cnt[..., None] > 0, acc / np.maximum(cnt, 1)[..., None], 0)
    return out.astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv')
    ap.add_argument('--seg', default='road_m2f_ro')
    ap.add_argument('--step', type=int, default=30)
    a = ap.parse_args()
    track = os.path.basename(os.path.normpath(a.track_dir))
    out = os.path.join(os.path.dirname(__file__), '..', 'work', 'viewer', f'{track}_seg')
    os.makedirs(out, exist_ok=True)
    tr = pd.read_csv(a.traj_csv)
    M = TiledMap(os.path.join(a.track_dir, 'map.npy'))
    B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
    R = RoadProbBEV(B, a.seg)
    R.use_masks = False  # road users come from the segmentation's own classes
    seg = {c: np.load(os.path.join(a.track_dir, a.seg, f'{c}.npz')) for c in B.cams}
    road = {c: seg[c]['road'] for c in B.cams}
    obj = {c: seg[c]['obj'] for c in B.cams}
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
            for prob, col, thick in [(cv2.resize(road[c][i], (ww, hh)) / 255.0, (255, 120, 0), 3),
                                     (cv2.resize(obj[c][i], (ww, hh)) / 255.0, (0, 0, 255), 2)]:
                cs, _ = cv2.findContours((prob > 0.5).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, [q for q in cs if cv2.contourArea(q) > 1500], -1, col, thick)
            if len(P):
                uv, ok = C.project(c, P[::max(1, len(P) // 3000)] @ Rmnt)
                for u_, v_ in uv[ok].astype(int):
                    cv2.circle(img, (u_, v_), 3, (0, 230, 255), -1)
            strip.append(cv2.resize(img, (720, 464)))
        cv2.imwrite(os.path.join(out, f'kf_{i:05d}.jpg'), np.concatenate(strip, 1), [cv2.IMWRITE_JPEG_QUALITY, 80])
        cb = colour_bev(B, i)
        p, v = R.render(i)
        ib = ((p > 0.5) & v).astype(np.uint8)
        cs, _ = cv2.findContours(ib, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big = cv2.resize(cb, (W * 2, H * 2), interpolation=cv2.INTER_LINEAR)
        cv2.drawContours(big, [q * 2 for q in cs if cv2.contourArea(q) > 100], -1, (255, 120, 0), 2)
        ey, ex = np.nonzero(edge)
        for yy, xx in zip(ey, ex):
            cv2.circle(big, (xx * 2, yy * 2), 1, (0, 230, 255), -1)
        cv2.putText(big, 'blue: camera road   yellow: map road (slope)', (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(os.path.join(out, f'bev_{i:05d}.jpg'), big, [cv2.IMWRITE_JPEG_QUALITY, 85])
        frames.append(int(i))
    json.dump({'track': track, 'frames': frames, 'label': f'camera road ({a.seg}, road class only) vs map slope road'},
              open(os.path.join(out, 'frames.json'), 'w'))
    print('wrote', len(frames), 'to', out)


if __name__ == '__main__':
    main()
