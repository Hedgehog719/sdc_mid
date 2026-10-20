#!/usr/bin/env python3
"""What the lane matcher sees: per frame, the three camera images (fl | f | fr) with
  magenta = paint response used by the image-space matcher (brighter than the local road median by > 15, times the
            road probability, ego-vehicle / hood masks applied - same as tools/calib_img2.response)
  green   = LiDAR marking-map points (road level) projected at the trajectory pose (cam_fix_vert calibration).
Magenta without green nearby = image paint the map does not have (or a false detection, e.g. bushes); green without
magenta = map markings the image does not show. Viewer frames -> work/viewer/<Track>_paint.

usage: render_paint.py <track_dir> <traj.csv> <marks_npz> --frames 0:-1:9 [--name paint]
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
from bev import ImageBEV  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj')
ap.add_argument('marks')
ap.add_argument('--frames', default='0:-1:9')
ap.add_argument('--name', default='paint')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--width', type=int, default=640, help='per-camera width in the mosaic')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TR = pd.read_csv(a.traj)
MK = MarkingMap(a.marks)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
RD = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in ('fl', 'f', 'fr')}


def response(cam, j, img):
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[cam][j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)
    bad = B.ego_bad(cam, img.shape).copy()
    if cam in B.masks:
        m = np.unpackbits(B.masks[cam][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    r[cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0] = 0
    return r, rp, bad


lo, hi, st = (int(v) for v in a.frames.split(':'))
frames = list(range(lo, len(TR) if hi < 0 else hi, st))
out = os.path.join(os.path.dirname(__file__), '..', 'work', 'viewer', f'{T}_{a.name}')
os.makedirs(out, exist_ok=True)
for j in frames:
    x, y, yaw = TR.x[j], TR.y[j], TR.yaw[j]
    P = MK.points(x, y, 40)
    q = vehicle_frame(P, (x, y, yaw), MK.height) if len(P) else np.zeros((0, 3))
    q = q[road_level(q) & (q[:, 0] > -8)] if len(q) else q
    tiles = []
    for cam in ('fl', 'f', 'fr'):
        img = cv2.imread(os.path.join(a.track_dir, f'{T}_images', f'{T}_{cam}_images', B.names[cam][j]))
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        r, rp, bad = response(cam, j, g)
        vis = img.copy()
        vis[bad > 0] = (vis[bad > 0] * 0.35).astype(np.uint8)               # masked ego parts dimmed
        vis[rp < 0.4] = (vis[rp < 0.4] * 0.6).astype(np.uint8)              # non-road dimmed
        hot = r > 0.3
        vis[hot] = (0.35 * vis[hot] + 0.65 * np.array([255, 0, 255])).astype(np.uint8)
        if len(q):
            uv, ok = C.project(cam, q @ Rm)
            for u, v in uv[ok].astype(int):
                if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    cv2.circle(vis, (u, v), 2, (0, 255, 0), -1)
        s = a.width / img.shape[1]
        vis = cv2.resize(vis, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        cv2.putText(vis, cam, (10, 30), 0, 1.0, (255, 255, 255), 2)
        tiles.append(vis)
    mos = np.hstack(tiles)
    cv2.rectangle(mos, (0, mos.shape[0] - 28), (mos.shape[1], mos.shape[0]), (0, 0, 0), -1)
    cv2.putText(mos, f'{T} frame {j} | magenta: paint the matcher uses (road prob x local contrast, ego masked) | green: LiDAR marking map at {os.path.basename(a.traj)}',
                (8, mos.shape[0] - 9), 0, 0.5, (255, 255, 255), 1)
    cv2.imwrite(os.path.join(out, f'kf_{j:05d}.jpg'), mos, [cv2.IMWRITE_JPEG_QUALITY, 85])
    cv2.imwrite(os.path.join(out, f'bev_{j:05d}.jpg'), mos[:10])
json.dump({'track': T, 'frames': frames, 'label': f'{T}: matcher paint (magenta) vs LiDAR marking map (green), pose {os.path.basename(a.traj)}'},
          open(os.path.join(out, 'frames.json'), 'w'))
print('wrote', out, len(frames))
