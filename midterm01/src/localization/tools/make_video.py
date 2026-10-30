#!/usr/bin/env python3
"""Video of a localization run: top = BEV (car on the left, driving to the right) with the camera marking response (white), the map
markings at the estimated pose (red) and the past trajectory (cyan); bottom = the three camera images (fl, f, fr).
usage: make_video.py <track_dir> <traj.csv> <marking_map.npz> <out.mp4> [--step 2] [--fps 15] [--terrain] [--dyn_csv dbg]
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
from bev import H, RES, W, X_MAX, Y_MAX, ImageBEV, lane_response  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402
from camera_model import Cameras  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('traj_csv')
ap.add_argument('marks')
ap.add_argument('out')
ap.add_argument('--step', type=int, default=2)
ap.add_argument('--fps', type=float, default=15)
ap.add_argument('--terrain', action='store_true')
ap.add_argument('--dyn_csv', default='')
ap.add_argument('--pano', type=float, default=0, help='bottom row as one cylindrical panorama of radius R m (0: three images)')
ap.add_argument('--pano_az', type=float, default=105, help='panorama half width (deg)')
ap.add_argument('--pano_el', type=float, nargs=2, default=[-30, 20], help='panorama elevation range (deg)')
ap.add_argument('--wide', action='store_true', help='1920x1080: panorama at full width below, BEV (grey road, black mask, white markings) cropped to fill the rest')
ap.add_argument('--bev_lat', type=float, default=0, help='--wide: show +-L m sideways (BEV scaled down, padded); 0 = fill the width')
ap.add_argument('--range', default='', help='lo:hi frames only')
ap.add_argument('--trail', type=int, default=400, help='past frames drawn as the trajectory')
a = ap.parse_args()

track = os.path.basename(os.path.normpath(a.track_dir))
tr = pd.read_csv(a.traj_csv)
B = ImageBEV(a.track_dir, 1.5, 0, 0, -1.5)
MK = MarkingMap(a.marks)
if a.terrain:
    B.terrain = MK.height
    B.TERRAIN_LAT = True
if a.dyn_csv:
    dd = pd.read_csv(a.dyn_csv)
    DY = np.zeros((len(tr), 2))
    for j, c in enumerate(('att_p', 'att_r')):
        if c in dd:
            v = dd[c].values[:len(tr)]
            DY[:len(v), j] = np.nan_to_num(v.astype(float))
    B.dyn = DY
cams = ['fl', 'f', 'fr']
imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
names = {c: sorted(os.listdir(imgdir[c])) for c in cams}

S = 3                                   # BEV upscale
BW, BH = H * S, W * S                   # rotated: width = forward axis, height = lateral axis
CW = BW // 3
CH = int(CW * 464 / 720)
Rm0 = Rotation.from_euler('ZYX', [-1.5, 0, 0], degrees=True).as_matrix()
if a.pano > 0:   # cylinder around the front camera: pixel -> (azimuth, elevation) -> point at radius R -> each camera
    C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
    PW = 1920 if a.wide else BW
    PH = int(PW * (a.pano_el[1] - a.pano_el[0]) / (2 * a.pano_az))
    az = np.radians(np.linspace(a.pano_az, -a.pano_az, PW))          # left of the car on the left
    el = np.radians(np.linspace(a.pano_el[1], a.pano_el[0], PH))
    AZ, EL = np.meshgrid(az, el)
    P = np.stack([a.pano * np.cos(AZ), a.pano * np.sin(AZ), a.pano * np.tan(EL)], -1).reshape(-1, 3)
    PMAP = {}
    for cam in cams:
        K, D_, T, iw, ih = C.cams[cam]
        uv, ok = C.project(cam, P @ Rm0)
        u, v = uv[:, 0], uv[:, 1]
        wgt = np.where(ok, np.minimum.reduce([u, iw - u, v, ih - v]) / 100.0, 0).clip(0, 1)   # feathered seams
        PMAP[cam] = (u.reshape(PH, PW).astype(np.float32), v.reshape(PH, PW).astype(np.float32),
                     wgt.reshape(PH, PW).astype(np.float32))
os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
BOT = PH if a.pano > 0 else CH
if a.wide:
    FW, FH = 1920, 1080
    TOPH = FH - PH
    SF = FW / H if a.bev_lat <= 0 else TOPH / (2 * a.bev_lat / RES)   # px per BEV cell
    CROP0 = int(W * SF / 2 - TOPH / 2)           # lateral crop centred on the car
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (FW, FH))
else:
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (BW, BH + BOT))
X, Y, YAW = tr.x.values, tr.y.values, tr.yaw.values
LO, HI = (0, len(tr)) if not a.range else [int(v) for v in a.range.split(':')]
for i in range(LO, min(HI, len(tr)), a.step):
    x, y, yaw = X[i], Y[i], YAW[i]
    B.pose = (x, y, yaw)
    resp, valid = B.render_resp(i, lane_response)
    if a.wide:   # grey road, black where no camera sees the road (masks, outside the images), white markings
        g = (90 + 165 * np.clip(resp / 2.0, 0, 1)).astype(np.uint8)
        g[~valid] = 0
    else:
        g = np.clip(resp / 2.0 * 255, 0, 255).astype(np.uint8)
    bev = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if not a.wide:
        bev[~valid] = (40, 40, 40)
    lanes = MK.bev(np.array([x, y]), yaw)[0] > 0.5
    bev[lanes] = (0.45 * bev[lanes] + 0.55 * np.array([40, 40, 230])).astype(np.uint8)
    Sx = SF if a.wide else S
    bev = cv2.resize(bev, (int(W * Sx), int(H * Sx)), interpolation=cv2.INTER_NEAREST)
    j0 = max(0, i - a.trail)
    dx, dy = X[j0:i + 1] - x, Y[j0:i + 1] - y
    c, s = np.cos(yaw), np.sin(yaw)
    fw, lt = c * dx + s * dy, -s * dx + c * dy
    rr = ((X_MAX - fw) / RES - 0.5) * Sx
    cc = ((Y_MAX - lt) / RES - 0.5) * Sx
    for r_, c_ in zip(rr.astype(int), cc.astype(int)):
        if 0 <= r_ < bev.shape[0] and 0 <= c_ < bev.shape[1]:
            cv2.circle(bev, (c_, r_), 3 if a.wide else 2, (230, 200, 40), -1)
    bev = cv2.rotate(bev, cv2.ROTATE_90_CLOCKWISE)   # car on the left, driving to the right
    if a.pano > 0:
        Rm = Rm0 if not a.dyn_csv else Rotation.from_euler('ZYX', [-1.5, DY[i, 0], DY[i, 1]], degrees=True).as_matrix()
        Pw = MK.points(x, y, 40.0)
        q = vehicle_frame(Pw, (x, y, yaw), MK.height) if len(Pw) else np.zeros((0, 3))
        q = q[(q[:, 0] > -6) & (q[:, 0] < 30) & road_level(q)] if len(q) else q
        nr = np.hypot(q[:, 0], q[:, 1]) < 15 if len(q) else np.zeros(0, bool)   # near: every point; far: thinned
        q = np.concatenate([q[nr], q[~nr][::max(1, int((~nr).sum()) // 1500)]]) if len(q) else q
        acc = np.zeros((PH, PW, 3), np.float32)
        wsum = np.zeros((PH, PW), np.float32)
        for cam in cams:
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]))
            if img is None:
                continue
            if len(q):   # map markings drawn in the camera image first, so the panorama keeps their true position
                uv, ok = C.project(cam, q @ Rm)
                for u_, v_ in uv[ok].astype(int):
                    cv2.circle(img, (u_, v_), 3, (40, 40, 230), -1)
            mx, my, wg = PMAP[cam]
            acc += cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).astype(np.float32) * wg[..., None]
            wsum += wg
        bottom = (acc / np.maximum(wsum, 1e-6)[..., None]).astype(np.uint8)
    else:
        strip = []
        for cam in cams:
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]))
            strip.append(cv2.resize(img, (CW, CH)) if img is not None else np.zeros((CH, CW, 3), np.uint8))
        bottom = np.concatenate(strip, 1)
    if a.wide:
        top = bev[max(CROP0, 0):max(CROP0, 0) + TOPH, :FW]
        top = cv2.copyMakeBorder(top, 0, TOPH - top.shape[0], (FW - top.shape[1]) // 2,
                                 FW - top.shape[1] - (FW - top.shape[1]) // 2, cv2.BORDER_CONSTANT)
        frame = np.concatenate([top, bottom], 0)
        legend_y = TOPH - 16
    else:
        bottom = cv2.copyMakeBorder(bottom, 0, 0, 0, BW - bottom.shape[1], cv2.BORDER_CONSTANT)
        frame = np.concatenate([bev, bottom], 0)
        legend_y = BH - 16
    cv2.putText(frame, f'{track}  frame {i}', (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(frame, 'white: camera markings   red: map markings   cyan: past trajectory', (16, legend_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    vw.write(frame)
    if i % 300 == 0:
        print(f'frame {i}/{len(tr)}', flush=True)
vw.release()
print('done ->', a.out)
