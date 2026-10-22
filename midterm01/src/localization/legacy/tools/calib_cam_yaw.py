#!/usr/bin/env python3
"""Per-camera yaw (and pitch) correction from pole landmarks on a track with an accurate reference trajectory.

For each camera, poles near the route are projected with a candidate extra rotation and scored by the image edge
energy across the projected pole (same score as pole_match.py); the angle maximising the summed score over many
frames is the extrinsic correction for that camera.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
import pole_match as PM  # noqa: E402
from bev import GroundRaster  # noqa: E402
from camera_model import Cameras  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv')
    ap.add_argument('--poles', default='poles.npy')
    ap.add_argument('--frames', default='0:3299:12')
    ap.add_argument('--yaws', type=float, nargs='+', default=list(np.arange(-4, 4.01, 0.5)))
    ap.add_argument('--pitches', type=float, nargs='+', default=[0.0])
    a = ap.parse_args()
    track = os.path.basename(os.path.normpath(a.track_dir))
    PM.A = argparse.Namespace(tensor_sigma=2.0)
    tr = pd.read_csv(a.traj_csv)
    P = np.load(os.path.join(a.track_dir, a.poles))
    G = GroundRaster(np.load(os.path.join(a.track_dir, 'map.npy'), mmap_mode='r'))
    C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
    cams = ['f', 'fl', 'fr']
    imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
    names = {c: sorted(os.listdir(imgdir[c])) for c in cams}
    masks = {c: np.load(os.path.join(a.track_dir, 'masks', f'{c}.npz'))['masks'] for c in cams}
    lo, hi, st = [int(v) for v in a.frames.split(':')]
    hs = np.linspace(0.3, 1.0, 14)
    score = {c: np.zeros((len(a.yaws), len(a.pitches))) for c in cams}
    count = {c: np.zeros((len(a.yaws), len(a.pitches))) for c in cams}
    for i in range(lo, min(hi, len(tr)), st):
        x, y, yaw = tr.x[i], tr.y[i], tr.yaw[i]
        c_, s_ = np.cos(yaw), np.sin(yaw)
        d = P[:, :2] - [x, y]
        u = c_ * d[:, 0] + s_ * d[:, 1]
        v = -s_ * d[:, 0] + c_ * d[:, 1]
        r = np.hypot(u, v)
        sel = (r > 4) & (r < 30)
        if not sel.any():
            continue
        zv = G.height(np.array([x]), np.array([y]))[0] + 1.5
        Q = []
        for k in np.flatnonzero(sel):
            hh = hs * min(P[k, 4], 5.0)
            Q.append(np.stack([np.full(len(hh), u[k]), np.full(len(hh), v[k]), P[k, 2] + hh - zv], 1))
        Q = np.stack(Q)  # (np, ns, 3)
        for cam in cams:
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]), cv2.IMREAD_GRAYSCALE)
            m = np.unpackbits(masks[cam][i], axis=1)[:, :img.shape[1] // 2]
            mask = cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            Jxx, Jyy, Jxy = PM.edge_tensor(img, mask)
            for a_, yw in enumerate(a.yaws):
                for b_, pt in enumerate(a.pitches):
                    R = Rotation.from_euler('ZYX', [-1.5 + yw, pt, 0], degrees=True).as_matrix()
                    q = Q.reshape(-1, 3) @ R
                    uv, ok = C.project(cam, q)
                    uv = uv.reshape(Q.shape[0], Q.shape[1], 2)
                    ok = ok.reshape(Q.shape[0], Q.shape[1])
                    du = np.gradient(uv, axis=1)
                    n = np.stack([-du[..., 1], du[..., 0]], -1)
                    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-9
                    xi = np.clip(uv[..., 0].astype(int), 0, img.shape[1] - 1)
                    yi = np.clip(uv[..., 1].astype(int), 0, img.shape[0] - 1)
                    e = Jxx[yi, xi] * n[..., 0] ** 2 + 2 * Jxy[yi, xi] * n[..., 0] * n[..., 1] + Jyy[yi, xi] * n[..., 1] ** 2
                    vis = ok.mean(1) > 0.6
                    score[cam][a_, b_] += np.minimum(np.where(ok, e, 0).sum(1)[vis] / np.maximum(ok.sum(1)[vis], 1), 8).sum()
                    count[cam][a_, b_] += vis.sum()
    for cam in cams:
        s = score[cam] / np.maximum(count[cam], 1)
        a_, b_ = np.unravel_index(np.argmax(s), s.shape)
        prof = ' '.join(f'{yw:+.1f}:{s[k, b_]:.2f}' for k, yw in enumerate(a.yaws))
        print(f'{cam}: best extra yaw {a.yaws[a_]:+.1f} deg, pitch {a.pitches[b_]:+.1f} deg (poles scored {int(count[cam][a_, b_])})')
        print(f'     yaw profile: {prof}')


if __name__ == '__main__':
    main()
