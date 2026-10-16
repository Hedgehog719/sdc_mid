#!/usr/bin/env python3
"""Fuse GNSS/IMU priors with scan-matching corrections and smooth the trajectory.

Scan matching of image clouds is noisy frame to frame (degenerate along the road), but its slowly varying
part carries the GNSS bias (mainly lateral) and heading error. We therefore:
  1. express each registration result as a correction to the prior in the vehicle frame (fwd, lat, yaw),
  2. gate outliers and weight by registration quality,
  3. robustly low-pass the corrections (weighted Gaussian, iterated Huber reweighting),
  4. apply the smoothed corrections to the GNSS position / IMU yaw, optionally smooth the final path.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

sys.path.insert(0, os.path.dirname(__file__))
from common import Sensors, ply_frames, wrap, yaw_of  # noqa: E402


def robust_smooth(v, w, sigma, iters=3, huber=1.0):
    """Weighted Gaussian smoothing with Huber reweighting; sigma in frames (0 = global weighted mean)."""
    w0 = w.copy()
    s = v
    for _ in range(iters):
        if sigma <= 0:
            s = np.full_like(v, np.sum(w * v) / max(np.sum(w), 1e-9))
        else:
            s = gaussian_filter1d(w * v, sigma, mode='nearest') / np.maximum(gaussian_filter1d(w, sigma, mode='nearest'), 1e-9)
        r = np.abs(v - s)
        w = w0 * np.where(r < huber, 1.0, huber / np.maximum(r, 1e-9))
    return s


def main():
    p = argparse.ArgumentParser()
    p.add_argument('track_dir')
    p.add_argument('output_csv')
    p.add_argument('--reg', nargs='*', default=[], help='register.py npz outputs (optional)')
    p.add_argument('--ply_dir', default='raw_ply', help='defines the frame list / row count')
    p.add_argument('--yaw_src', choices=['imu', 'gnss', 'imu_cf'], default='imu')
    p.add_argument('--yaw_cf_sigma', type=float, default=10.0, help='seconds, for imu_cf')
    p.add_argument('--interp', choices=['linear', 'cubic'], default='linear')
    p.add_argument('--use', default='fwd,lat,yaw', help='which corrections to apply')
    p.add_argument('--sigma_fwd', type=float, default=150.0)
    p.add_argument('--sigma_lat', type=float, default=60.0)
    p.add_argument('--sigma_yaw', type=float, default=60.0)
    p.add_argument('--gate_xy', type=float, default=3.0)
    p.add_argument('--gate_yaw', type=float, default=6.0, help='deg')
    p.add_argument('--gain', type=float, default=1.0, help='scale applied to smoothed corrections')
    p.add_argument('--smooth_xy', type=float, default=0.0, help='final Gaussian smoothing of x/y (frames)')
    p.add_argument('--smooth_yaw', type=float, default=0.0)
    p.add_argument('--lever', type=float, nargs=2, default=[0.0, 0.0],
                   help='constant (fwd, lat) offset GNSS->reference point in the vehicle frame')
    p.add_argument('--rts', action='store_true', help='Kalman/RTS smoothing of GNSS with IMU heading')
    p.add_argument('--rts_sigma', type=float, default=0.3)
    p.add_argument('--rts_qacc', type=float, default=1.0)
    p.add_argument('--rts_qpos', type=float, default=0.05)
    p.add_argument('--rts_scan_sigma', type=float, default=0.0,
                   help='>0: feed gated scan-matching positions to the RTS smoother with this sigma (m)')
    p.add_argument('--dump', default='', help='optional npz with corrections for inspection')
    a = p.parse_args()

    ts, _ = ply_frames(os.path.join(a.track_dir, a.ply_dir))
    s = Sensors(a.track_dir)
    xyz = s.gnss_xyz(ts, a.interp)
    if a.yaw_src == 'imu':
        yaw = s.imu_yaw(ts)
    elif a.yaw_src == 'imu_cf':
        yaw = s.imu_yaw_cf(ts, a.yaw_cf_sigma)
    else:
        yaw = s.gnss_yaw(ts)
    if a.rts:
        from rts import rts_smooth
        if a.yaw_src == 'imu_cf':
            psi_fn = lambda t: s.imu_yaw_cf(t, a.yaw_cf_sigma)  # noqa: E731
        else:
            psi_fn = s.imu_yaw
        tm, zm, sm = s.gt, s.gxyz[:, :2], np.full(len(s.gt), a.rts_sigma)
        if a.reg and a.rts_scan_sigma > 0:
            z = {k: np.concatenate([np.load(f)[k] for f in a.reg]) for k in ['ts', 'T', 'P', 'inlier', 'map_keep']}
            d = z['T'][:, :2, 3] - z['P'][:, :2, 3]
            dyaw = wrap(yaw_of(z['T']) - yaw_of(z['P']))
            w = z['inlier'] * z['map_keep']
            good = (np.linalg.norm(d, axis=1) < a.gate_xy) & (np.abs(np.degrees(dyaw)) < a.gate_yaw) & (w > 0.3)
            # scan position = GNSS(at frame) + correction, so the lever arm / interpolation stay consistent
            gxy = s.gnss_xyz(z['ts'][good], a.interp)[:, :2]
            tm = np.concatenate([tm, z['ts'][good]])
            zm = np.concatenate([zm, gxy + d[good]])
            sm = np.concatenate([sm, a.rts_scan_sigma / np.sqrt(np.clip(w[good], 0.1, 1))])
            print('scan measurements used: %d / %d' % (good.sum(), len(good)))
        st = rts_smooth(ts, tm, zm, psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos, meas_sigma=sm)
        xyz = xyz.copy()
        xyz[:, :2] = st[:, :2]
    n = len(ts)
    corr = np.zeros((n, 3))  # fwd, lat, yaw(rad)
    if a.reg:
        z = {k: np.concatenate([np.load(f)[k] for f in a.reg]) for k in ['ts', 'T', 'P', 'inlier', 'map_keep']}
        pos = {t: i for i, t in enumerate(ts)}
        idx = np.array([pos.get(t, -1) for t in z['ts']])
        ok = idx >= 0
        T, P = z['T'][ok], z['P'][ok]
        idx = idx[ok]
        Ryaw = yaw_of(P)
        d = T[:, :3, 3] - P[:, :3, 3]
        fwd = np.cos(Ryaw) * d[:, 0] + np.sin(Ryaw) * d[:, 1]
        lat = -np.sin(Ryaw) * d[:, 0] + np.cos(Ryaw) * d[:, 1]
        dyaw = wrap(yaw_of(T) - Ryaw)
        w = (z['inlier'][ok] * z['map_keep'][ok]).astype(float)
        good = (np.abs(fwd) < a.gate_xy) & (np.abs(lat) < a.gate_xy) & (np.abs(np.degrees(dyaw)) < a.gate_yaw)
        W = np.zeros(n)
        V = np.zeros((n, 3))
        W[idx] = w * good
        V[idx] = np.stack([fwd, lat, dyaw], 1)
        use = a.use.split(',')
        for k, (name, sig, hub) in enumerate([('fwd', a.sigma_fwd, 1.0), ('lat', a.sigma_lat, 0.7),
                                              ('yaw', a.sigma_yaw, np.radians(2))]):
            if name in use:
                corr[:, k] = a.gain * robust_smooth(V[:, k], W, sig, huber=hub)
        print('good frames %d/%d  mean corr fwd %.2f lat %.2f yaw %.2f deg' % (
            good.sum(), len(good), corr[:, 0].mean(), corr[:, 1].mean(), np.degrees(corr[:, 2].mean())))
        if a.dump:
            np.savez(a.dump, V=V, W=W, corr=corr)
    corr[:, 0] += a.lever[0]
    corr[:, 1] += a.lever[1]
    x = xyz[:, 0] + np.cos(yaw) * corr[:, 0] - np.sin(yaw) * corr[:, 1]
    y = xyz[:, 1] + np.sin(yaw) * corr[:, 0] + np.cos(yaw) * corr[:, 1]
    yw = np.unwrap(yaw + corr[:, 2])
    if a.smooth_xy > 0:
        x = gaussian_filter1d(x, a.smooth_xy, mode='nearest')
        y = gaussian_filter1d(y, a.smooth_xy, mode='nearest')
    if a.smooth_yaw > 0:
        yw = gaussian_filter1d(yw, a.smooth_yaw, mode='nearest')
    pd.DataFrame({'idx': np.arange(n), 'x': x, 'y': y, 'yaw': wrap(yw)}).to_csv(a.output_csv, index=False)
    print('wrote', a.output_csv, n)


if __name__ == '__main__':
    main()
