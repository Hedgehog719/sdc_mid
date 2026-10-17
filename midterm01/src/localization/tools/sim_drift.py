#!/usr/bin/env python3
"""Offline test of map-to-image matching against Track1-like GNSS drift, on a track with a good reference.

make: writes a drifted trajectory (reference + smooth random drift, given rms and correlation time) as CSV,
      used as the prior for bev_match.py.
fuse: RTS-fuses drifted GNSS (sampled at 4 Hz) with bev_match measurements and reports errors vs reference.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d

sys.path.insert(0, os.path.dirname(__file__))
from common import ply_frames  # noqa: E402
from rts import rts_smooth  # noqa: E402


def drift(n, dt, rms, tau, seed):
    rng = np.random.default_rng(seed)
    d = gaussian_filter1d(rng.normal(size=(n, 2)), tau / dt, axis=0, mode='wrap')
    return d / d.std(0) * rms / np.sqrt(2)


def report(name, xy, ref):
    e = np.hypot(xy[:, 0] - ref.x.values, xy[:, 1] - ref.y.values)
    print(f'{name:>18}: pos rmse {np.sqrt(np.mean(e ** 2)):.3f}  median {np.median(e):.3f}  p95 {np.percentile(e, 95):.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['make', 'fuse'])
    ap.add_argument('track_dir')
    ap.add_argument('ref_csv')
    ap.add_argument('drift_csv')
    ap.add_argument('--rms', type=float, default=0.9)
    ap.add_argument('--tau', type=float, default=10.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--bev', default='')
    ap.add_argument('--sharp', type=float, default=0.02)
    ap.add_argument('--bev_sigma', type=float, default=0.5)
    ap.add_argument('--gnss_sigma', type=float, default=0.3)
    ap.add_argument('--qpos', type=float, default=0.2)
    ap.add_argument('--huber', type=float, default=2.0)
    ap.add_argument('--fwd_sigma', type=float, default=0.0, help='>0: anisotropic BEV fix (fwd sigma, lat=bev_sigma)')
    ap.add_argument('--min_speed', type=float, default=3.0)
    ap.add_argument('--peak_ref', type=float, default=0.0)
    a = ap.parse_args()
    ref = pd.read_csv(a.ref_csv)
    ts, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
    dt = np.median(np.diff(ts)) * 1e-9
    if a.cmd == 'make':
        d = drift(len(ref), dt, a.rms, a.tau, a.seed)
        out = ref.copy()
        out['x'] += d[:, 0]
        out['y'] += d[:, 1]
        out.to_csv(a.drift_csv, index=False)
        report('drifted', out[['x', 'y']].values, ref)
        return
    dr = pd.read_csv(a.drift_csv)
    tg = np.arange(ts[0], ts[-1], 0.25e9)
    gx = np.stack([np.interp(tg, ts, dr.x), np.interp(tg, ts, dr.y)], 1)
    yaw_u = np.unwrap(ref.yaw.values)
    psi = lambda t: np.interp(np.atleast_1d(t).astype(float), ts.astype(float), yaw_u)  # noqa: E731
    report('drifted GNSS', dr[['x', 'y']].values, ref)
    st = rts_smooth(ts, tg, gx, psi, a.gnss_sigma, 1.0, a.qpos)
    report('RTS(GNSS)', st[:, :2], ref)
    if a.bev:
        z = np.load(a.bev)
        speed = np.interp(z['ts'].astype(float), ts.astype(float), np.abs(st[:, 2]))
        m = (z['sharp'] > a.sharp) & (speed > a.min_speed)
        tm = np.concatenate([tg, z['ts'][m]])
        zm = np.concatenate([gx, np.stack([z['mx'][m], z['my'][m]], 1)])
        cov = np.zeros((len(tm), 2, 2))
        cov[:len(tg)] = np.eye(2) * a.gnss_sigma ** 2
        yw = z['pyaw'][m]
        sf = a.fwd_sigma if a.fwd_sigma > 0 else a.bev_sigma
        for k, y in enumerate(yw):
            Rr = np.array([[np.cos(y), -np.sin(y)], [np.sin(y), np.cos(y)]])
            sc = np.clip(a.peak_ref / max(z['peak'][m][k], 1e-3), 0.5, 3.0) if a.peak_ref > 0 else 1.0
            cov[len(tg) + k] = Rr @ np.diag([sf ** 2, a.bev_sigma ** 2]) @ Rr.T * sc ** 2
        st2 = rts_smooth(ts, tm, zm, psi, a.gnss_sigma, 1.0, a.qpos, huber=a.huber, meas_cov=cov)
        report(f'RTS(GNSS+BEV n={m.sum()})', st2[:, :2], ref)


if __name__ == '__main__':
    main()
