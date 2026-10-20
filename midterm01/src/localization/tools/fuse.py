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


def bev_edge(dyaw, max_yaw=None):
    """BEV fixes whose yaw hit the +-max_yaw search limit (the optimum lies outside: unreliable).
    max_yaw: per-fix search limit in deg (npz key 'max_yaw'); default = the largest |dyaw| in the data."""
    d = np.abs(np.degrees(dyaw))
    if not len(d):
        return np.zeros(0, bool)
    lim = np.full(len(d), d.max()) if max_yaw is None else np.asarray(max_yaw, float)
    return d >= lim - 0.01


def npz_max_yaw(parts):
    return np.concatenate([q['max_yaw'] for q in parts]) if all('max_yaw' in q for q in parts) else None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('track_dir')
    p.add_argument('output_csv')
    p.add_argument('--reg', nargs='*', default=[], help='register.py npz outputs (optional)')
    p.add_argument('--ply_dir', default='raw_ply', help='defines the frame list / row count')
    p.add_argument('--yaw_src', choices=['imu', 'gnss', 'imu_cf', 'path'], default='imu')
    p.add_argument('--yaw_cf_ref', choices=['gnss', 'motion'], default='gnss')
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
    p.add_argument('--path_iters', type=int, default=2, help='yaw_src=path: RTS <-> heading iterations')
    p.add_argument('--path_min_speed', type=float, default=2.0)
    p.add_argument('--yaw_bias', type=float, default=0.0, help='deg added to the final yaw')
    p.add_argument('--bev', nargs='*', default=[], help='bev_match.py npz file(s): map-to-image fixes fed to the RTS smoother')
    p.add_argument('--bev_lat_sigma', type=float, default=0.3)
    p.add_argument('--bev_fwd_sigma', type=float, default=5.0, help='large = lateral-only fix')
    p.add_argument('--bev_aniso_k', type=float, default=0.0,
                   help='>0: per-fix fwd sigma = clip(k / sharp_f, aniso_min, bev_fwd_sigma) (needs sharp_f in npz)')
    p.add_argument('--bev_aniso_min', type=float, default=0.3)
    p.add_argument('--yaw_from_bev', default='', help='npz whose (smoothed) BEV yaw replaces the yaw over --yaw_from_bev_frames')
    p.add_argument('--yaw_from_bev_frames', default='0:180:300', help='lo:hi:end - full replacement lo..hi, blend to end')
    p.add_argument('--gnss_bias_tau', type=float, default=0.0,
                   help='>0: model GNSS errors as a Gauss-Markov bias state with this correlation time (s)')
    p.add_argument('--gnss_bias_sigma', type=float, default=0.5, help='GNSS bias steady-state sigma (m)')
    p.add_argument('--gnss_white', type=float, default=0.05, help='GNSS white-noise sigma (m) with the bias state')
    p.add_argument('--map_bias_len', type=float, default=0.0,
                   help='>0: common error of the map fixes as Gauss-Markov in distance, correlation length (m)')
    p.add_argument('--map_bias_sigma', type=float, default=0.15, help='map-error steady-state sigma (m) per axis')
    p.add_argument('--gnss_huber', type=float, default=0.0, help='>0: Huber threshold for the GNSS rows (bias model)')
    p.add_argument('--nis', action='store_true', help='print the filter consistency (NIS) per measurement group')
    p.add_argument('--gnss_lat_sigma', type=float, default=0.0, help='>0: extra lateral (cross-track) noise of the GNSS rows '
                   '(m): GNSS keeps its forward information while the map fixes decide the lateral position')
    p.add_argument('--gnss_keep_s', type=float, default=-1.0, help='>=0: GNSS-free run - keep only the first N seconds of '
                   'GNSS (position + yaw) as the initial guess (Track03 rehearsal: no GNSS pose provided)')
    p.add_argument('--psi_csv', default='', help='CSV whose yaw is the heading input of the position RTS')
    p.add_argument('--bev_keep_edge', action='store_true',
                   help='keep BEV fixes whose yaw hit the search limit (default: dropped, they are unreliable)')
    p.add_argument('--bev_aniso_lat', action='store_true', help='also lat sigma = clip(k / sharp_l, aniso_min, 3 lat)')
    p.add_argument('--bev_min_speed', type=float, default=3.0)
    p.add_argument('--bev_huber', type=float, default=2.0)
    p.add_argument('--bev_peak_ref', type=float, default=0.0,
                   help='>0: scale BEV sigma by clip(ref/peak, 0.5, 3): weak matches (occluded / few markings) count less')
    p.add_argument('--xy_bias', type=float, nargs=2, default=[0.0, 0.0], help='m subtracted from final x/y')
    p.add_argument('--time_shift', type=float, default=0.0, help='s, resample the final trajectory at t + shift')
    p.add_argument('--rts_qlat', type=float, default=None, help='cross-track process noise (NHC); default = isotropic')
    p.add_argument('--zupt_speed', type=float, default=0.0, help='>0: zero-velocity updates where speed < this (m/s)')
    p.add_argument('--zupt_hold_yaw', action='store_true', help='hold yaw constant while stationary (ZIHR)')
    p.add_argument('--yaw_rts', default='', choices=['', 'gnss', 'traj'],
                   help='replace yaw by IMU yaw + RTS-smoothed offset; reference = GNSS fixes or the fused trajectory')
    p.add_argument('--yaw_rts_q', type=float, default=0.05, help='offset random walk, deg/sqrt(s)')
    p.add_argument('--yaw_rts_base', type=float, default=1.0, help='s, baseline for the motion heading')
    p.add_argument('--yaw_rts_sigpos', type=float, default=0.15, help='m, position noise of the heading reference')
    p.add_argument('--yaw_rts_bev', type=float, default=0.0, help='>0: also use BEV yaw fixes with this sigma (deg)')
    p.add_argument('--imu_acc', action='store_true', help='IMU acceleration (gravity removed) as a known input of the '
                   'along-track speed in the RTS; set --rts_qacc to the IMU error level (e.g. 0.2)')
    p.add_argument('--bev_yaw_skip', default='', help='lo:hi frames whose BEV fixes are used for position only, not yaw '
                   '(map section with an unreliable orientation, e.g. a rotated start plate)')
    p.add_argument('--pole', default='', help='pole_match.py npz: pole-landmark fixes (along-track constraint)')
    p.add_argument('--pole_pct', type=float, default=50, help='keep pole fixes whose peak is above this percentile')
    p.add_argument('--pole_fwd_sigma', type=float, default=0.3)
    p.add_argument('--pole_lat_sigma', type=float, default=0.6)
    p.add_argument('--pole_min_speed', type=float, default=0.0, help='drop pole fixes below this speed (m/s)')
    p.add_argument('--road', default='', help='bev_match.py --feature road npz: lateral fixes from the road region')
    p.add_argument('--near', default='', help='near_fix.py npz: near-range image-to-map fixes (road markings 1-12 m ahead)')
    p.add_argument('--near_fwd_sigma', type=float, default=0.3)
    p.add_argument('--near_lat_sigma', type=float, default=0.15)
    p.add_argument('--near_min_sharp', type=float, default=0.0, help='keep near fixes whose lateral peak sharpness >= this')
    p.add_argument('--near_yaw_sigma', type=float, default=0.0, help='>0: near-fix headings (near_fix.py --yaw_search) '
                   'join the yaw smoother with this sigma (deg); only fixes with a clear heading peak (sharp_y > 0)')
    p.add_argument('--near_aniso_k', type=float, default=0.0, help='>0: per-fix sigma = clip(k / peak sharpness, base sigma, '
                   '2 m fwd / 1 m lat) - flat forward peaks (lines along the road) get little forward weight')
    p.add_argument('--road_pct', type=float, default=50)
    p.add_argument('--road_lat_sigma', type=float, default=0.5)
    p.add_argument('--road_fwd_sigma', type=float, default=5.0)
    p.add_argument('--start_hold', type=float, default=0.0,
                   help='>0: while the IMU heading has changed less than this (deg) since the start and speed < '
                        '--start_speed, hold the yaw constant and put the positions on a straight line')
    p.add_argument('--start_speed', type=float, default=4.0)
    p.add_argument('--dump', default='', help='optional npz with corrections for inspection')
    a = p.parse_args()

    ts, _ = ply_frames(os.path.join(a.track_dir, a.ply_dir))
    s = Sensors(a.track_dir)
    if a.gnss_keep_s >= 0:   # GNSS-free: the first seconds only (initial pose / heading)
        kg = s.gt <= s.gt[0] + max(a.gnss_keep_s, 0.5) * 1e9
        kg[:2] = True
        s.gt, s.gxyz, s.gyaw = s.gt[kg], s.gxyz[kg], s.gyaw[kg]
        print(f'GNSS-free: kept {kg.sum()} GNSS samples (first {a.gnss_keep_s:.1f} s) as the initial guess')
    xyz = s.gnss_xyz(ts, a.interp)
    if a.yaw_src == 'imu':
        yaw = s.imu_yaw(ts)
    elif a.yaw_src == 'imu_cf':
        yaw = s.imu_yaw_cf(ts, a.yaw_cf_sigma, a.yaw_cf_ref)
    else:
        yaw = s.gnss_yaw(ts)
    if a.yaw_src == 'path':
        # heading of travel of the RTS-smoothed path, fused with IMU yaw (complementary), iterated
        from rts import rts_smooth
        from scipy.interpolate import interp1d
        psi_fn = lambda t: s.imu_yaw_cf(t, a.yaw_cf_sigma, 'gnss')  # noqa: E731
        imu_ts = np.unwrap(s.imu_rot(ts).as_euler('ZYX')[:, 0])
        frame_dt = np.median(np.diff(ts)) * 1e-9
        for _ in range(a.path_iters):
            st = rts_smooth(ts, s.gt, s.gxyz[:, :2], psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos)
            d = np.gradient(st[:, :2], axis=0)
            heading = np.arctan2(d[:, 1], d[:, 0])
            w = (np.linalg.norm(d, axis=1) / frame_dt > a.path_min_speed).astype(float)
            off = np.unwrap(wrap(heading - imu_ts))
            sig = a.yaw_cf_sigma / frame_dt
            off_s = gaussian_filter1d(off * w, sig, mode='nearest') / np.maximum(gaussian_filter1d(w, sig, mode='nearest'), 1e-6)
            yaw = imu_ts + off_s
            f = interp1d(ts.astype(np.float64), yaw, bounds_error=False, fill_value=(yaw[0], yaw[-1]))
            psi_fn = lambda t, f=f: f(np.atleast_1d(t).astype(np.float64))  # noqa: E731
    ZUPT_MASK = np.zeros(len(ts), bool)
    if a.rts:
        from rts import rts_smooth
        if a.yaw_src == 'imu_cf':
            psi_fn = lambda t: s.imu_yaw_cf(t, a.yaw_cf_sigma, a.yaw_cf_ref)  # noqa: E731
        elif a.yaw_src != 'path':
            psi_fn = s.imu_yaw
        if a.psi_csv:  # heading input of the position RTS from an earlier, better-headed solution
            from scipy.interpolate import interp1d
            _pc = pd.read_csv(a.psi_csv)
            _f = interp1d(ts.astype(np.float64), np.unwrap(_pc.yaw.values), bounds_error=False,
                          fill_value=(_pc.yaw.values[0], _pc.yaw.values[-1]))
            psi_fn = lambda t, f=_f: f(np.atleast_1d(t).astype(np.float64))  # noqa: E731
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
        cov = np.eye(2)[None] * (sm ** 2)[:, None, None]
        GROUPS = [('gnss', len(tm))]
        huber = 3.0
        if a.bev:
            st0 = rts_smooth(ts, tm, zm, psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos, meas_sigma=sm)
            parts = [np.load(f) for f in a.bev]
            keys = ['ts', 'pyaw', 'mx', 'my', 'peak', 'dyaw']
            if a.bev_aniso_k > 0:
                keys += ['sharp_f', 'sharp_l']
            b = {k: np.concatenate([q[k] for q in parts]) for k in keys}
            speed = np.interp(b['ts'].astype(float), ts.astype(float), np.abs(st0[:, 2]))
            m = speed > a.bev_min_speed
            if not a.bev_keep_edge:
                m &= ~bev_edge(b['dyaw'], npz_max_yaw(parts))
            yw = b['pyaw'][m]
            c, s_ = np.cos(yw), np.sin(yw)
            Rr = np.stack([np.stack([c, -s_], 1), np.stack([s_, c], 1)], 1)
            sf = np.full(m.sum(), a.bev_fwd_sigma)
            sl = np.full(m.sum(), a.bev_lat_sigma)
            if a.bev_aniso_k > 0:  # NCC peak distinctiveness along each axis -> per-fix anisotropic covariance
                sf = np.clip(a.bev_aniso_k / np.maximum(b['sharp_f'][m], 1e-3), a.bev_aniso_min, a.bev_fwd_sigma)
                if a.bev_aniso_lat:
                    sl = np.clip(a.bev_aniso_k / np.maximum(b['sharp_l'][m], 1e-3), a.bev_aniso_min, 3 * a.bev_lat_sigma)
            D = np.zeros((m.sum(), 2, 2))
            D[:, 0, 0], D[:, 1, 1] = sf ** 2, sl ** 2
            bc = Rr @ D @ Rr.transpose(0, 2, 1)
            if a.bev_peak_ref > 0:
                sc = np.clip(a.bev_peak_ref / np.maximum(b['peak'][m], 1e-3), 0.5, 3.0)
                bc = bc * (sc ** 2)[:, None, None]
            tm = np.concatenate([tm, b['ts'][m]])
            zm = np.concatenate([zm, np.stack([b['mx'][m], b['my'][m]], 1)])
            cov = np.concatenate([cov, bc])
            huber = a.bev_huber
            print('BEV fixes used: %d / %d' % (m.sum(), len(m)))
            GROUPS.append(('bev', len(tm)))
        if a.pole:
            pz = np.load(a.pole)
            pm = pz['peak'] >= np.percentile(pz['peak'], a.pole_pct)
            if a.pole_min_speed > 0:  # pole association breaks down when (nearly) stationary
                st_p = rts_smooth(ts, s.gt, s.gxyz[:, :2], psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos)
                pm &= np.interp(pz['ts'].astype(float), ts.astype(float), np.abs(st_p[:, 2])) >= a.pole_min_speed
            yw = pz['pyaw'][pm]
            c, s_ = np.cos(yw), np.sin(yw)
            Rr = np.stack([np.stack([c, -s_], 1), np.stack([s_, c], 1)], 1)
            pc = Rr @ np.diag([a.pole_fwd_sigma ** 2, a.pole_lat_sigma ** 2]) @ Rr.transpose(0, 2, 1)
            tm = np.concatenate([tm, pz['ts'][pm]])
            zm = np.concatenate([zm, np.stack([pz['mx'][pm], pz['my'][pm]], 1)])
            cov = np.concatenate([cov, pc])
            huber = a.bev_huber
            print('pole fixes used: %d / %d' % (pm.sum(), len(pm)))
            GROUPS.append(('pole', len(tm)))
        if a.near:
            nz = np.load(a.near)
            nm_ = nz['sharp_l'] >= a.near_min_sharp
            yw = nz['pyaw'][nm_]
            c, s_ = np.cos(yw), np.sin(yw)
            Rr = np.stack([np.stack([c, -s_], 1), np.stack([s_, c], 1)], 1)
            if a.near_aniso_k > 0:
                sfn = np.clip(a.near_aniso_k / np.maximum(nz['sharp_f'][nm_], 1e-4), a.near_fwd_sigma, 2.0)
                sln = np.clip(a.near_aniso_k / np.maximum(nz['sharp_l'][nm_], 1e-4), a.near_lat_sigma, 1.0)
                D_ = np.zeros((nm_.sum(), 2, 2))
                D_[:, 0, 0], D_[:, 1, 1] = sfn ** 2, sln ** 2
                ncv = Rr @ D_ @ Rr.transpose(0, 2, 1)
            else:
                ncv = Rr @ np.diag([a.near_fwd_sigma ** 2, a.near_lat_sigma ** 2]) @ Rr.transpose(0, 2, 1)
            tm = np.concatenate([tm, nz['ts_i'][nm_]])
            zm = np.concatenate([zm, np.stack([nz['mx'][nm_], nz['my'][nm_]], 1)])
            cov = np.concatenate([cov, ncv])
            huber = a.bev_huber
            print('near fixes used: %d / %d' % (nm_.sum(), len(nm_)))
            GROUPS.append(('near', len(tm)))
        if a.road:
            rz = np.load(a.road)
            rm = rz['peak'] >= np.percentile(rz['peak'], a.road_pct)
            yw = rz['pyaw'][rm]
            c, s_ = np.cos(yw), np.sin(yw)
            Rr = np.stack([np.stack([c, -s_], 1), np.stack([s_, c], 1)], 1)
            rcv = Rr @ np.diag([a.road_fwd_sigma ** 2, a.road_lat_sigma ** 2]) @ Rr.transpose(0, 2, 1)
            tm = np.concatenate([tm, rz['ts'][rm]])
            zm = np.concatenate([zm, np.stack([rz['mx'][rm], rz['my'][rm]], 1)])
            cov = np.concatenate([cov, rcv])
            huber = a.bev_huber
            print('road fixes used: %d / %d' % (rm.sum(), len(rm)))
            GROUPS.append(('road', len(tm)))
        tz = None
        if a.zupt_speed > 0:
            st0 = rts_smooth(ts, tm, zm, psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos, huber=huber, meas_cov=cov)
            from scipy.ndimage import binary_opening
            still = binary_opening(np.abs(st0[:, 2]) < a.zupt_speed, np.ones(15, bool))
            tz = ts[still]
            ZUPT_MASK[:] = still
            print('ZUPT frames: %d' % still.sum())
        gb, gmask = None, None
        if a.gnss_bias_tau > 0:  # coloured GNSS noise: bias states (the first len(s.gt) rows are GNSS)
            gb = (a.gnss_bias_tau, a.gnss_bias_sigma, a.gnss_white)
            gmask = np.zeros(len(tm), bool)
            gmask[:len(s.gt)] = True
        mb, mmask = None, None
        if a.map_bias_len > 0:  # the map fixes (all rows after the GNSS ones) share the map's own error
            mb = (a.map_bias_len, a.map_bias_sigma)
            mmask = np.zeros(len(tm), bool)
            mmask[len(s.gt):] = True
        acc_fn = None
        if a.imu_acc:  # IMU specific force -> world frame, gravity removed, lightly smoothed (bag receive time)
            from scipy.spatial.transform import Rotation as _R
            from scipy.ndimage import gaussian_filter1d as _g
            _im = pd.read_csv(os.path.join(a.track_dir, 'sensors', 'imu.csv'))
            _t = _im.t.values.astype(np.float64)
            _aw = _R.from_quat(_im[['qx', 'qy', 'qz', 'qw']].values).apply(_im[['ax', 'ay', 'az']].values)
            _dt = np.median(np.diff(_t)) * 1e-9
            _ax, _ay = _g(_aw[:, 0], 0.05 / _dt), _g(_aw[:, 1], 0.05 / _dt)
            def acc_fn(tt):   # rotate from the IMU's world yaw into the map heading frame used by the smoother
                off = psi_fn(tt) - np.unwrap(s.imu_rot(tt).as_euler('ZYX')[:, 0])
                ax_, ay_ = np.interp(tt, _t, _ax), np.interp(tt, _t, _ay)
                return np.stack([np.cos(off) * ax_ - np.sin(off) * ay_, np.sin(off) * ax_ + np.cos(off) * ay_], 1)
        st = rts_smooth(ts, tm, zm, psi_fn, a.rts_sigma, a.rts_qacc, a.rts_qpos, huber=huber, meas_cov=cov,
                        q_lat=a.rts_qlat, t_zupt=tz, gnss_bias=gb, gnss_mask=gmask, map_bias=mb, map_mask=mmask,
                        return_nis=a.nis, huber_g=a.gnss_huber if a.gnss_huber > 0 else None, acc_fn=acc_fn,
                        gnss_lat_extra=a.gnss_lat_sigma)
        if a.nis:  # filter consistency per group: mean NIS ~ 2 (2D) if the noise model matches the data
            st, nis = st
            lo_ = 0
            for gname, hi_ in GROUPS:
                v = nis[lo_:hi_]
                v = v[np.isfinite(v)]
                if len(v):
                    print(f'NIS {gname:5s} n {len(v):5d}  mean {v.mean():7.2f}  median {np.median(v):6.2f}  '
                          f'(chi2(2): mean 2, median 1.39)  share > 9.2: {np.mean(v > 9.21) * 100:4.1f} % (1 % expected)')
                lo_ = hi_
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
    yw = np.unwrap(yaw + corr[:, 2]) + np.radians(a.yaw_bias)
    if a.yaw_rts:
        from yaw_rts import motion_heading, yaw_offset_rts
        imu_fn = lambda t: np.unwrap(s.imu_rot(np.atleast_1d(t)).as_euler('ZYX')[:, 0])  # noqa: E731
        if a.yaw_rts == 'gnss':
            mt, my_, ms = motion_heading(s.gt.astype(np.int64), s.gxyz[:, :2], a.yaw_rts_base, a.yaw_rts_sigpos)
        else:  # fused trajectory (BEV-corrected positions)
            mt, my_, ms = motion_heading(ts, np.stack([x, y], 1), a.yaw_rts_base, a.yaw_rts_sigpos)
        if a.yaw_rts_bev > 0 and a.bev:
            parts = [np.load(f) for f in a.bev]
            bt = np.concatenate([q['ts'] for q in parts])
            byaw = np.concatenate([q['myaw'] for q in parts])
            ok = np.ones(len(bt), bool)
            if not a.bev_keep_edge:
                ok &= ~bev_edge(np.concatenate([q['dyaw'] for q in parts]), npz_max_yaw(parts))
            if a.bev_yaw_skip:
                lo, hi = (int(v) for v in a.bev_yaw_skip.split(':'))
                bi = np.concatenate([q['idx'] for q in parts])
                ok &= ~((bi >= lo) & (bi < hi))
            bt, byaw = bt[ok], byaw[ok]
            mt = np.concatenate([mt, bt])
            my_ = np.concatenate([my_, byaw])
            ms = np.concatenate([ms, np.full(len(bt), np.radians(a.yaw_rts_bev))])
        if a.near and a.near_yaw_sigma > 0:
            nz_ = np.load(a.near)
            if 'sharp_y' in nz_:
                ky = nz_['sharp_y'] > 0
                mt = np.concatenate([mt, nz_['ts_i'][ky].astype(mt.dtype)])
                my_ = np.concatenate([my_, nz_['myaw'][ky]])
                ms = np.concatenate([ms, np.full(ky.sum(), np.radians(a.near_yaw_sigma))])
                print('near yaw fixes used: %d / %d' % (ky.sum(), len(ky)))
        o = np.argsort(mt)
        b = yaw_offset_rts(ts, imu_fn, mt[o], my_[o], ms[o], a.yaw_rts_q)
        yw = imu_fn(ts) + b + np.radians(a.yaw_bias)
    if a.yaw_from_bev:
        # map-consistent yaw where the map itself is rotated (Track1 start: built from the mapping run's drifting
        # departure scans): smoothed BEV yaw over lo..hi, blended back into the RTS yaw over hi..end
        lo, hi, end = [int(v) for v in a.yaw_from_bev_frames.split(':')]
        zq = np.load(a.yaw_from_bev)
        lim = zq['max_yaw'] if 'max_yaw' in zq else np.full(len(zq['idx']), np.abs(np.degrees(zq['dyaw'])).max())
        ok = np.abs(np.degrees(zq['dyaw'])) < lim - 0.01
        qi, qy = zq['idx'][ok], np.unwrap(zq['myaw'][ok])
        o = np.argsort(qi)
        qi, qy = qi[o], qy[o]
        from scipy.ndimage import median_filter
        qy = median_filter(qy, 7, mode='nearest')
        by = np.interp(np.arange(n), qi, qy)
        w = np.clip((end - np.arange(n)) / max(end - hi, 1), 0, 1)
        w[:lo] = 0
        yw = np.unwrap(yw)
        by = by + 2 * np.pi * np.round((yw - by) / (2 * np.pi))
        print('yaw from BEV over frames %d-%d (blend to %d): mean change %.2f deg' % (lo, hi, end, np.degrees(np.mean((by - yw)[:hi]))))
        yw = w * by + (1 - w) * yw
    if a.zupt_hold_yaw and ZUPT_MASK.any():
        # ZIHR: while stationary, keep the heading of travel from just before the stop
        idx = np.flatnonzero(ZUPT_MASK)
        starts = idx[np.r_[True, np.diff(idx) > 1]]
        ends = idx[np.r_[np.diff(idx) > 1, True]]
        for s0, e0 in zip(starts, ends):
            ref_i = s0 - 1 if s0 > 0 else min(e0 + 1, n - 1)
            yw[s0:e0 + 1] = yw[ref_i]
    if a.smooth_xy > 0:
        x = gaussian_filter1d(x, a.smooth_xy, mode='nearest')
        y = gaussian_filter1d(y, a.smooth_xy, mode='nearest')
    if a.smooth_yaw > 0:
        yw = gaussian_filter1d(yw, a.smooth_yaw, mode='nearest')
    if a.start_hold > 0:
        # start segment: slow, straight, IMU heading nearly constant -> constant yaw and a straight path
        imu0 = np.degrees(np.unwrap(s.imu_rot(ts).as_euler('ZYX')[:, 0]))
        tt = (ts - ts[0]) * 1e-9
        spd = np.hypot(np.gradient(x, tt), np.gradient(y, tt))
        moved = (np.abs(imu0 - imu0[0]) > a.start_hold) | (spd > a.start_speed)
        n0 = int(np.argmax(moved)) if moved.any() else n
        if n0 > 30:
            psi0 = np.angle(np.mean(np.exp(1j * yw[:n0])))
            c0, s0 = np.cos(psi0), np.sin(psi0)
            along = c0 * x[:n0] + s0 * y[:n0]
            lat = np.median(-s0 * x[:n0] + c0 * y[:n0])
            # blend back into the free trajectory over 1 s to avoid a step
            w = np.ones(n0)
            nb = min(28, n0)
            w[-nb:] = np.linspace(1, 0, nb)
            xl, yl = c0 * along - s0 * lat, s0 * along + c0 * lat
            x[:n0] = w * xl + (1 - w) * x[:n0]
            y[:n0] = w * yl + (1 - w) * y[:n0]
            yw[:n0] = np.unwrap(w * psi0 + (1 - w) * np.unwrap(yw[:n0] - psi0 + psi0))
            print(f'start hold: frames 0-{n0} ({tt[n0]:.1f} s), yaw fixed at {np.degrees(psi0):.2f} deg')
    if a.time_shift != 0.0:
        tt = (ts - ts[0]) * 1e-9
        x = np.interp(tt + a.time_shift, tt, x)
        y = np.interp(tt + a.time_shift, tt, y)
        yw = np.interp(tt + a.time_shift, tt, yw)
    x = x - a.xy_bias[0]
    y = y - a.xy_bias[1]
    pd.DataFrame({'idx': np.arange(n), 'x': x, 'y': y, 'yaw': wrap(yw)}).to_csv(a.output_csv, index=False)
    print('wrote', a.output_csv, n)


if __name__ == '__main__':
    main()
