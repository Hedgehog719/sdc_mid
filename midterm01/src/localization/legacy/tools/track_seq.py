#!/usr/bin/env python3
"""GNSS-free first pass (Track03): sequential tracking of the vehicle pose against the LiDAR marking map.

Track03 has no GNSS: the batch tools (bev_match / near_fix) need a prior trajectory within about a metre. This tool
produces it from the initial pose alone:
  initial pose: --init x,y,yaw (e.g. from init_search: init_guess.txt refined by a wide lane match), else
                init_guess.txt as is
  per processed frame (every --step frames, in time order):
    predict  heading += IMU heading change (sensors/imu.csv if present, else the estimated yaw rate * dt);
             position += speed * dt along the heading
    measure  near-range image-to-map alignment around the prediction: the map's marking cells 1..--rmax m ahead
             (road level) are projected into the three cameras; score = mean paint response at the marks minus
             --bg_weight x at background road cells; grid over (fwd, lat) +-span, then heading +-yaw_span, then a
             finer (fwd, lat) step (same score as near_fix)
    update   alpha-beta filter in the predicted vehicle frame: forward position + speed, lateral position, heading
             (+ yaw rate without IMU); a peak on the search edge or with a weak contrast is skipped (coast)
Writes the trajectory of every frame (idx, x, y, yaw; linear interpolation between processed frames) and the
accepted measurements (near-fix npz format: idx, ts, mx, my, myaw, sharp_f, sharp_l, ...).
usage: track_seq.py <track_dir> <marks_D.npz> <out.csv> [--init x,y,yaw --step 3]
"""
import argparse
import os
import sys
from multiprocessing import Pool

import cv2
cv2.setNumThreads(1)
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import ImageBEV  # noqa: E402
from common import ply_frames  # noqa: E402
from marking_map import MarkingMap, road_level, vehicle_frame  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('marks')
ap.add_argument('out')
ap.add_argument('--init', default='', help='x,y,yaw_rad initial pose (default: init_guess.txt)')
ap.add_argument('--step', type=int, default=3)
ap.add_argument('--rmax', type=float, default=14.0)
ap.add_argument('--lat', type=float, default=6.0)
ap.add_argument('--span', type=float, default=0.5)
ap.add_argument('--span_start', type=float, default=1.2, help='search span of the first 30 processed frames and after '
                '5 consecutive misses (the initial pose may be ~1 m off)')
ap.add_argument('--yaw_span', type=float, default=1.0, help='deg')
ap.add_argument('--bg_weight', type=float, default=1.0)
ap.add_argument('--min_contrast', type=float, default=0.12)
ap.add_argument('--gain', type=float, nargs=5, default=[0.4, 0.7, 0.5, 0.05, 0.05],
                help='alpha forward, alpha lateral, alpha heading, beta speed, beta yaw rate')
ap.add_argument('--no_imu', action='store_true')
ap.add_argument('--acc_noise', type=float, default=0.05, help='m/s^2: IMU along-track accel noise (speed follows the IMU)')
ap.add_argument('--bias_walk', type=float, default=0.003, help='m/s^2/sqrt(s)')
ap.add_argument('--v0_sd', type=float, default=0.3, help='m/s: initial speed uncertainty (initial speed 0)')
ap.add_argument('--min_sharp_f', type=float, default=0.02, help='forward correction only for a distinct forward peak')
ap.add_argument('--vo', default='', help='bev_vo.py npz: road-surface odometry (speed measurement, scale estimated)')
ap.add_argument('--floor', type=float, nargs=3, default=[0.3, 0.15, 0.3], help='sd floors: forward m, lateral m, heading deg')
ap.add_argument('--min_peak', type=float, default=0.2)
ap.add_argument('--min_peak_yaw', type=float, default=0.25)
ap.add_argument('--min_sharp_l', type=float, default=0.1)
ap.add_argument('--gyro_bias', type=float, default=-0.23, help='deg/s initial gyro bias (Track01/02: -0.22/-0.25)')
ap.add_argument('--gyro_bias_sd', type=float, default=0.02)
ap.add_argument('--gyro_bias_walk', type=float, default=0.0005, help='deg/s/sqrt(s)')
ap.add_argument('--gyro_noise', type=float, default=0.1, help='deg/sqrt(s) heading random walk of the bias-corrected gyro')
ap.add_argument('--yaw_sigma', type=float, default=0.6, help='deg: lane-match heading noise (IMU heading is precise short-term)')
ap.add_argument('--near_too', action='store_true', help='with --matcher lane: also near-range alignment for lateral / heading')
ap.add_argument('--max_window', type=float, default=6.0, help='m: largest lane-match shift window')
ap.add_argument('--speed_gain', type=float, default=0.0, help='fraction of the Kalman speed gain applied (0 = speed from the IMU)')
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--frames', default='', help='lo:hi (processed range, default all)')
ap.add_argument('--matcher', choices=['lane', 'near'], default='lane', help='lane: single-frame BEV lane match (bev_match, +-2.5 m window, distinct forward peaks); near: near-range paint alignment')
ap.add_argument('--lane_marks', default='', help='soft marking map (marking_map --mode soft) for --matcher lane')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TS, _ = ply_frames(os.path.join(a.track_dir, 'raw_ply'))
TS = np.asarray(TS, np.int64)
MK = MarkingMap(a.marks)
B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
C = B.C
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
CAMS = ('fl', 'f', 'fr')
RD = {c: np.load(os.path.join(a.track_dir, 'road', f'{c}.npz'))['road'] for c in CAMS}


def response(cam, j):
    img = cv2.imread(os.path.join(a.track_dir, f'{T}_images', f'{T}_{cam}_images', B.names[cam][j]), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    med = cv2.resize(cv2.medianBlur(small, 15), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR).astype(np.float32)
    r = np.clip((img.astype(np.float32) - med - 15.0) / 30.0, 0, 1)
    rp = cv2.resize(RD[cam][j].astype(np.float32) / 255.0, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    r *= np.clip(rp * 1.5, 0, 1)
    bad = B.ego_bad(cam, img.shape).copy()
    if cam in B.masks and j < len(B.masks[cam]):
        m = np.unpackbits(B.masks[cam][j], axis=1)[:, :img.shape[1] // 2]
        bad |= cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    r[cv2.dilate(bad, np.ones((9, 9), np.uint8)) > 0] = 0
    return cv2.GaussianBlur(r, (0, 0), 2.0)


def responses(j):
    return j, {c: response(c, j) for c in CAMS}


def score(q, resp):
    vals = []
    for c in CAMS:
        uv, ok = C.project(c, q @ Rm)
        h, w = resp[c].shape
        u, v = uv[:, 0], uv[:, 1]
        ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        if ins.any():
            vals.append(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1),
                                  cv2.INTER_LINEAR).ravel())
    v = np.concatenate(vals) if vals else np.zeros(0)
    return float(v.mean()) if len(v) > 30 else np.nan


def measure(j, pose, resp, span_f, span_l):
    """Correction (dfwd, dlat, dyaw) of pose for frame j with per-axis validity and peak sharpness; None if unusable."""
    x, y, yaw = pose
    P = MK.points(x, y, a.rmax + 6)
    if len(P) < 60:
        return None
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > 1.0) & (q[:, 0] < a.rmax) & (np.abs(q[:, 1]) < a.lat)]
    if len(q) < 60:
        return None
    if len(q) > 2500:
        q = q[np.random.default_rng(j).choice(len(q), 2500, replace=False)]
    qb = None
    if a.bg_weight > 0:
        Bw = MK.background(x, y, a.rmax + 6, 2500, np.random.default_rng(j))
        if len(Bw):
            qb = vehicle_frame(Bw, (x, y, yaw), MK.height)
            qb = qb[road_level(qb, tol=0.15, slope=0.02) & (qb[:, 0] > 1.0) & (qb[:, 0] < a.rmax) & (np.abs(qb[:, 1]) < a.lat)]
            if len(qb) < 60:
                qb = None

    def sc(df, dl, rot=0.0):
        def tf(p):
            p = p - [df, dl, 0]
            if rot:
                ct, st_ = np.cos(-rot), np.sin(-rot)
                p = np.c_[ct * p[:, 0] - st_ * p[:, 1], st_ * p[:, 0] + ct * p[:, 1], p[:, 2]]
            return p
        v = score(tf(q), resp)
        if qb is not None and np.isfinite(v):
            b = score(tf(qb), resp)
            v = v - a.bg_weight * (b if np.isfinite(b) else 0.0)
        return v

    sf_ = 0.1 if span_f <= 0.8 else 0.2
    fo = np.round(np.arange(-span_f, span_f + 1e-9, sf_), 3)
    lo_ = np.round(np.arange(-span_l, span_l + 1e-9, 0.1), 3)
    g = np.array([[sc(df, dl) for dl in lo_] for df in fo])
    if not np.isfinite(g).any():
        return None
    g = np.nan_to_num(g, nan=-1.0)
    i, k = np.unravel_index(np.argmax(g), g.shape)
    peak = g[i, k]
    contrast = (peak - np.median(g)) / max(abs(peak), 1e-6)
    if contrast < a.min_contrast:
        return None
    f_ok = 0 < i < len(fo) - 1
    l_ok = 0 < k < len(lo_) - 1
    df, dl = fo[i], lo_[k]
    ys = np.radians(np.arange(-a.yaw_span, a.yaw_span + 1e-9, 0.2))
    gy = np.nan_to_num(np.array([sc(df, dl, th) for th in ys]), nan=-1.0)
    m = int(np.argmax(gy))
    dyaw, y_ok = 0.0, 0 < m < len(ys) - 1
    if y_ok:
        den = gy[m - 1] - 2 * gy[m] + gy[m + 1]
        dyaw = float(ys[m] + (np.clip(0.5 * (gy[m - 1] - gy[m + 1]) / den, -0.5, 0.5) if den < 0 else 0) * np.radians(0.2))
    fine = np.array([-0.05, 0.0, 0.05])
    gf = np.nan_to_num(np.array([[sc(df + u, dl + w, dyaw) for w in fine] for u in fine]), nan=-1.0)

    def para(v):
        den = v[0] - 2 * v[1] + v[2]
        return float(np.clip(0.5 * (v[0] - v[2]) / den, -1, 1)) * 0.05 if den < 0 else 0.0
    i2, k2 = np.unravel_index(np.argmax(gf), gf.shape)
    df, dl = df + fine[i2] + (para(gf[:, 1]) if i2 == 1 else 0.0), dl + fine[k2] + (para(gf[1, :]) if k2 == 1 else 0.0)
    n3f = max(1, int(round(0.3 / sf_)))
    sf = peak - max(g[max(i - n3f, 0), k], g[min(i + n3f, len(fo) - 1), k])
    sl = peak - max(g[i, max(k - 3, 0)], g[i, min(k + 3, len(lo_) - 1)])
    return df, dl, dyaw, sf, sl, contrast, peak, f_ok, l_ok, y_ok


def lane_measure(j, pose, win=2.5):
    try:
        r = LS.match(j, pose)
    except (IndexError, ValueError):   # no map around the pose (map edge)
        return None
    if r is None:
        return None
    peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l = r
    lim = win - 0.15
    # quality gates (weak / ambiguous matches at intersections and bends lock onto wrong lines)
    f_ok = abs(fwd) < lim and sharp_f > 0.003 and peak > a.min_peak
    l_ok = abs(lat) < lim and sharp_l > a.min_sharp_l and peak > a.min_peak
    y_ok = abs(np.degrees(dyaw)) < 3.0 - 0.05 and peak > a.min_peak_yaw
    if peak < 0.08:
        return None
    return fwd, lat, dyaw, sharp_f, sharp_l, sharp, peak, f_ok, l_ok, y_ok


def imu_heading():
    """Heading change from the gyro (angular rate rotated to the world frame, z component, integrated); the IMU's
    own orientation yaw drifts by several degrees (Track01 / 02), the gyro has a near-constant bias (~-0.23 deg/s)
    that the tracker estimates."""
    f = os.path.join(a.track_dir, 'sensors', 'imu.csv')
    if a.no_imu or not os.path.exists(f):
        return None
    im = pd.read_csv(f)
    q = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
    wz = q.apply(im[['wx', 'wy', 'wz']].values)[:, 2]
    t = im.t.values.astype(np.float64)
    yaw = np.concatenate([[0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(t) * 1e-9)])
    return lambda tq: np.interp(np.asarray(tq, np.float64), t, yaw)


def imu_accel():
    f = os.path.join(a.track_dir, 'sensors', 'imu.csv')
    if a.no_imu or not os.path.exists(f):
        return None
    im = pd.read_csv(f)
    q = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
    aw = q.apply(im[['ax', 'ay', 'az']].values)
    yaw = np.unwrap(q.as_euler('ZYX')[:, 0])
    along = np.cos(yaw) * aw[:, 0] + np.sin(yaw) * aw[:, 1]
    t = im.t.values.astype(np.float64)
    ca = np.concatenate([[0], np.cumsum(0.5 * (along[1:] + along[:-1]) * np.diff(t) * 1e-9)])   # integral of accel
    return lambda t0, t1: (np.interp(t1, t, ca) - np.interp(t0, t, ca))   # velocity change over [t0, t1]


def initial_pose():
    if a.init:
        return [float(v) for v in a.init.split(',')]
    M = np.loadtxt(os.path.join(a.track_dir, 'init_guess.txt'), delimiter=',')
    yaw = Rotation.from_matrix(M[:3, :3]).as_euler('ZYX')[0]
    return [M[0, 3], M[1, 3], yaw]


if __name__ == '__main__':
    n = len(TS)
    lo, hi = (int(v) for v in a.frames.split(':')) if a.frames else (0, n)
    frames = list(range(lo, hi, a.step))
    imu = imu_heading()
    dvel = imu_accel()
    print(f'{T}: {n} frames, processing {len(frames)} (step {a.step}), IMU heading: {"yes" if imu else "no"}, matcher {a.matcher}', flush=True)
    if a.matcher == 'lane':
        global LS
        import lane_single as LS
        LS.setup(a.track_dir, a.lane_marks, yaw_mount=a.yaw_mount)
    x, y, yaw = initial_pose()
    v, b, w = 0.0, 0.0, 0.0
    P3 = np.diag([1.0, a.v0_sd, 0.05, 0.05]) ** 2   # along-track [position, speed, accel bias, VO scale]
    kvo = 1.0
    VO = None
    if a.vo:
        zv = np.load(a.vo)
        VO = (zv['j'], zv['fwd'], zv['dt'], zv['resp'])
    Pl, Py = 1.0 ** 2, np.radians(2.0) ** 2      # cross-track position, heading
    bg = np.radians(a.gyro_bias)                  # gyro bias (rad/s), estimated
    PY = np.diag([Py, np.radians(a.gyro_bias_sd) ** 2])
    sa = a.acc_noise if dvel else 1.5            # m/s^2 white accel noise (IMU error / unmodelled)
    sb = a.bias_walk                             # m/s^2/sqrt(s) accel bias walk
    qy = np.radians(a.gyro_noise) ** 2 if imu else np.radians(1.0) ** 2   # rad^2/s heading walk
    poses, meas = [], []
    big_f, big_l = [], []
    t_prev = None
    j_prev = None
    nvo = 0
    nf = nl = ny = 0
    block = 4 * a.workers
    with Pool(a.workers) as pool:
        for b0 in range(0, len(frames), block):
            it = pool.imap(responses, frames[b0:b0 + block]) if (a.matcher == 'near' or a.near_too) else ((j_, None) for j_ in frames[b0:b0 + block])
            for j, resp in it:
                t = TS[j] * 1e-9
                if t_prev is not None:
                    dt = t - t_prev
                    dyaw_p = (imu(TS[j]) - imu(int(t_prev * 1e9)) - bg * dt) if imu else w * dt
                    dv = (dvel(t_prev * 1e9, TS[j] * 1.0) if dvel else 0.0) - b * dt
                    v_new = max(0.0, v + dv)
                    ym = yaw + 0.5 * dyaw_p
                    d = 0.5 * (v + v_new) * dt
                    x, y = x + d * np.cos(ym), y + d * np.sin(ym)
                    yaw, v = yaw + dyaw_p, v_new
                    F = np.array([[1, dt, -dt * dt / 2, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    G = np.array([dt * dt / 2, dt, 0.0, 0.0])
                    P3 = F @ P3 @ F.T + sa ** 2 * np.outer(G, G) + np.diag([1e-4 * dt, 0, sb ** 2 * dt, 1e-6 * dt])
                    if VO is not None:   # VO speed over the pairs since the previous processed frame: z = kvo * v
                        mm = (VO[0] >= j_prev) & (VO[0] < j) & (VO[3] > 0.5)
                        if mm.sum() >= max(1, (j - j_prev) // 2):
                            zv_ = VO[1][mm].sum() / max(VO[2][mm].sum(), 1e-3)
                            Hv = np.array([0, kvo, 0, v])
                            Rv = (0.04 * max(v, 1.0)) ** 2
                            Sv = Hv @ P3 @ Hv + Rv
                            inn = zv_ - kvo * v
                            if inn * inn < 16 * Sv:
                                K = P3 @ Hv / Sv
                                c0_, s0_ = np.cos(yaw), np.sin(yaw)
                                x, y = x + c0_ * K[0] * inn, y + s0_ * K[0] * inn
                                v, b, kvo = max(0.0, v + K[1] * inn), b + K[2] * inn, float(np.clip(kvo + K[3] * inn, 0.7, 1.3))
                                P3 = P3 - np.outer(K, Hv @ P3)
                                nvo += 1
                    Pl = Pl + (0.05 ** 2) * dt + (d ** 2) * Py
                    if imu:
                        Fy = np.array([[1.0, -dt], [0.0, 1.0]])
                        PY = Fy @ PY @ Fy.T + np.diag([qy * dt, np.radians(a.gyro_bias_walk) ** 2 * dt])
                        Py = PY[0, 0]
                    else:
                        Py = Py + qy * dt
                if a.matcher == 'lane':   # search window follows the forward uncertainty (re-acquire after long gaps)
                    win = float(np.clip(3 * np.sqrt(P3[0, 0]), 2.5, a.max_window))
                    LS.set_window(max_shift=win)
                    r = lane_measure(j, (x, y, yaw), win)
                else:
                    span_f = float(np.clip(3 * np.sqrt(P3[0, 0]), 0.4, 2.0))
                    span_l = float(np.clip(3 * np.sqrt(Pl), 0.4, 1.2))
                    r = measure(j, (x, y, yaw), resp, span_f, span_l)
                if r is not None:
                    df, dl, dyaw, sf, sl, contrast, peak, f_ok, l_ok, y_ok = r
                    c, s_ = np.cos(yaw), np.sin(yaw)
                    meas.append((j, TS[j], x + c * df - s_ * dl, y + s_ * df + c * dl, yaw + dyaw, df, dl, dyaw, sf, sl,
                                 contrast, peak, f_ok, l_ok, y_ok))
                    if f_ok and sf > a.min_sharp_f:
                        sig = float(np.clip((0.01 if a.matcher == 'lane' else 0.006) / sf, 0.2 if a.matcher == 'lane' else 0.15, 3.0))
                        S = P3[0, 0] + sig ** 2
                        big_f.append(np.sign(df) if df * df > 6.25 * S else 0)
                        if len(big_f) >= 4 and abs(sum(big_f[-4:])) == 4:   # 4 consistent large corrections: we drifted
                            P3[0, 0] += df * df
                            S = P3[0, 0] + sig ** 2
                            big_f.clear()
                        if df * df < 16 * S:
                            K = P3[:, 0] / S
                            if VO is None and a.speed_gain <= 0:
                                K = K * np.array([1, 0, 0, 0])     # no VO: forward fixes correct position only
                            x, y = x + c * K[0] * df, y + s_ * K[0] * df
                            v, b, kvo = max(0.0, v + K[1] * df), b + K[2] * df, float(np.clip(kvo + K[3] * df, 0.7, 1.3))
                            P3 = P3 - np.outer(K, P3[0, :])
                            nf += 1
                    if l_ok and sl > 0:
                        sig = float(np.clip((0.01 if a.matcher == 'lane' else 0.006) / sl, 0.1 if a.matcher == 'lane' else 0.08, 1.0))
                        S = Pl + sig ** 2
                        big_l.append(np.sign(dl) if (dl * dl > 6.25 * S and sl > 0.15) else 0)
                        if len(big_l) >= 4 and abs(sum(big_l[-4:])) == 4:
                            Pl += dl * dl
                            S = Pl + sig ** 2
                            big_l.clear()
                        if dl * dl < 16 * S or len(poses) < 30:
                            K = Pl / S
                            x, y = x - s_ * K * dl, y + c * K * dl
                            Pl = (1 - K) * Pl
                            nl += 1
                    if y_ok:
                        S = Py + np.radians(a.yaw_sigma if a.matcher == 'lane' else 0.3) ** 2
                        if dyaw * dyaw < 16 * S or len(poses) < 30:
                            if imu:   # 2-state update: heading + gyro bias
                                Ky = PY[:, 0] / (PY[0, 0] + S - Py)
                                yaw, bg = yaw + Ky[0] * dyaw, bg + Ky[1] * dyaw
                                PY = PY - np.outer(Ky, PY[0, :])
                                Py = PY[0, 0]
                            else:
                                K = Py / S
                                yaw = yaw + K * dyaw
                                if t_prev is not None:
                                    w = w + 0.1 * K * dyaw / max(dt, 1e-3)
                                Py = (1 - K) * Py
                            ny += 1
                if a.matcher == 'lane' and a.near_too and resp is not None:
                    rn = measure(j, (x, y, yaw), resp, 0.4, float(np.clip(3 * np.sqrt(Pl), 0.4, 1.0)))
                    if rn is not None:
                        _, dl2, dyaw2, _, sl2, _, _, _, l_ok2, y_ok2 = rn
                        c, s_ = np.cos(yaw), np.sin(yaw)
                        if l_ok2 and sl2 > 0:
                            sig = float(np.clip(0.006 / sl2, 0.08, 0.6))
                            S = Pl + sig ** 2
                            if dl2 * dl2 < 16 * S:
                                K = Pl / S
                                x, y = x - s_ * K * dl2, y + c * K * dl2
                                Pl = (1 - K) * Pl
                                nl += 1
                        if y_ok2 and imu:
                            S = PY[0, 0] + np.radians(0.6) ** 2
                            if dyaw2 * dyaw2 < 16 * S:
                                Ky = PY[:, 0] / S
                                yaw, bg = yaw + Ky[0] * dyaw2, bg + Ky[1] * dyaw2
                                PY = PY - np.outer(Ky, PY[0, :])
                                Py = PY[0, 0]
                P3[0, 0] = max(P3[0, 0], a.floor[0] ** 2)        # never overconfident: map fixes can be wrong,
                Pl = max(Pl, a.floor[1] ** 2)                    # drift between fixes is not fully modelled
                if imu:
                    PY[0, 0] = max(PY[0, 0], np.radians(a.floor[2]) ** 2)
                    Py = PY[0, 0]
                else:
                    Py = max(Py, np.radians(a.floor[2]) ** 2)
                poses.append((j, x, y, yaw, v))
                t_prev = t
                j_prev = j
            if (b0 // block) % 10 == 0:
                print(f'  frame {poses[-1][0]}: x {x:.2f} y {y:.2f} yaw {np.degrees(yaw):.2f} v {v:.2f} m/s b {b:+.3f} | '
                      f'sd fwd {np.sqrt(P3[0, 0]):.2f} lat {np.sqrt(Pl):.2f} kvo {kvo:.3f} bg {np.degrees(bg):+.3f} | accepted fwd {nf} lat {nl} yaw {ny} vo {nvo} of {len(poses)}', flush=True)
    P = np.array(poses)
    allf = np.arange(n)
    out = pd.DataFrame({'idx': allf, 'x': np.interp(allf, P[:, 0], P[:, 1]), 'y': np.interp(allf, P[:, 0], P[:, 2]),
                        'yaw': np.interp(allf, P[:, 0], np.unwrap(P[:, 3]))})
    out.to_csv(a.out, index=False)
    M_ = np.array(meas)
    np.savez(a.out.replace('.csv', '_meas.npz'), idx=M_[:, 0].astype(int), ts=M_[:, 1].astype(np.int64),
             ts_i=M_[:, 1].astype(np.int64), mx=M_[:, 2], my=M_[:, 3], myaw=M_[:, 4], fwd=M_[:, 5], lat=M_[:, 6],
             dyaw=M_[:, 7], sharp_f=M_[:, 8], sharp_l=M_[:, 9], sharp_y=M_[:, 14], peak=M_[:, 11], contrast=M_[:, 10],
             f_ok=M_[:, 12], l_ok=M_[:, 13], px=np.zeros(len(M_)), py=np.zeros(len(M_)), pyaw=np.zeros(len(M_)),
             base=np.zeros(len(M_)))
    print(f'{T}: accepted forward {nf}, lateral {nl}, heading {ny} of {len(poses)} -> {a.out}')
