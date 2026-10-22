#!/usr/bin/env python3
"""Online (causal) localization against the LiDAR map: one Kalman filter run in time order.

The pose of frame k is estimated from data up to frame k only and written once (never revised later): no backward
smoothing, no use of future frames, no offline trajectory correction (competition rules 10/22).

Per frame k (time order):
  k = 0      initial pose: init_guess.txt refined by a wide single-frame lane match of frame 0 (heading +-25 deg,
             position +-1 m; the given init_guess can be 18 deg off), then a narrow one (+-3 deg, +-1 m)
  predict    heading: gyro rate (world z) minus an estimated gyro bias; speed: IMU along-track acceleration minus an
             estimated accel bias; position: speed * dt along the heading
  VO         road-surface odometry of the frame pair (k-1, k) (bev_vo.py) as a speed measurement (scale estimated)
  GNSS       (Track01/02, --gnss) samples received in (t_k-1, t_k], propagated to t_k, as position measurements
  map        every --step frames: single-frame BEV lane match (bev_match.match_frame, no accumulation) and near-range
             image alignment (marking cells 1..14 m ahead projected into the 3 cameras) -> forward / lateral /
             heading corrections with sharpness-based noise, match-quality gates and innovation gates
  output     the posterior pose of frame k
Calibration (not trajectory information): camera rotation / translation corrections (SDC_ROT_FIX json), camera mount
yaw, ego-vehicle masks, map-derived marking maps built without any trajectory (marking_map.py ALL).
usage: localize_online.py <track_dir> <marks_D.npz> <marks_S.npz> <out.csv> [--vo vo.npz] [--gnss] [--no_imu]
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
import lane_single as LS  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('track_dir')
ap.add_argument('marks', help='marking map (marking_map.py ALL ... --hi 2.5 --lo 1.2 --cell 0.7)')
ap.add_argument('lane_marks', help='soft marking map (marking_map.py ALL ... --mode soft)')
ap.add_argument('out')
ap.add_argument('--vo', default='', help='bev_vo.py npz (road-surface odometry)')
ap.add_argument('--gnss', action='store_true', help='use sensors/gnss.csv (Track01/02)')
ap.add_argument('--gnss_sigma', type=float, nargs=2, default=[0.3, 0.3], help='m: along / across-track GNSS noise')
ap.add_argument('--no_imu', action='store_true')
ap.add_argument('--step', type=int, default=3, help='map measurement every step frames')
ap.add_argument('--init', default='', help='x,y,yaw_rad instead of init_guess.txt')
ap.add_argument('--init_yaw', type=float, default=25.0, help='deg: heading window of the initial search')
ap.add_argument('--rmax', type=float, default=14.0)
ap.add_argument('--lat', type=float, default=6.0)
ap.add_argument('--bg_weight', type=float, default=1.0)
ap.add_argument('--min_contrast', type=float, default=0.12)
ap.add_argument('--acc_noise', type=float, default=0.05)
ap.add_argument('--bias_walk', type=float, default=0.003)
ap.add_argument('--v0_sd', type=float, default=0.3)
ap.add_argument('--min_sharp_f', type=float, default=0.02)
ap.add_argument('--floor', type=float, nargs=3, default=[0.3, 0.15, 0.3], help='sd floors: forward m, lateral m, heading deg')
ap.add_argument('--min_peak', type=float, default=0.2)
ap.add_argument('--min_peak_yaw', type=float, default=0.25)
ap.add_argument('--min_sharp_l', type=float, default=0.1)
ap.add_argument('--gyro_bias', type=float, default=-0.23)
ap.add_argument('--gyro_bias_sd', type=float, default=0.003, help='deg/s: the gyro bias is a sensor calibration constant')
ap.add_argument('--gyro_bias_walk', type=float, default=0.0001)
ap.add_argument('--gyro_noise', type=float, default=0.1)
ap.add_argument('--yaw_sigma', type=float, default=0.6)
ap.add_argument('--max_window', type=float, default=6.0)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=3)
ap.add_argument('--record', default='', help='save the per-frame map measurements (absolute poses) for --replay')
ap.add_argument('--replay', default='', help='run the filter on recorded map measurements (fast parameter studies)')
ap.add_argument('--lane_yaw_off', type=float, default=0.25, help='deg: heading bias of the lane-match headings (calibration, '
                'from the course of travel: T1 +0.25, T2 +0.25)')
ap.add_argument('--near_yaw_off', type=float, default=0.2, help='deg: heading bias of the near-alignment headings (T1 +0.26, T2 +0.15)')
ap.add_argument('--gross_yaw', type=float, default=5.0, help='deg: the frame-0 search corrects init_guess only above this')
ap.add_argument('--start_yaw_sigma', type=float, default=1.5, help='deg: front-camera near heading noise in the start segment')
ap.add_argument('--start_dist', type=float, default=15.0, help='m travelled (so far) below which lane matches give lateral only')
a = ap.parse_args()

T = os.path.basename(os.path.normpath(a.track_dir))
TS = np.asarray(ply_frames(os.path.join(a.track_dir, 'raw_ply'))[0], np.int64)
CAMS = ('fl', 'f', 'fr')
Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
if not a.replay:
    MK = MarkingMap(a.marks)
    B = ImageBEV(a.track_dir, 1.5, 0, 0, a.yaw_mount)
    C = B.C
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


def responses(j):   # image preprocessing of frame j (parallel prefetch; the estimate of frame j uses it at frame j)
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


def near_measure(j, pose, resp, span_l):
    """Near-range image alignment of frame j: (dlat, dyaw, sharp_l, lateral ok, heading ok) or None."""
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

    fo = np.round(np.arange(-0.4, 0.4001, 0.1), 3)
    lo_ = np.round(np.arange(-span_l, span_l + 1e-9, 0.1), 3)
    g = np.nan_to_num(np.array([[sc(df, dl) for dl in lo_] for df in fo]), nan=-1.0)
    i, k = np.unravel_index(np.argmax(g), g.shape)
    peak = g[i, k]
    if (peak - np.median(g)) / max(abs(peak), 1e-6) < a.min_contrast:
        return None
    l_ok = 0 < k < len(lo_) - 1
    df, dl = fo[i], lo_[k]
    ys = np.radians(np.arange(-1.0, 1.0001, 0.2))
    gy = np.nan_to_num(np.array([sc(df, dl, th) for th in ys]), nan=-1.0)
    m = int(np.argmax(gy))
    dyaw, y_ok = 0.0, 0 < m < len(ys) - 1
    if y_ok:
        den = gy[m - 1] - 2 * gy[m] + gy[m + 1]
        dyaw = float(ys[m] + (np.clip(0.5 * (gy[m - 1] - gy[m + 1]) / den, -0.5, 0.5) if den < 0 else 0) * np.radians(0.2))
    gl = np.nan_to_num(np.array([sc(df, dl + w, dyaw) for w in (-0.05, 0.0, 0.05)]), nan=-1.0)
    den = gl[0] - 2 * gl[1] + gl[2]
    if den < 0:
        dl += float(np.clip(0.5 * (gl[0] - gl[2]) / den, -1, 1)) * 0.05
    sl = peak - max(g[i, max(k - 3, 0)], g[i, min(k + 3, len(lo_) - 1)])
    return dl, dyaw, sl, l_ok, y_ok


def near_yaw(j, pose, cams=('f',), span=3.0, step=0.2):
    """Heading correction of pose from the near-range markings in the given cameras (front: the side cameras see
    parked cars / kerbs at the start); returns (dyaw, ok)."""
    x, y, yaw = pose
    P = MK.points(x, y, a.rmax + 6)
    if len(P) < 60:
        return 0.0, False
    q = vehicle_frame(P, (x, y, yaw), MK.height)
    q = q[road_level(q) & (q[:, 0] > 1.0) & (q[:, 0] < a.rmax) & (np.abs(q[:, 1]) < a.lat)]
    if len(q) < 60:
        return 0.0, False
    resp = {c: response(c, j) for c in cams}

    def sc(rot):
        ct, st_ = np.cos(-rot), np.sin(-rot)
        p = np.c_[ct * q[:, 0] - st_ * q[:, 1], st_ * q[:, 0] + ct * q[:, 1], q[:, 2]]
        vals = []
        for c in cams:
            uv, ok = C.project(c, p @ Rm)
            h, w = resp[c].shape
            u, v = uv[:, 0], uv[:, 1]
            ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
            if ins.sum() > 30:
                vals.append(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).ravel())
        return float(np.concatenate(vals).mean()) if vals else np.nan
    ys = np.radians(np.arange(-span, span + 1e-9, step))
    g = np.nan_to_num(np.array([sc(t) for t in ys]), nan=-1.0)
    m = int(np.argmax(g))
    if not 0 < m < len(ys) - 1 or g[m] <= 0:
        return 0.0, False
    den = g[m - 1] - 2 * g[m] + g[m + 1]
    return float(ys[m] + (np.clip(0.5 * (g[m - 1] - g[m + 1]) / den, -0.5, 0.5) if den < 0 else 0) * np.radians(step)), True


def lane_measure(j, pose, win):
    try:
        r = LS.match(j, pose)
    except (IndexError, ValueError):   # no map around the pose (map edge)
        return None
    if r is None:
        return None
    peak, sharp, fwd, lat, dyaw, sharp_f, sharp_l = r
    if peak < 0.08:
        return None
    lim = win - 0.15
    f_ok = abs(fwd) < lim and sharp_f > 0.003 and peak > a.min_peak
    l_ok = abs(lat) < lim and sharp_l > a.min_sharp_l and peak > a.min_peak
    y_ok = abs(np.degrees(dyaw)) < 3.0 - 0.05 and peak > a.min_peak_yaw
    return fwd, lat, dyaw, sharp_f, sharp_l, peak, f_ok, l_ok, y_ok


def load_imu():
    f = os.path.join(a.track_dir, 'sensors', 'imu.csv')
    if a.no_imu or not os.path.exists(f) or os.path.getsize(f) < 200:
        return None, None
    im = pd.read_csv(f)
    q = Rotation.from_quat(im[['qx', 'qy', 'qz', 'qw']].values)
    t = im.t.values.astype(np.float64)
    wz = q.apply(im[['wx', 'wy', 'wz']].values)[:, 2]
    yaw_g = np.concatenate([[0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(t) * 1e-9)])
    aw = q.apply(im[['ax', 'ay', 'az']].values)
    yq = np.unwrap(q.as_euler('ZYX')[:, 0])
    along = np.cos(yq) * aw[:, 0] + np.sin(yq) * aw[:, 1]
    ca = np.concatenate([[0], np.cumsum(0.5 * (along[1:] + along[:-1]) * np.diff(t) * 1e-9)])
    heading = lambda t0, t1: float(np.interp(t1, t, yaw_g) - np.interp(t0, t, yaw_g))   # noqa: E731
    dvel = lambda t0, t1: float(np.interp(t1, t, ca) - np.interp(t0, t, ca))            # noqa: E731
    return heading, dvel


def load_gnss():
    f = os.path.join(a.track_dir, 'sensors', 'gnss.csv')
    if not a.gnss or not os.path.exists(f):
        return None
    g = pd.read_csv(f)
    return g.t.values.astype(np.int64), g.x.values, g.y.values


def initial_pose(G):
    """Frame 0 only (causal). Position: the latest GNSS sample at frame 0 (Track01/02) or init_guess.txt; heading:
    init_guess refined by wide then narrow single-frame lane matches (only their heading is used: at the start the
    map can be defective and the position estimate of one frame is unreliable; the filter corrects the position)."""
    if a.init:
        return [float(v) for v in a.init.split(',')]
    M = np.loadtxt(os.path.join(a.track_dir, 'init_guess.txt'), delimiter=',')
    x, y, yaw = M[0, 3], M[1, 3], Rotation.from_matrix(M[:3, :3]).as_euler('ZYX')[0]
    if G is not None:
        k = int(np.searchsorted(G[0], TS[0], side='right')) - 1
        if k >= 0 and (TS[0] - G[0][k]) * 1e-9 < 0.5:
            x, y = G[1][k], G[2][k]
    y0 = yaw
    for my, ms, ys in ((a.init_yaw, 1.0, 1.0), (3.0, 1.0, 0.5)):
        LS.set_window(max_shift=ms, max_yaw=my, yaw_step=ys)
        try:
            r = LS.match(0, (x, y, yaw))
        except (IndexError, ValueError):
            r = None
        if r is not None and r[0] > 0.08:
            yaw = yaw + r[4] - np.radians(a.lane_yaw_off)
    LS.set_window(max_shift=2.5, max_yaw=3.0, yaw_step=0.5)
    if abs(np.degrees(yaw - y0)) < a.gross_yaw:   # init_guess heading is fine: keep it (the start map can be defective)
        yaw = y0
    return [x, y, yaw]


if __name__ == '__main__':
    n = len(TS)
    REC = {'lane': {}, 'near': {}, 'front': {}}
    REPLAY = None
    if a.replay:
        zr = np.load(a.replay, allow_pickle=True)
        REPLAY = zr['rec'].item()
    else:
        LS.setup(a.track_dir, a.lane_marks, yaw_mount=a.yaw_mount)
    heading, dvel = load_imu()
    G = load_gnss()
    VO = None
    if a.vo:
        zv = np.load(a.vo)
        VO = {int(j): (f, d, r) for j, f, d, r in zip(zv['j'], zv['fwd'], zv['dt'], zv['resp'])}   # pair (j, j + 1)
    print(f'{T}: {n} frames, IMU {"yes" if heading else "no"}, GNSS {"yes" if G is not None else "no"}, VO {"yes" if VO else "no"}', flush=True)
    x, y, yaw = REPLAY['init'] if REPLAY is not None else initial_pose(G)
    REC['init'] = (x, y, yaw)
    print(f'initial pose (frame 0): x {x:.2f} y {y:.2f} yaw {np.degrees(yaw):.2f} deg', flush=True)
    v, b, w, kvo = 0.0, 0.0, 0.0, 1.0
    P3 = np.diag([1.0, a.v0_sd, 0.05, 0.05]) ** 2      # along-track [position, speed, accel bias, VO scale]
    Pl = 1.0 ** 2                                       # cross-track position
    bg = np.radians(a.gyro_bias)
    PY = np.diag([np.radians(2.0), np.radians(a.gyro_bias_sd)]) ** 2   # [heading, gyro bias]
    sa = a.acc_noise if dvel else 1.5
    qy = np.radians(a.gyro_noise) ** 2 if heading else np.radians(1.0) ** 2
    out = np.zeros((n, 3))
    big_f, big_l = [], []
    travelled = 0.0
    cnt = dict(fwd=0, lat=0, yaw=0, vo=0, gnss=0)
    gi = 0
    if G is not None:
        gi = int(np.searchsorted(G[0], TS[0], side='right'))   # samples up to frame 0 are not used afterwards

    def upd_fwd(e, sig, gate=16.0):
        global x, y, v, b, kvo, P3
        S = P3[0, 0] + sig ** 2
        if e * e >= gate * S:
            return False
        K = P3[:, 0] / S
        c, s = np.cos(yaw), np.sin(yaw)
        x, y = x + c * K[0] * e, y + s * K[0] * e
        v, b, kvo = max(0.0, v + K[1] * e), b + K[2] * e, float(np.clip(kvo + K[3] * e, 0.7, 1.3))
        P3 = P3 - np.outer(K, P3[0, :])
        return True

    def upd_lat(e, sig, gate=16.0):
        global x, y, Pl
        S = Pl + sig ** 2
        if e * e >= gate * S:
            return False
        K = Pl / S
        c, s = np.cos(yaw), np.sin(yaw)
        x, y = x - s * K * e, y + c * K * e
        Pl = (1 - K) * Pl
        return True

    def upd_yaw(e, sig, gate=16.0):
        global yaw, bg, PY, w
        S = PY[0, 0] + sig ** 2
        if e * e >= gate * S:
            return False
        K = PY[:, 0] / S
        yaw, bg = yaw + K[0] * e, bg + (K[1] * e if heading else 0.0)
        if not heading:
            w = w + 0.1 * K[0] * e / 0.1
        PY = PY - np.outer(K, PY[0, :])
        return True

    meas_frames = list(range(a.step, n, a.step))
    block = 4 * a.workers
    mi = 0
    with Pool(a.workers) as pool:
        pending = iter(())
        resp_cache = {}
        for k in range(n):
            if k > 0:
                t0, t1 = TS[k - 1] * 1.0, TS[k] * 1.0
                dt = (t1 - t0) * 1e-9
                dyaw_p = (heading(t0, t1) - bg * dt) if heading else w * dt
                v_new = max(0.0, v + ((dvel(t0, t1) if dvel else 0.0) - b * dt))
                ym = yaw + 0.5 * dyaw_p
                d = 0.5 * (v + v_new) * dt
                x, y, yaw, v = x + d * np.cos(ym), y + d * np.sin(ym), yaw + dyaw_p, v_new
                travelled += abs(d)
                F = np.array([[1, dt, -dt * dt / 2, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                Gv = np.array([dt * dt / 2, dt, 0.0, 0.0])
                P3 = F @ P3 @ F.T + sa ** 2 * np.outer(Gv, Gv) + np.diag([1e-4 * dt, 0, a.bias_walk ** 2 * dt, 1e-6 * dt])
                Pl = Pl + 0.05 ** 2 * dt + d * d * PY[0, 0]
                Fy = np.array([[1.0, -dt], [0.0, 1.0]]) if heading else np.eye(2)
                PY = Fy @ PY @ Fy.T + np.diag([qy * dt, np.radians(a.gyro_bias_walk) ** 2 * dt])
                if VO is not None and (k - 1) in VO and VO[k - 1][2] > 0.5:   # speed of the pair (k-1, k)
                    zf, zd, _ = VO[k - 1]
                    zv_ = zf / max(zd, 1e-3)
                    Hv = np.array([0, kvo, 0, v])
                    Sv = Hv @ P3 @ Hv + (0.04 * max(v, 1.0)) ** 2 * 3   # single pair: noisier than a 3-pair mean
                    inn = zv_ - kvo * v
                    if inn * inn < 16 * Sv:
                        K = P3 @ Hv / Sv
                        c, s = np.cos(yaw), np.sin(yaw)
                        x, y = x + c * K[0] * inn, y + s * K[0] * inn
                        v, b, kvo = max(0.0, v + K[1] * inn), b + K[2] * inn, float(np.clip(kvo + K[3] * inn, 0.7, 1.3))
                        P3 = P3 - np.outer(K, Hv @ P3)
                        cnt['vo'] += 1
                if G is not None:   # GNSS samples received in (t_k-1, t_k], propagated to t_k
                    while gi < len(G[0]) and G[0][gi] <= TS[k]:
                        lag = (TS[k] - G[0][gi]) * 1e-9
                        gx, gy = G[1][gi] + v * lag * np.cos(yaw), G[2][gi] + v * lag * np.sin(yaw)
                        c, s = np.cos(yaw), np.sin(yaw)
                        ef, el = c * (gx - x) + s * (gy - y), -s * (gx - x) + c * (gy - y)
                        ok1 = upd_fwd(ef, a.gnss_sigma[0], 25.0)
                        ok2 = upd_lat(el, a.gnss_sigma[1], 25.0)
                        cnt['gnss'] += int(ok1 or ok2)
                        gi += 1
            if k in meas_frames[mi:mi + 1]:
                mi += 1
                win = float(np.clip(3 * np.sqrt(P3[0, 0]), 2.5, a.max_window))
                c0, s0 = np.cos(yaw), np.sin(yaw)
                if REPLAY is None:
                    if k not in resp_cache:   # prefetch the next block of measurement frames' image preprocessing
                        blk = meas_frames[mi - 1:mi - 1 + block]
                        resp_cache = dict(pool.imap(responses, blk))
                    resp = resp_cache.pop(k)
                    LS.set_window(max_shift=win)
                    r = lane_measure(k, (x, y, yaw), win)
                    if r is not None:   # record as absolute pose + match quality
                        REC['lane'][k] = (x + c0 * r[0] - s0 * r[1], y + s0 * r[0] + c0 * r[1], yaw + r[2], r[3], r[4], r[5])
                else:
                    r = None
                    if k in REPLAY['lane']:
                        ax, ay, ayaw, sf_, sl_, pk = REPLAY['lane'][k]
                        dfr, dlr = c0 * (ax - x) + s0 * (ay - y), -s0 * (ax - x) + c0 * (ay - y)
                        dyr = (ayaw - yaw + np.pi) % (2 * np.pi) - np.pi
                        lim = win - 0.15
                        r = (dfr, dlr, dyr, sf_, sl_, pk, abs(dfr) < lim and sf_ > 0.003 and pk > a.min_peak,
                             abs(dlr) < lim and sl_ > a.min_sharp_l and pk > a.min_peak,
                             abs(np.degrees(dyr)) < 3.0 - 0.05 and pk > a.min_peak_yaw)
                if r is not None:
                    df, dl, dyaw, sf, sl, peak, f_ok, l_ok, y_ok = r
                    if travelled < a.start_dist:   # (near-)stationary start: heading / forward from the near markings
                        f_ok = y_ok = False
                    if f_ok and sf > a.min_sharp_f:
                        sig = float(np.clip(0.01 / sf, 0.2, 3.0))
                        S = P3[0, 0] + sig ** 2
                        big_f.append(np.sign(df) if df * df > 6.25 * S else 0)
                        if len(big_f) >= 4 and abs(sum(big_f[-4:])) == 4:   # 4 consistent large corrections: drifted
                            P3[0, 0] += df * df
                            big_f.clear()
                        cnt['fwd'] += int(upd_fwd(df, sig))
                    if l_ok and sl > 0:
                        sig = float(np.clip(0.01 / sl, 0.1, 1.0))
                        S = Pl + sig ** 2
                        big_l.append(np.sign(dl) if (dl * dl > 6.25 * S and sl > 0.15) else 0)
                        if len(big_l) >= 4 and abs(sum(big_l[-4:])) == 4:
                            Pl += dl * dl
                            big_l.clear()
                        cnt['lat'] += int(upd_lat(dl, sig))
                    if y_ok:
                        cnt['yaw'] += int(upd_yaw(dyaw - np.radians(a.lane_yaw_off), np.radians(a.yaw_sigma)))
                c0, s0 = np.cos(yaw), np.sin(yaw)
                if REPLAY is None:
                    rn = near_measure(k, (x, y, yaw), resp, float(np.clip(3 * np.sqrt(Pl), 0.4, 1.0)))
                    if rn is not None:
                        REC['near'][k] = (x - s0 * rn[0], y + c0 * rn[0], yaw + rn[1], rn[2], rn[3], rn[4])
                else:
                    rn = None
                    if k in REPLAY['near']:
                        nx, ny, nyaw, sl_, lok, yok = REPLAY['near'][k]
                        rn = (-s0 * (nx - x) + c0 * (ny - y), (nyaw - yaw + np.pi) % (2 * np.pi) - np.pi, sl_, lok, yok)
                if rn is not None:
                    dl2, dyaw2, sl2, l_ok2, y_ok2 = rn
                    if l_ok2 and sl2 > 0:
                        cnt['lat'] += int(upd_lat(dl2, float(np.clip(0.006 / sl2, 0.08, 0.6))))
                    if y_ok2 and heading and travelled >= a.start_dist:
                        cnt['yaw'] += int(upd_yaw(dyaw2 - np.radians(a.near_yaw_off), np.radians(0.6)))
                if REPLAY is None and a.record and travelled < 3 * a.start_dist:   # record a margin for replays
                    dyf_, okf_ = near_yaw(k, (x, y, yaw), span=2.0)
                    if okf_:
                        REC['front'][k] = yaw + dyf_
                if travelled < a.start_dist:   # start: heading from the front camera's near markings only, weakly
                    if REPLAY is None:
                        if a.record:
                            okf = k in REC['front']
                            dyf = ((REC['front'][k] - yaw + np.pi) % (2 * np.pi) - np.pi) if okf else 0.0
                        else:
                            dyf, okf = near_yaw(k, (x, y, yaw), span=2.0)
                    else:
                        okf = k in REPLAY['front']
                        dyf = ((REPLAY['front'][k] - yaw + np.pi) % (2 * np.pi) - np.pi) if okf else 0.0
                    if okf:
                        cnt['yaw'] += int(upd_yaw(dyf - np.radians(a.near_yaw_off), np.radians(a.start_yaw_sigma)))
                P3[0, 0] = max(P3[0, 0], a.floor[0] ** 2)
                Pl = max(Pl, a.floor[1] ** 2)
                PY[0, 0] = max(PY[0, 0], np.radians(a.floor[2]) ** 2)
            out[k] = (x, y, yaw)                   # written once: frame k's estimate uses data up to frame k
            if k % 300 == 0:
                print(f'  frame {k}: x {x:.2f} y {y:.2f} yaw {np.degrees(yaw):.2f} v {v:.2f} | sd fwd {np.sqrt(P3[0, 0]):.2f} '
                      f'lat {np.sqrt(Pl):.2f} | bg {np.degrees(bg):+.3f} kvo {kvo:.3f} | {cnt}', flush=True)
    pd.DataFrame({'idx': np.arange(n), 'x': out[:, 0], 'y': out[:, 1], 'yaw': out[:, 2]}).to_csv(a.out, index=False)
    if a.record:
        np.savez(a.record, rec=np.array(REC, dtype=object))
    print(f'{T}: {cnt} -> {a.out}')
