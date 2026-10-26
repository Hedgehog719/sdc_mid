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
ap.add_argument('--gnss_sigma', type=float, nargs=2, default=[0.3, 0.3], help='m: along / across-track GNSS noise (white part '
                'when --gnss_bias_sd > 0)')
ap.add_argument('--gnss_bias_sd', type=float, default=0.0, help='m (joint EKF): GNSS error modelled as a first-order Gauss-Markov bias '
                'state of this sd (GNSS errors are strongly time-correlated: autocorrelation 0.8-0.9 at 1 s); 0 = white noise only')
ap.add_argument('--gnss_bias_tau', type=float, default=3.0, help='s: correlation time of the GNSS bias')
ap.add_argument('--no_imu', action='store_true')
ap.add_argument('--step', type=int, default=3, help='map measurement every step frames')
ap.add_argument('--init', default='', help='x,y,yaw_rad instead of init_guess.txt')
ap.add_argument('--init_yaw', type=float, default=25.0, help='deg: heading window of the initial search')
ap.add_argument('--init_offset', type=float, default=0.0, help='deg added to the frame-0 heading (heading hypotheses, '
                'tools/hyp_select.py: the frame-0 match can be flat over several degrees)')
ap.add_argument('--rmax', type=float, default=14.0)
ap.add_argument('--lat', type=float, default=6.0)
ap.add_argument('--bg_weight', type=float, default=1.0)
ap.add_argument('--min_contrast', type=float, default=0.12)
ap.add_argument('--acc_noise', type=float, default=0.05)
ap.add_argument('--bias_walk', type=float, default=0.003)
ap.add_argument('--v0_sd', type=float, default=0.3)
ap.add_argument('--vo_dr_init', action='store_true', help='speed unknown at the start (--v0_sd large): until 5 VO pairs give '
                'the median speed, move the position by each pair\'s VO displacement (otherwise it stays put for 4 frames)')
ap.add_argument('--min_sharp_f', type=float, default=0.02)
ap.add_argument('--min_sharp_f_lo', type=float, default=1.0, help='forward fixes with sharpness in (min_sharp_f_lo, '
                'min_sharp_f] only while the forward sd exceeds --lo_sharp_sd (they are biased by -0.2..-0.5 m, but beat '
                'drifting without GNSS)')
ap.add_argument('--lo_sharp_sd', type=float, default=0.35)
ap.add_argument('--relaxed_sharp_f', type=float, default=0.0, help='experimental forward sharpness threshold after '
                '--relaxed_sharp_from; 0 keeps --min_sharp_f throughout')
ap.add_argument('--relaxed_sharp_from', type=int, default=600)
ap.add_argument('--fwd_jump_gate', type=float, nargs=2, default=[0.0, 0.25], metavar=('DIST', 'PEAK'),
                help='experimental rejection of forward jumps above DIST m with peak below PEAK while locked; DIST=0 disables')
ap.add_argument('--floor', type=float, nargs=3, default=[0.3, 0.15, 0.3], help='sd floors: forward m, lateral m, heading deg')
ap.add_argument('--min_peak', type=float, default=0.2)
ap.add_argument('--min_peak_yaw', type=float, default=0.25)
ap.add_argument('--yaw_soft_peak', type=float, default=0.0, help='>0: lane-match headings with a peak between this and '
                '--min_peak_yaw are used too, with sigma --yaw_sigma x (min_peak_yaw / peak)^2 (Track03: the 0.21-0.25 '
                'headings scatter like the 0.25-0.30 ones; a hard cut lost consistent +1 deg corrections at 585-645)')
ap.add_argument('--min_sharp_l', type=float, default=0.1)
ap.add_argument('--min_peak_lo', type=float, default=0.2, help='lane-match peak needed while the state is uncertain (see --lo_peak_sd)')
ap.add_argument('--lo_peak_sd', type=float, nargs=2, default=[0.5, 0.3], help='m: forward / lateral sd above which --min_peak_lo applies')
ap.add_argument('--drift_n', type=int, default=0, help='>0: this many consecutive lane matches (any lateral sharpness) all '
                'more than 2.5 sigma off to the same side -> the lateral state has drifted: inflate its variance (T1 rehearsal: '
                'a 1.5 m wrong lock lasted 20 s while the rejected lane matches kept saying +0.5..+1.7 m)')
ap.add_argument('--stand_search', type=int, default=0, help='>0: while standing still with an uncertain heading (sd > 0.5 deg) '
                'repeat the frame-0 heading search every this many frames as a heading measurement (needs --zupt)')
ap.add_argument('--seg_gate', type=float, default=0.0, help='>0: the frame-0 and standstill heading searches use only the '
                'road surface the cameras see (road probability above this, BEV) for both the image response and the '
                'map marks (Track02 start: map artefacts of the queued cars around the vehicle)')
ap.add_argument('--seg_gate_all', action='store_true', help='--seg_gate for every lane match')
ap.add_argument('--seg_gate_from', type=int, default=0, help='--seg_gate_track: only from this frame on (Track03: the gated '
                'matches of the start dropped the right forward fixes while the position was settling)')
ap.add_argument('--seg_gate_track', action='store_true', help='--seg_gate for the tracking lane matches only (not the '
                'frame-0 / standstill heading searches)')
ap.add_argument('--stand_search_sigma', type=float, default=1.5, help='deg: noise of one standstill heading search')
ap.add_argument('--resp_ratio', type=float, default=0.0, help='>0: near-alignment marking response from the brightness ratio to '
                'the local median (response starts at this ratio, full at +0.5), instead of the fixed 15..45 grey levels above '
                'it: markings in tree shade keep their response')
ap.add_argument('--pitch_track', action='store_true', help='track the camera pitch causally from the lane matches: every '
                '--pitch_every lane-match frames also match at pitch +-pitch_step and move towards the higher peak (road slope '
                'changes, braking dive: the right pitch raises the peak, e.g. 0.31 -> 0.45 at a braking frame)')
ap.add_argument('--pitch_every', type=int, default=2)
ap.add_argument('--pitch_step', type=float, default=0.5, help='deg')
ap.add_argument('--pitch_gain', type=float, default=0.5)
ap.add_argument('--pitch_max', type=float, default=2.0, help='deg')
ap.add_argument('--still_meas', type=int, default=0, help='--still_freeze: still take the map measurements on this many '
                'measurement frames per standstill, 1 s apart from 1 s after the stop (the stop pose itself is measured; the '
                'same scene measured hundreds of times is what made the pose wander)')
ap.add_argument('--still_freeze', action='store_true', help='no lane / near measurements while the IMU shows standstill '
                '(--zupt): the same image measured again and again let the pose wander 0.5 m / 2 deg during a 40 s stop')
ap.add_argument('--terrain', action='store_true', help='lane matching on the map road surface: the image BEV is projected '
                'onto the marking-map ground heights around the prior pose (relative to the road plane under the vehicle) '
                'instead of a flat plane (vertical curves / crowned roads stretch the far field by metres)')
ap.add_argument('--terrain_lat', action='store_true', help='--terrain: also the lateral shape of the map road surface '
                '(crown, gutters, kerb-side lanes) beside the lane strip (Track03 725: the kerb-side line 0.4 m off)')
ap.add_argument('--att_imu_map', type=float, default=0.0, help='>0: body attitude relative to the road per frame (causal) = '
                'IMU roll / pitch minus the map road plane under the prior pose, minus the exponential average of that '
                'difference over this many seconds (IMU mounting); replaces the --dyn_att values (roll in turns, dive, sway)')
ap.add_argument('--att_roll_only', action='store_true', help='--att_imu_map: roll only (keep the --dyn_att pitch model)')
ap.add_argument('--att_pitch_only', action='store_true', help='--att_imu_map: pitch only')
ap.add_argument('--roll_imu_map', type=float, default=0.0, help='>0: static body roll relative to the road, estimated '
                'online: running mean of IMU roll - map road-plane roll over the frames so far (time constant this many '
                'seconds once longer than the elapsed time), added to the camera rig roll (Track03: +0.37 deg, steady)')
ap.add_argument('--att_resid', action='store_true', help='--att_imu_map: high-pass the difference to the --dyn_att '
                'values and add it to them (dive model + fast residual: bumps, sway) instead of replacing them')
ap.add_argument('--dyn_att', action='store_true', help='per-frame dynamic body attitude (causal) in the camera models: '
                'IMU roll high-passed + suspension pitch from the longitudinal acceleration (braking dive, roll in turns)')
ap.add_argument('--pitch_per_accel', type=float, default=-0.8, help='deg of body pitch per m/s^2 of longitudinal acceleration')
ap.add_argument('--roll_tau', type=float, default=2.0, help='s: time constant of the causal high-pass of the IMU roll / pitch')
ap.add_argument('--dyn_pitch_imu', action='store_true', help='add the high-passed IMU pitch to the dynamic pitch')
ap.add_argument('--dyn_roll_scale', type=float, default=1.0, help='factor on the high-passed IMU roll (0 = pitch model only)')
ap.add_argument('--roll_per_lat', type=float, default=0.0, help='deg of body roll per m/s^2 of lateral acceleration (speed x yaw '
                'rate), replaces the IMU roll term for that frame (use with --dyn_roll_scale 0)')
ap.add_argument('--dyn_sign', type=float, default=1.0)
ap.add_argument('--roll_add', type=float, default=0.0, help='deg: constant body roll relative to the road plane added to the '
                'camera rig (left side up +; Track03: IMU roll - map road plane = +0.37 deg along the whole track)')
ap.add_argument('--imu_causal', action='store_true', help='IMU integrals up to the last sample at or before the frame time '
                '(default: linear interpolation at the frame time, which reads the next sample, < 10 ms later)')
ap.add_argument('--lo_peak_start', action='store_true', help='weak lane matches (--min_peak_lo) only before the first regular forward fix '
                '(the start); later they caused a 1 m wrong lateral lock in the T1 rehearsal')
ap.add_argument('--init_pos', type=float, default=0.0, help='>0 (no GNSS): when the frame-0 heading search corrects init_guess by more '
                'than --gross_yaw, also take the frame-0 match position (shift window this many metres)')
ap.add_argument('--gyro_bias', type=float, default=-0.23)
ap.add_argument('--gyro_bias_sd', type=float, default=0.003, help='deg/s: the gyro bias is a sensor calibration constant')
ap.add_argument('--gyro_bias_walk', type=float, default=0.0001)
ap.add_argument('--gyro_noise', type=float, default=0.1)
ap.add_argument('--gyro_scale_sd', type=float, default=0.0, help='relative gyro scale-factor uncertainty (heading variance grows with '
                'the turned angle: after an 84 deg turn a 1 %% scale error is 0.84 deg, and the filter must accept lane headings)')
ap.add_argument('--yaw_sigma', type=float, default=0.6)
ap.add_argument('--max_window', type=float, default=6.0)
ap.add_argument('--yaw_mount', type=float, default=-1.5)
ap.add_argument('--workers', type=int, default=3)
ap.add_argument('--near_yaw_sigma', type=float, default=0.6, help='deg; 0 = no heading from the near alignment (its heading '
                'responds weakly: perturbing the prior heading by 1 deg moves it by only ~0.1 deg)')
ap.add_argument('--lane_k', type=float, nargs=2, default=[0.01, 0.01], help='lane fix sigma = k / sharpness (forward, lateral)')
ap.add_argument('--near_k', type=float, default=0.006)
ap.add_argument('--near_fwd', type=float, default=0.0, help='>0: near-alignment forward offset as a measurement, sigma = '
                'near_fwd / forward sharpness (0 = recorded only)')
ap.add_argument('--near_fwd_min', type=float, default=0.02, help='smallest forward sharpness for a near forward measurement')
ap.add_argument('--near_min_sigma', type=float, default=0.08, help='m: smallest near-alignment lateral sigma (perturbation '
                'test: core sd 0.02-0.03 m, 20-40 %% outliers)')
ap.add_argument('--lane_fc', type=float, default=0.0, help='>0: lane forward sigma = lane_fc / sqrt(sharpness) (measured: '
                'robust sd 0.42 m at sharpness 0.015, 0.75 m at 0.0045, 0.2 m at 0.06 -> ~0.05) instead of lane_k[0] / sharpness')
ap.add_argument('--vo_noise', type=float, default=0.04, help='relative VO speed noise of a 3-pair mean')
ap.add_argument('--kvo_range', type=float, nargs=2, default=[0.85, 1.15], help='VO scale limits (measured: T2 ~1.0, T1 ~0.9; '
                'a loose range let a wrong speed be absorbed into the scale)')
ap.add_argument('--meas_prior', default='', help='parameter studies only: record the map measurements around this '
                'trajectory (own earlier output) instead of the filter state, so that replays stay valid')
ap.add_argument('--accum_dist', type=float, default=0.0, help='>0: lane matches on the camera response accumulated over '
                'the past accum_dist metres (frames <= k, warped by the filter\'s own past poses)')
ap.add_argument('--record', default='', help='save the per-frame map measurements (absolute poses) for --replay')
ap.add_argument('--replay', default='', help='run the filter on recorded map measurements (fast parameter studies)')
ap.add_argument('--lane_yaw_off', type=float, default=0.25, help='deg: heading bias of the lane-match headings (calibration, '
                'from the course of travel: T1 +0.25, T2 +0.25)')
ap.add_argument('--near_yaw_off', type=float, default=0.2, help='deg: heading bias of the near-alignment headings (T1 +0.26, T2 +0.15)')
ap.add_argument('--gross_yaw', type=float, default=5.0, help='deg: the frame-0 search corrects init_guess only above this')
ap.add_argument('--start_yaw_sigma', type=float, default=1.5, help='deg: front-camera near heading noise in the start segment')
ap.add_argument('--start_dist', type=float, default=15.0, help='m travelled (so far) below which lane matches give lateral only')
ap.add_argument('--joint', action='store_true', help='one EKF over [x, y, heading, speed, accel bias, VO scale, gyro bias]: '
                'lateral fixes also correct the heading through the motion model (needs the IMU)')
ap.add_argument('--lat_q', type=float, default=0.05, help='m/sqrt(s): lateral model noise (side slip, unmodelled motion)')
ap.add_argument('--yaw0_sd', type=float, default=2.0, help='deg: initial heading uncertainty')
ap.add_argument('--pivot', choices=['marks', 'origin'], default='marks', help='near alignment: heading candidates rotate '
                'about the centroid of the visible marks (decoupled from the lateral shift) or about the vehicle origin')
ap.add_argument('--zupt', action='store_true', help='joint EKF: when the IMU shows standstill (last 1 s: accelerometer '
                'norm steady, gyro small) the gyro reading measures its bias and the speed is zero')
ap.add_argument('--zupt_acc', type=float, default=0.06, help='m/s^2: accelerometer-norm sd over 1 s below which = standstill')
ap.add_argument('--zupt_gyro', type=float, default=1.2, help='deg/s: max gyro norm over 1 s for standstill')
ap.add_argument('--huber', type=float, default=0.0, help='>0 (joint EKF): map-match innovations above this many sigma '
                'get their noise inflated proportionally (Huber weight)')
ap.add_argument('--frames', type=int, default=0, help='>0: stop after this many frames (diagnostics)')
ap.add_argument('--debug', default='', help='csv: per-frame state and every measurement (value, accepted or not)')
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
    if a.resp_ratio > 0:   # brightness ratio to the local median: the same response in tree shade and in sunlight
        r = np.clip((img.astype(np.float32) / np.maximum(med, 8.0) - a.resp_ratio) / 0.5, 0, 1)
    else:
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


R_CUR = None   # camera-rig rotation of the frame being measured (yaw mount x dynamic pitch / roll, --dyn_att)


def rig_rotation(j):
    if DYN is None:
        return Rm
    return Rotation.from_euler('ZYX', [a.yaw_mount, DYN[j, 0], DYN[j, 1]], degrees=True).as_matrix()


def score(q, resp):
    vals = []
    Rr = Rm if R_CUR is None else R_CUR
    for c in CAMS:
        uv, ok = C.project(c, q @ Rr)
        h, w = resp[c].shape
        u, v = uv[:, 0], uv[:, 1]
        ins = ok & (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        if ins.any():
            vals.append(cv2.remap(resp[c], u[ins].astype(np.float32).reshape(-1, 1), v[ins].astype(np.float32).reshape(-1, 1),
                                  cv2.INTER_LINEAR).ravel())
    v = np.concatenate(vals) if vals else np.zeros(0)
    return float(v.mean()) if len(v) > 30 else np.nan


def near_measure(j, pose, resp, span_l, yaw_span=1.0):
    """Near-range image alignment of frame j: (dlat, dyaw, sharp_l, lateral ok, heading ok) or None."""
    global R_CUR
    R_CUR = rig_rotation(j)
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

    # heading candidates rotate about the centroid of the visible marks, not the vehicle origin: marks in a narrow range
    # band move sideways under a rotation about the origin, the lateral search absorbs that and the heading stays wrong
    cx, cy = (float(q[:, 0].mean()), float(q[:, 1].mean())) if a.pivot == 'marks' else (0.0, 0.0)

    def sc(df, dl, rot=0.0):
        def tf(p):
            p = p - [df, dl, 0]
            if rot:
                ct, st_ = np.cos(-rot), np.sin(-rot)
                px, py = p[:, 0] - cx, p[:, 1] - cy
                p = np.c_[ct * px - st_ * py + cx, st_ * px + ct * py + cy, p[:, 2]]
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
    ys = np.radians(np.arange(-yaw_span, yaw_span + 1e-4, 0.2))
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
    # forward offset from the same grid (transverse marks: stop lines, crosswalks, dash ends), refined at the heading
    f_ok = 0 < i < len(fo) - 1
    gf = np.nan_to_num(np.array([sc(df + w, dl, dyaw) for w in (-0.05, 0.0, 0.05)]), nan=-1.0)
    den = gf[0] - 2 * gf[1] + gf[2]
    if den < 0:
        df += float(np.clip(0.5 * (gf[0] - gf[2]) / den, -1, 1)) * 0.05
    sf = peak - max(g[max(i - 2, 0), k], g[min(i + 2, len(fo) - 1), k])
    # the same correction about the vehicle origin: translation (df, dl) + c - R(dyaw) c
    dl += cy * (1 - np.cos(dyaw)) - np.sin(dyaw) * cx
    df += cx * (1 - np.cos(dyaw)) + np.sin(dyaw) * cy
    return dl, dyaw, sl, l_ok, y_ok, df, sf, f_ok


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


def lane_measure(j, pose, win, lo_f=False, lo_l=False):
    """lo_f / lo_l: the forward / lateral state is uncertain -> accept weaker matches (peak > --min_peak_lo)."""
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
    f_ok = abs(fwd) < lim and sharp_f > 0.003 and peak > (a.min_peak_lo if lo_f else a.min_peak)
    l_ok = abs(lat) < lim and sharp_l > a.min_sharp_l and peak > (a.min_peak_lo if lo_l else a.min_peak)
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
    if a.imu_causal:   # integrals at the query time from the last IMU sample at or before it, extrapolated with that
        # sample's rate (no sample after frame k is read, and no lag of up to one IMU period in turns)
        last = lambda tq: max(int(np.searchsorted(t, tq, side='right')) - 1, 0)   # noqa: E731

        def at(cum, rate, tq):
            i = last(tq)
            return float(cum[i] + rate[i] * max(tq - t[i], 0.0) * 1e-9)
        heading = lambda t0, t1: at(yaw_g, wz, t1) - at(yaw_g, wz, t0)           # noqa: E731
        dvel = lambda t0, t1: at(ca, along, t1) - at(ca, along, t0)              # noqa: E731
    else:
        heading = lambda t0, t1: float(np.interp(t1, t, yaw_g) - np.interp(t0, t, yaw_g))   # noqa: E731
        dvel = lambda t0, t1: float(np.interp(t1, t, ca) - np.interp(t0, t, ca))            # noqa: E731
    global IMU_RAW, DYN, ATT
    IMU_RAW = (t, wz, np.linalg.norm(im[['ax', 'ay', 'az']].values, axis=1), np.linalg.norm(im[['wx', 'wy', 'wz']].values, axis=1))
    if a.att_imu_map > 0 or a.roll_imu_map > 0:
        ea = q.as_euler('ZYX', degrees=True)
        ATT = (t, ea[:, 1], ea[:, 2])   # IMU pitch (nose down +), roll (left side up +)
    if a.dyn_att:
        # dynamic body attitude per frame (causal): roll = IMU roll minus its exponential average over the past
        # roll_tau seconds (turns, bumps; the slow part is road tilt); pitch = suspension model pitch_per_accel x the
        # longitudinal acceleration over the past 0.5 s (nose dive when braking, squat when accelerating) [+ the
        # high-passed IMU pitch with --dyn_pitch_imu]
        e = q.as_euler('ZYX', degrees=True)
        idx = np.clip(np.searchsorted(t, TS.astype(np.float64), side='right') - 1, 0, len(t) - 1)
        roll, pitch = e[idx, 2], e[idx, 1]
        i0 = np.clip(np.searchsorted(t, TS.astype(np.float64) - 0.5e9, side='right') - 1, 0, len(t) - 1)
        acc = (ca[idx] - ca[i0]) / np.maximum((t[idx] - t[i0]) * 1e-9, 1e-3)
        DYN = np.zeros((len(TS), 2))
        br, bp = roll[0], pitch[0]
        for k in range(len(TS)):
            if k > 0:
                al = min((TS[k] - TS[k - 1]) * 1e-9 / a.roll_tau, 1.0)
                br, bp = br + al * (roll[k] - br), bp + al * (pitch[k] - bp)
            DYN[k] = (a.pitch_per_accel * acc[k] + (pitch[k] - bp if a.dyn_pitch_imu else 0.0), a.dyn_roll_scale * (roll[k] - br))
        DYN *= a.dyn_sign
    return heading, dvel


DYN = None   # (frames, 2) dynamic pitch, roll in degrees (--dyn_att)
ATT = None   # (IMU times, pitch deg, roll deg) for --att_imu_map
PL_U, PL_V = (g.ravel() for g in np.meshgrid(np.linspace(-4, 4, 9), np.linspace(-2, 2, 5)))   # road plane under the vehicle


IMU_RAW = None


def standstill(t1):
    """IMU samples of the last second up to t1 (causal): standstill -> (mean world-z gyro rate over the last frame
    interval is taken by the caller); returns True when the accelerometer norm is steady and the gyro small."""
    t, _, an, wn = IMU_RAW
    m0, m1 = np.searchsorted(t, t1 - 1e9, 'right'), np.searchsorted(t, t1, 'right')
    return m1 - m0 > 50 and an[m0:m1].std() < a.zupt_acc and wn[m0:m1].max() < np.radians(a.zupt_gyro)


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
    LS.BM.A.seg_gate = 0.0 if a.seg_gate_track else a.seg_gate
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
    elif a.init_pos and G is None:   # init_guess grossly off (heading > gross_yaw): its position is suspect as well ->
        LS.set_window(max_shift=a.init_pos, max_yaw=1.0, yaw_step=0.5)   # frame-0 match position, wide window
        try:
            r = LS.match(0, (x, y, yaw))
        except (IndexError, ValueError):
            r = None
        if r is not None and r[0] > a.min_peak_lo:
            c, s = np.cos(yaw), np.sin(yaw)
            x, y = x + c * r[2] - s * r[3], y + s * r[2] + c * r[3]
            print(f'frame-0 position correction: forward {r[2]:+.2f} m, lateral {r[3]:+.2f} m (peak {r[0]:.3f})', flush=True)
        LS.set_window(max_shift=2.5, max_yaw=3.0, yaw_step=0.5)
    # gate for the tracking matches from here on (the frame-0 heading search and position match stay ungated)
    LS.BM.A.seg_gate = a.seg_gate if (a.seg_gate_all or (a.seg_gate_track and a.seg_gate_from <= 0)) else 0.0
    return [x, y, yaw]


if __name__ == '__main__':
    n = len(TS) if a.frames <= 0 else min(len(TS), a.frames)
    DBG = []
    REC = {'lane': {}, 'near': {}, 'front': {}}
    MP = pd.read_csv(a.meas_prior) if a.meas_prior else None
    REPLAY = None
    if a.replay:
        zr = np.load(a.replay, allow_pickle=True)
        REPLAY = zr['rec'].item()
    else:
        LS.setup(a.track_dir, a.lane_marks, yaw_mount=a.yaw_mount)
        if a.accum_dist > 0:
            LS.setup_accum(n, accum_dist=a.accum_dist)
    heading, dvel = load_imu()
    if (a.pitch_track or a.att_imu_map > 0 or a.roll_add or a.roll_imu_map > 0) and DYN is None and REPLAY is None:
        DYN = np.zeros((n, 2))
    if a.roll_add and DYN is not None:
        DYN[:, 1] += a.roll_add
    att_ema = None
    still_k0, still_used = None, 0   # --still_meas: frame where the current standstill began
    roll_mean, roll_n = 0.0, 0   # --roll_imu_map
    if a.terrain and REPLAY is None:
        LS.BM.BEV.terrain = MK.height
        LS.BM.BEV.TERRAIN_LAT = a.terrain_lat
    if DYN is not None and REPLAY is None:
        LS.BM.BEV.dyn = DYN
        print(f'dynamic attitude: roll sd {DYN[:, 1].std():.2f} deg (max {np.abs(DYN[:, 1]).max():.1f}), pitch sd {DYN[:, 0].std():.2f} deg '
              f'(max {np.abs(DYN[:, 0]).max():.1f})', flush=True)
    G = load_gnss()
    VO = None
    if a.vo:
        zv = np.load(a.vo)
        VO = {int(j): (f, d, r) for j, f, d, r in zip(zv['j'], zv['fwd'], zv['dt'], zv['resp'])}   # pair (j, j + 1)
    print(f'{T}: {n} frames, IMU {"yes" if heading else "no"}, GNSS {"yes" if G is not None else "no"}, VO {"yes" if VO else "no"}', flush=True)
    if REPLAY is None:
        LS.set_motion_frame(0, 0.0)
    x, y, yaw = REPLAY['init'] if REPLAY is not None else initial_pose(G)
    yaw += np.radians(a.init_offset)
    REC['init'] = (x, y, yaw)
    print(f'initial pose (frame 0): x {x:.2f} y {y:.2f} yaw {np.degrees(yaw):.2f} deg', flush=True)
    v, b, w, kvo = 0.0, 0.0, 0.0, 1.0
    P3 = np.diag([1.0, a.v0_sd, 0.05, 0.05]) ** 2      # along-track [position, speed, accel bias, VO scale]
    Pl = 1.0 ** 2                                       # cross-track position
    bg = np.radians(a.gyro_bias)
    PY = np.diag([np.radians(a.yaw0_sd), np.radians(a.gyro_bias_sd)]) ** 2   # [heading, gyro bias]
    sa = a.acc_noise if dvel else 1.5
    qy = np.radians(a.gyro_noise) ** 2 if heading else np.radians(1.0) ** 2
    if a.joint and not heading:
        print('--joint needs the IMU: using the separate filters', flush=True)
        a.joint = False
    # joint EKF state [x, y, heading, speed, accel bias, VO scale, gyro bias] (the variables x, y, yaw, v, b, kvo, bg)
    # + GNSS bias [x, y] (time-correlated GNSS error, first-order Gauss-Markov; zero variance = off)
    PJ = np.diag([1.0, 1.0, np.radians(a.yaw0_sd), a.v0_sd, 0.05, 0.05, np.radians(a.gyro_bias_sd), a.gnss_bias_sd, a.gnss_bias_sd]) ** 2
    gbx = gby = 0.0
    LL = 0.0          # running log-likelihood of the map measurements
    out = np.zeros((n, 3))
    big_f, big_l = [], []
    drift_l = []
    pitch_est = 0.0   # tracked camera pitch offset (--pitch_track), degrees
    vo_hist = []
    lo_ok = True      # weak lane matches allowed (--min_peak_lo); with --lo_peak_start only until the first regular lock
    travelled = 0.0
    cnt = dict(fwd=0, lat=0, yaw=0, vo=0, gnss=0, still=0)
    gi = 0
    if G is not None:
        gi = int(np.searchsorted(G[0], TS[0], side='right'))   # samples up to frame 0 are not used afterwards

    def axis(kind):   # unit vector of the vehicle's forward ('f') or left ('l') axis in the map frame
        return np.array([np.cos(yaw), np.sin(yaw)]) if kind == 'f' else np.array([-np.sin(yaw), np.cos(yaw)])

    def var(kind):    # current variance: forward / lateral position (m^2), heading ('y', rad^2)
        if a.joint:
            return float(PJ[2, 2]) if kind == 'y' else float(axis(kind) @ PJ[:2, :2] @ axis(kind))
        return {'f': P3[0, 0], 'l': Pl, 'y': PY[0, 0]}[kind]

    def inflate(kind, dv):
        global P3, Pl, PY, PJ
        if a.joint:
            if kind == 'y':
                PJ[2, 2] += dv
            else:
                PJ[:2, :2] += dv * np.outer(axis(kind), axis(kind))
        elif kind == 'f':
            P3[0, 0] += dv
        elif kind == 'l':
            Pl += dv
        else:
            PY[0, 0] += dv

    def floor(kind, sd):
        cur = var(kind)
        if cur < sd * sd:
            inflate(kind, sd * sd - cur)

    def jupd(h, e, r2, gate, robust=False):
        """Scalar measurement update of the joint EKF: innovation e = z - h.s, noise variance r2. robust: Huber
        weight (map matches have heavy tails: a normalised innovation above --huber inflates the noise)."""
        global x, y, yaw, v, b, kvo, bg, PJ, gbx, gby, LL
        S = float(h @ PJ @ h) + r2
        if e * e >= gate * S:
            if robust:   # map measurement outside the gate: counts against this filter (hypothesis selection)
                LL -= 0.5 * gate + 0.5 * np.log(2 * np.pi * S)
            return False
        if robust:   # log-likelihood of the map measurements (tools/hyp_select.py)
            LL -= 0.5 * e * e / S + 0.5 * np.log(2 * np.pi * S)
        t = abs(e) / np.sqrt(S)
        if robust and a.huber > 0 and t > a.huber:
            S = S - r2 + r2 * t / a.huber
        K = PJ @ h / S
        x, y, yaw = x + K[0] * e, y + K[1] * e, yaw + K[2] * e
        v, b, kvo, bg = max(0.0, v + K[3] * e), b + K[4] * e, float(np.clip(kvo + K[5] * e, *a.kvo_range)), bg + K[6] * e
        gbx, gby = gbx + K[7] * e, gby + K[8] * e
        PJ = PJ - np.outer(K, h @ PJ)
        PJ = 0.5 * (PJ + PJ.T)
        return True

    def jpredict(dt, dpsi, dvm):
        """Joint EKF prediction over dt: heading change dpsi (gyro - bias), speed change dvm (IMU, before the bias)."""
        global x, y, yaw, v, PJ, gbx, gby
        v_new = max(0.0, v + dvm - b * dt)
        ym = yaw + 0.5 * dpsi
        d = 0.5 * (v + v_new) * dt
        cm, sm = np.cos(ym), np.sin(ym)
        F = np.eye(9)
        F[0, 2], F[1, 2] = -d * sm, d * cm
        F[0, 3], F[1, 3] = dt * cm, dt * sm
        F[0, 4], F[1, 4] = -0.5 * dt * dt * cm, -0.5 * dt * dt * sm
        F[0, 6], F[1, 6] = 0.5 * d * dt * sm, -0.5 * d * dt * cm
        F[2, 6], F[3, 4] = -dt, -dt
        Gv = np.array([0.5 * dt * dt * cm, 0.5 * dt * dt * sm, 0, dt, 0, 0, 0, 0, 0])
        Q = sa ** 2 * np.outer(Gv, Gv)
        Rr = np.array([[cm, -sm], [sm, cm]])
        Q[:2, :2] += Rr @ np.diag([1e-4 * dt, a.lat_q ** 2 * dt]) @ Rr.T
        Q[2, 2] += qy * dt + (a.gyro_scale_sd * dpsi) ** 2   # gyro scale-factor error grows the heading variance in turns
        Q[4, 4] += a.bias_walk ** 2 * dt
        Q[5, 5] += 1e-6 * dt
        Q[6, 6] += np.radians(a.gyro_bias_walk) ** 2 * dt
        if a.gnss_bias_sd > 0:
            rho = np.exp(-dt / a.gnss_bias_tau)
            F[7, 7] = F[8, 8] = rho
            Q[7, 7] = Q[8, 8] = a.gnss_bias_sd ** 2 * (1 - rho * rho)
            gbx, gby = rho * gbx, rho * gby
        PJ = F @ PJ @ F.T + Q
        x, y, yaw, v = x + d * cm, y + d * sm, yaw + dpsi, v_new
        return d

    def upd_fwd(e, sig, gate=16.0, robust=False):
        global x, y, v, b, kvo, P3
        if a.joint:
            return jupd(np.r_[axis('f'), np.zeros(7)], e, sig ** 2, gate, robust)
        S = P3[0, 0] + sig ** 2
        if e * e >= gate * S:
            return False
        K = P3[:, 0] / S
        c, s = np.cos(yaw), np.sin(yaw)
        x, y = x + c * K[0] * e, y + s * K[0] * e
        v, b, kvo = max(0.0, v + K[1] * e), b + K[2] * e, float(np.clip(kvo + K[3] * e, *a.kvo_range))
        P3 = P3 - np.outer(K, P3[0, :])
        return True

    def upd_lat(e, sig, gate=16.0, robust=False):
        global x, y, Pl
        if a.joint:
            return jupd(np.r_[axis('l'), np.zeros(7)], e, sig ** 2, gate, robust)
        S = Pl + sig ** 2
        if e * e >= gate * S:
            return False
        K = Pl / S
        c, s = np.cos(yaw), np.sin(yaw)
        x, y = x - s * K * e, y + c * K * e
        Pl = (1 - K) * Pl
        return True

    def upd_yaw(e, sig, gate=16.0, robust=False):
        global yaw, bg, PY, w
        if a.joint:
            return jupd(np.eye(9)[2], e, sig ** 2, gate, robust)
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
            D = {'k': k}
            if k > 0:
                t0, t1 = TS[k - 1] * 1.0, TS[k] * 1.0
                dt = (t1 - t0) * 1e-9
                dyaw_p = (heading(t0, t1) - bg * dt) if heading else w * dt
                if a.joint:
                    d = jpredict(dt, dyaw_p, dvel(t0, t1) if dvel else 0.0)
                else:
                    v_new = max(0.0, v + ((dvel(t0, t1) if dvel else 0.0) - b * dt))
                    ym = yaw + 0.5 * dyaw_p
                    d = 0.5 * (v + v_new) * dt
                    x, y, yaw, v = x + d * np.cos(ym), y + d * np.sin(ym), yaw + dyaw_p, v_new
                    F = np.array([[1, dt, -dt * dt / 2, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    Gv = np.array([dt * dt / 2, dt, 0.0, 0.0])
                    P3 = F @ P3 @ F.T + sa ** 2 * np.outer(Gv, Gv) + np.diag([1e-4 * dt, 0, a.bias_walk ** 2 * dt, 1e-6 * dt])
                    Pl = Pl + a.lat_q ** 2 * dt + d * d * PY[0, 0]
                    Fy = np.array([[1.0, -dt], [0.0, 1.0]]) if heading else np.eye(2)
                    PY = Fy @ PY @ Fy.T + np.diag([qy * dt, np.radians(a.gyro_bias_walk) ** 2 * dt])
                travelled += abs(d)
                if DYN is not None and a.roll_per_lat != 0.0 and REPLAY is None:
                    # suspension roll model: body roll proportional to the lateral (centripetal) acceleration speed x yaw
                    # rate (the IMU's own roll in turns contains the centripetal pseudo-tilt)
                    DYN[k, 1] = a.dyn_sign * a.roll_per_lat * v * dyaw_p / max(dt, 1e-3)
                # standstill: IMU quiet for 1 s, no speed change over that second (a gentle start barely changes the
                # accelerometer norm: T2 frames 200-220 at 0.3 m/s^2), slow, and the VO (when valid) agrees
                vo_moving = VO is not None and (k - 1) in VO and VO[k - 1][2] > 0.5 and abs(VO[k - 1][0]) > 0.004
                if a.zupt and a.joint and v < 0.3 and standstill(t1) and abs(dvel(t1 - 1e9, t1)) < 0.08 and not vo_moving:
                    ti_, wz_ = IMU_RAW[0], IMU_RAW[1]
                    j0, j1 = np.searchsorted(ti_, t0, 'right'), np.searchsorted(ti_, t1, 'right')
                    if j1 > j0:
                        jupd(np.eye(9)[6], float(wz_[j0:j1].mean()) - bg, np.radians(0.1) ** 2 / (j1 - j0), 25.0)
                        jupd(np.eye(9)[3], -v, 0.05 ** 2, 1e9)
                        cnt['still'] += 1
                        D['still'] = 1
                if VO is not None and (k - 1) in VO and VO[k - 1][2] > 0.5:   # speed of the pair (k-1, k)
                    zf, zd, _ = VO[k - 1]
                    zv_ = zf / max(zd, 1e-3)
                    rv = (a.vo_noise * max(v, 1.0)) ** 2 * 3   # single pair: noisier than a 3-pair mean
                    inn = zv_ - kvo * v
                    D['vo_z'], D['vo_v'] = zv_, kvo * v
                    vo_hist.append(zv_)
                    if a.joint and PJ[3, 3] > 4.0:   # speed unknown (start in motion): one pair can be an alias (the
                        # first VO pair has no prediction) -> wait for 5 valid pairs, initialise with their median
                        if len(vo_hist) >= 5:
                            zm = float(np.median(vo_hist[-5:]))
                            D['vo_a'] = int(jupd(np.array([0, 0, 0, kvo, 0, v, 0, 0, 0]), zm - kvo * v, (0.1 * max(zm, 1.0)) ** 2, 1e9))
                            cnt['vo'] += D['vo_a']
                        elif a.vo_dr_init:   # until then the VO displacement of the pair (k-1, k) is the only motion
                            # information: dead-reckon the position with it (the speed x the frame interval cannot:
                            # the speed is unknown and the first intervals are irregular, Track03 11.7 / 48.3 / 23.7 ms
                            # for a steady 0.25-0.28 m per pair)
                            dd = zf / max(kvo, 1e-3) - d
                            x, y = x + dd * np.cos(yaw), y + dd * np.sin(yaw)
                            u = np.array([np.cos(yaw), np.sin(yaw)])
                            PJ[:2, :2] += np.outer(u, u) * (0.1 * abs(zf)) ** 2
                            D['vo_a'] = 1
                        Sv = -1.0
                    elif a.joint:
                        D['vo_a'] = int(jupd(np.array([0, 0, 0, kvo, 0, v, 0, 0, 0]), inn, rv, 16.0))
                        cnt['vo'] += D['vo_a']
                        Sv = -1.0
                    else:
                        Hv = np.array([0, kvo, 0, v])
                        Sv = Hv @ P3 @ Hv + rv
                    if inn * inn < 16 * Sv:
                        K = P3 @ Hv / Sv
                        c, s = np.cos(yaw), np.sin(yaw)
                        x, y = x + c * K[0] * inn, y + s * K[0] * inn
                        v, b, kvo = max(0.0, v + K[1] * inn), b + K[2] * inn, float(np.clip(kvo + K[3] * inn, *a.kvo_range))
                        P3 = P3 - np.outer(K, Hv @ P3)
                        cnt['vo'] += 1
                        D['vo_a'] = 1
                if G is not None:   # GNSS samples received in (t_k-1, t_k], propagated to t_k
                    while gi < len(G[0]) and G[0][gi] <= TS[k]:
                        lag = (TS[k] - G[0][gi]) * 1e-9
                        gx, gy = G[1][gi] + v * lag * np.cos(yaw), G[2][gi] + v * lag * np.sin(yaw)
                        c, s = np.cos(yaw), np.sin(yaw)
                        if a.joint and a.gnss_bias_sd > 0:   # GNSS = position + time-correlated bias + white noise
                            ef = c * (gx - x - gbx) + s * (gy - y - gby)
                            ok1 = jupd(np.array([c, s, 0, 0, 0, 0, 0, c, s]), ef, a.gnss_sigma[0] ** 2, 25.0)
                            el = -s * (gx - x - gbx) + c * (gy - y - gby)
                            ok2 = jupd(np.array([-s, c, 0, 0, 0, 0, 0, -s, c]), el, a.gnss_sigma[1] ** 2, 25.0)
                        else:
                            ef, el = c * (gx - x) + s * (gy - y), -s * (gx - x) + c * (gy - y)
                            ok1 = upd_fwd(ef, a.gnss_sigma[0], 25.0)
                            ok2 = upd_lat(el, a.gnss_sigma[1], 25.0)
                        D.update(g_ef=ef, g_el=el, g_fa=int(ok1), g_la=int(ok2))
                        cnt['gnss'] += int(ok1 or ok2)
                        gi += 1
            if D.get('still'):
                if still_k0 is None:
                    still_k0, still_used = k, 0
            else:
                still_k0 = None
            if ATT is not None and REPLAY is None:
                # body attitude relative to the road (causal): IMU roll / pitch at the frame time minus the map road plane
                # under the prior pose, minus the slow average of that difference (IMU mounting): body roll in turns,
                # braking dive / squat, sway on uneven roads (the road plane itself is the static tilt of the vehicle)
                ti = max(int(np.searchsorted(ATT[0], TS[k], side='right')) - 1, 0)
                c_, s_ = np.cos(yaw), np.sin(yaw)
                pz_, okp_ = MK.height(x + c_ * PL_U - s_ * PL_V, y + s_ * PL_U + c_ * PL_V)
                okp_ &= np.isfinite(pz_)
                if okp_.sum() >= 10:
                    pa_ = np.linalg.lstsq(np.stack([PL_U[okp_], PL_V[okp_], np.ones(okp_.sum())], 1), pz_[okp_], rcond=None)[0]
                    diff_ = np.array([ATT[1][ti] + np.degrees(np.arctan(pa_[0])), ATT[2][ti] - np.degrees(np.arctan(pa_[1]))])
                    if a.roll_imu_map > 0:   # static body roll: running mean of the roll difference (frames <= k)
                        roll_n += 1
                        al_ = max(1.0 / roll_n, min((TS[k] - TS[k - 1]) * 1e-9 / a.roll_imu_map, 1.0) if k else 1.0)
                        roll_mean += al_ * (diff_[1] - roll_mean)
                    if a.att_imu_map > 0:
                        base_ = DYN[k].copy() if a.att_resid else np.zeros(2)   # --dyn_att values of this frame
                        diff_ -= base_
                        if att_ema is None:
                            att_ema = diff_.copy()
                        else:
                            al_ = min((TS[k] - TS[k - 1]) * 1e-9 / a.att_imu_map, 1.0)
                            att_ema = att_ema + al_ * (diff_ - att_ema)
                        hp_ = diff_ - att_ema
                        if not a.att_pitch_only:
                            DYN[k, 1] = base_[1] + hp_[1]
                        if not a.att_roll_only:
                            DYN[k, 0] = base_[0] + hp_[0]
                if a.roll_imu_map > 0:   # also when the map plane is missing here (last estimate)
                    DYN[k, 1] += roll_mean
                D['att_p'], D['att_r'] = float(DYN[k, 0]), float(DYN[k, 1])
            if (a.stand_search > 0 and REPLAY is None and D.get('still') and k % a.stand_search == 0
                    and var('y') > np.radians(0.5) ** 2):
                # standing with an uncertain heading: repeat the frame-0 heading search (vehicles around move, other
                # markings appear; Track02: frame 0 -20.7 deg, frames 30-180 -17.9 deg, reference -18.8) as a heading
                # measurement - the searches are averaged by the filter
                yaw_s = yaw
                LS.BM.A.seg_gate = 0.0 if a.seg_gate_track else a.seg_gate
                LS.set_motion_frame(k, v)
                for my, ms, ys in ((a.init_yaw, 1.0, 1.0), (3.0, 1.0, 0.5)):
                    LS.set_window(max_shift=ms, max_yaw=my, yaw_step=ys)
                    try:
                        rs = LS.match(k, (x, y, yaw_s))
                    except (IndexError, ValueError):
                        rs = None
                    if rs is not None and rs[0] > 0.08:
                        yaw_s = yaw_s + rs[4] - np.radians(a.lane_yaw_off)
                LS.set_window(max_shift=2.5, max_yaw=3.0, yaw_step=0.5)
                LS.BM.A.seg_gate = a.seg_gate if (a.seg_gate_all or a.seg_gate_track) else 0.0
                D['ss_dy'] = np.degrees((yaw_s - yaw + np.pi) % (2 * np.pi) - np.pi)
                D['ss_a'] = int(upd_yaw((yaw_s - yaw + np.pi) % (2 * np.pi) - np.pi, np.radians(a.stand_search_sigma), 25.0))
                cnt['yaw'] += D['ss_a']
            if k in meas_frames[mi:mi + 1]:
                mi += 1
                win = float(np.clip(3 * np.sqrt(var('f')), 2.5, a.max_window))
                c0, s0 = np.cos(yaw), np.sin(yaw)
                if REPLAY is None:
                    if k not in resp_cache:   # prefetch the next block of measurement frames' image preprocessing
                        blk = meas_frames[mi - 1:mi - 1 + block]
                        resp_cache = dict(pool.imap(responses, blk))
                    resp = resp_cache.pop(k)
                    LS.set_window(max_shift=win)
                    mp = (MP.x[k], MP.y[k], MP.yaw[k]) if MP is not None else (x, y, yaw)
                    if a.accum_dist > 0:
                        LS.set_pose(k, mp)   # current prior of frame k (past frames hold their final poses)
                    frozen = a.still_freeze and bool(D.get('still'))
                    if frozen and a.still_meas > 0 and still_k0 is not None:   # a few measurements of the stop pose
                        el = k - still_k0
                        if el >= 28 and still_used < a.still_meas and el >= 28 * (still_used + 1):
                            frozen = False
                            still_used += 1
                            D['still_m'] = 1
                    frozen_k = frozen
                    if DYN is not None and a.pitch_track and not frozen:
                        DYN[k, 0] = pitch_est   # camera pitch tracked from the lane matches (slope changes, braking dive)
                    if a.seg_gate_track and a.seg_gate_from > 0:
                        LS.BM.A.seg_gate = a.seg_gate if k >= a.seg_gate_from else 0.0
                    LS.set_motion_frame(k, v)
                    r = None if frozen else lane_measure(k, mp, win, lo_ok and var('f') > a.lo_peak_sd[0] ** 2, lo_ok and var('l') > a.lo_peak_sd[1] ** 2)
                    if a.pitch_track and not frozen and r is not None and mi % a.pitch_every == 0:
                        pk = [None, r[5], None]
                        for ii, dp in ((0, -a.pitch_step), (2, a.pitch_step)):
                            DYN[k, 0] = pitch_est + dp
                            try:
                                rr = LS.match(k, mp)
                            except (IndexError, ValueError):
                                rr = None
                            pk[ii] = rr[0] if rr is not None else None
                        DYN[k, 0] = pitch_est
                        if None not in pk and min(pk) > 0.1:
                            den = pk[0] - 2 * pk[1] + pk[2]
                            if den < 0:
                                dp = float(np.clip(0.5 * (pk[0] - pk[2]) / den, -1, 1)) * a.pitch_step
                            else:   # no interior maximum: step towards the higher side
                                dp = a.pitch_step * (1 if pk[2] > pk[0] else -1)
                            pitch_est = float(np.clip(pitch_est + a.pitch_gain * dp, -a.pitch_max, a.pitch_max))
                        D['pitch'] = pitch_est
                    if r is not None:   # record as absolute pose + match quality
                        cm, sm = np.cos(mp[2]), np.sin(mp[2])
                        REC['lane'][k] = (mp[0] + cm * r[0] - sm * r[1], mp[1] + sm * r[0] + cm * r[1], mp[2] + r[2], r[3], r[4], r[5])
                        if MP is not None:   # express relative to the filter state
                            ax, ay, ayaw = REC['lane'][k][:3]
                            dfr, dlr = c0 * (ax - x) + s0 * (ay - y), -s0 * (ax - x) + c0 * (ay - y)
                            r = (dfr, dlr, (ayaw - yaw + np.pi) % (2 * np.pi) - np.pi) + tuple(r[3:6]) + (r[6], r[7], r[8])
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
                    D.update(ln_df=df, ln_dl=dl, ln_dy=np.degrees(dyaw), ln_sf=sf, ln_sl=sl, ln_pk=peak, ln_fok=int(f_ok),
                             ln_lok=int(l_ok), ln_yok=int(y_ok), ln_fa=0, ln_la=0, ln_ya=0)
                    min_sf = a.relaxed_sharp_f if a.relaxed_sharp_f > 0 and k >= a.relaxed_sharp_from else a.min_sharp_f
                    jump_ok = not (a.fwd_jump_gate[0] > 0 and var('f') <= a.lo_sharp_sd ** 2
                                   and abs(df) > a.fwd_jump_gate[0] and peak < a.fwd_jump_gate[1])
                    if f_ok and jump_ok and (sf > min_sf or (sf > a.min_sharp_f_lo and var('f') > a.lo_sharp_sd ** 2)):
                        sig = float(np.clip(a.lane_fc / np.sqrt(sf), 0.15, 3.0) if a.lane_fc > 0 else np.clip(a.lane_k[0] / sf, 0.2, 3.0))
                        S = var('f') + sig ** 2
                        big_f.append(np.sign(df) if df * df > 6.25 * S else 0)
                        if len(big_f) >= 4 and abs(sum(big_f[-4:])) == 4:
                            inflate('f', df * df)
                            big_f.clear()
                        D['ln_fa'] = int(upd_fwd(df, sig, robust=True))
                        cnt['fwd'] += D['ln_fa']
                        if a.lo_peak_start and D['ln_fa'] and peak > a.min_peak:
                            lo_ok = False
                    if a.drift_n > 0 and peak > a.min_peak and abs(dl) < win - 0.15:   # wrong-lock detector: weak lane
                        S2 = var('l') + max(a.lane_k[1] / max(sl, 1e-3), 0.1) ** 2   # matches (low sharpness) are not
                        drift_l.append(np.sign(dl) if dl * dl > 6.25 * S2 else 0)    # used, but a run of large
                        if len(drift_l) >= a.drift_n and abs(sum(drift_l[-a.drift_n:])) == a.drift_n:   # same-sign ones
                            inflate('l', dl * dl)                                    # means the filter drifted
                            drift_l.clear()
                            D['drift'] = 1
                    if l_ok and sl > 0:
                        sig = float(np.clip(a.lane_k[1] / sl, 0.1, 1.0))
                        S = var('l') + sig ** 2
                        big_l.append(np.sign(dl) if (dl * dl > 6.25 * S and sl > 0.15) else 0)
                        if len(big_l) >= 4 and abs(sum(big_l[-4:])) == 4:
                            inflate('l', dl * dl)
                            big_l.clear()
                        D['ln_la'] = int(upd_lat(dl, sig, robust=True))
                        cnt['lat'] += D['ln_la']
                    ys_ok = (not y_ok and a.yaw_soft_peak > 0 and travelled >= a.start_dist and peak >= a.yaw_soft_peak
                             and abs(np.degrees(dyaw)) < 3.0 - 0.05)   # weak match: heading with a wider sigma
                    if y_ok or ys_ok:
                        sy = a.yaw_sigma * (1.0 if y_ok else (a.min_peak_yaw / peak) ** 2)
                        D['ln_ya'] = int(upd_yaw(dyaw - np.radians(a.lane_yaw_off), np.radians(sy), robust=True))
                        cnt['yaw'] += D['ln_ya']
                c0, s0 = np.cos(yaw), np.sin(yaw)
                if REPLAY is None:
                    mp = (MP.x[k], MP.y[k], MP.yaw[k]) if MP is not None else (x, y, yaw)
                    yspan = float(np.clip(3 * np.degrees(np.sqrt(var('y'))), 1.0, 3.0)) if MP is None else 3.0
                    rn = None if (a.still_freeze and bool(D.get('still')) and not D.get('still_m')) else \
                        near_measure(k, mp, resp, float(np.clip(3 * np.sqrt(var('l')), 0.4, 1.0)) if MP is None else 1.0, yspan)
                    if rn is not None:
                        cm, sm = np.cos(mp[2]), np.sin(mp[2])
                        REC['near'][k] = (mp[0] - sm * rn[0], mp[1] + cm * rn[0], mp[2] + rn[1], rn[2], rn[3], rn[4],
                                          mp[0] + cm * rn[5], mp[1] + sm * rn[5], rn[6], rn[7])
                        if MP is not None:
                            nx, ny, nyaw = REC['near'][k][:3]
                            fx, fy = REC['near'][k][6:8]
                            rn = (-s0 * (nx - x) + c0 * (ny - y), (nyaw - yaw + np.pi) % (2 * np.pi) - np.pi, rn[2], rn[3], rn[4],
                                  c0 * (fx - x) + s0 * (fy - y), rn[6], rn[7])
                else:
                    rn = None
                    if k in REPLAY['near']:
                        nx, ny, nyaw, sl_, lok, yok = REPLAY['near'][k][:6]
                        rn = (-s0 * (nx - x) + c0 * (ny - y), (nyaw - yaw + np.pi) % (2 * np.pi) - np.pi, sl_, lok, yok, 0.0, 0.0, False)
                if rn is not None:
                    dl2, dyaw2, sl2, l_ok2, y_ok2, df2, sf2, f_ok2 = rn
                    D.update(nr_dl=dl2, nr_dy=np.degrees(dyaw2), nr_sl=sl2, nr_lok=int(l_ok2), nr_yok=int(y_ok2), nr_la=0, nr_ya=0,
                             nr_df=df2, nr_sf=sf2, nr_fok=int(f_ok2), nr_fa=0)
                    if a.near_fwd > 0 and f_ok2 and sf2 > a.near_fwd_min:
                        D['nr_fa'] = int(upd_fwd(df2, float(np.clip(a.near_fwd / sf2, 0.08, 1.0)), robust=True))
                        cnt['fwd'] += D['nr_fa']
                    if l_ok2 and sl2 > 0:
                        D['nr_la'] = int(upd_lat(dl2, float(np.clip(a.near_k / sl2, a.near_min_sigma, 0.6)), robust=True))
                        cnt['lat'] += D['nr_la']
                    if y_ok2 and heading and travelled >= a.start_dist and a.near_yaw_sigma > 0:
                        D['nr_ya'] = int(upd_yaw(dyaw2 - np.radians(a.near_yaw_off), np.radians(a.near_yaw_sigma)))
                        cnt['yaw'] += D['nr_ya']
                if REPLAY is None and a.record and travelled < 3 * a.start_dist:   # record a margin for replays
                    mp = (MP.x[k], MP.y[k], MP.yaw[k]) if MP is not None else (x, y, yaw)
                    dyf_, okf_ = near_yaw(k, mp, span=2.0)
                    if okf_:
                        REC['front'][k] = mp[2] + dyf_
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
                    D['fr_dy'] = np.degrees(dyf) if okf else np.nan
                    if okf:
                        D['fr_a'] = int(upd_yaw(dyf - np.radians(a.near_yaw_off), np.radians(a.start_yaw_sigma)))
                        cnt['yaw'] += D['fr_a']
                floor('f', a.floor[0])
                floor('l', a.floor[1])
                floor('y', np.radians(a.floor[2]))
            out[k] = (x, y, yaw)                   # written once: frame k's estimate uses data up to frame k
            if a.debug:
                D.update(x=x, y=y, yaw=np.degrees(yaw), v=v, ba=b, kvo=kvo, bg=np.degrees(bg), ll=LL, sdf=np.sqrt(var('f')),
                         sdl=np.sqrt(var('l')), sdy=np.degrees(np.sqrt(var('y'))))
                DBG.append(D)
            if a.accum_dist > 0 and REPLAY is None:
                LS.set_pose(k, (x, y, yaw) if MP is None else (MP.x[k], MP.y[k], MP.yaw[k]))
            if k % 300 == 0:
                print(f'  frame {k}: x {x:.2f} y {y:.2f} yaw {np.degrees(yaw):.2f} v {v:.2f} | sd fwd {np.sqrt(var("f")):.2f} '
                      f'lat {np.sqrt(var("l")):.2f} yaw {np.degrees(np.sqrt(var("y"))):.2f} | bg {np.degrees(bg):+.3f} kvo {kvo:.3f} | {cnt}', flush=True)
    # yaw in (-pi, pi] like the ground truth (txt_to_idx_x_y_yaw.py: atan2 of the rotation matrix); the filter heading is
    # continuous (Track03 turns through 180 deg at frame ~2458); values already inside are written unchanged
    yaw_out = out[:, 2] - 2 * np.pi * np.round(out[:, 2] / (2 * np.pi))
    pd.DataFrame({'idx': np.arange(n), 'x': out[:, 0], 'y': out[:, 1], 'yaw': yaw_out}).to_csv(a.out, index=False)
    if a.record:
        np.savez(a.record, rec=np.array(REC, dtype=object))
    if a.debug:
        pd.DataFrame(DBG).to_csv(a.debug, index=False)
    print(f'{T}: {cnt} -> {a.out}')
