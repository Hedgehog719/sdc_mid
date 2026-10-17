#!/bin/bash
# Generate leaderboard candidates (variants around the current best of each method) into
# submissions/candidates/track{1,2}/, plus their offline distance to the current best.
cd "$(dirname "$0")/.."
DATA=${DATA:-/root/data}
C1=submissions/candidates/track1; C2=submissions/candidates/track2
mkdir -p $C1 $C2
T1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qpos 0.2 --yaw_bias 0.29"
B1="--bev work/bm_t1.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2"
f1(){ n=$1; shift; python3 tools/fuse.py $DATA/Track1 $C1/$n.csv "$@" >/dev/null; }
# BEV weights
f1 bev_l05_f1.0   $T1 --bev work/bm_t1.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 1.0
f1 bev_l05_f1.5   $T1 --bev work/bm_t1.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 1.5
f1 bev_l05_f3.0   $T1 --bev work/bm_t1.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 3.0
f1 bev_l04_f1.5   $T1 --bev work/bm_t1.npz --bev_lat_sigma 0.4 --bev_fwd_sigma 1.5
f1 bev_huber1.5   $T1 $B1 --bev_huber 1.5
f1 bev_huber3     $T1 $B1 --bev_huber 3
f1 bev_minspd1    $T1 $B1 --bev_min_speed 1
# RTS / yaw with BEV
f1 bev_q0.1       --yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qpos 0.1 --yaw_bias 0.29 $B1
f1 bev_q0.4       --yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qpos 0.4 --yaw_bias 0.29 $B1
f1 bev_cf5        --yaw_src imu_cf --yaw_cf_sigma 5 --rts --rts_qpos 0.2 --yaw_bias 0.29 $B1
f1 bev_cf20       --yaw_src imu_cf --yaw_cf_sigma 20 --rts --rts_qpos 0.2 --yaw_bias 0.29 $B1
f1 bev_cf10motion --yaw_src imu_cf --yaw_cf_sigma 10 --yaw_cf_ref motion --rts --rts_qpos 0.2 --yaw_bias 0.29 $B1
# second BEV iteration / longer accumulation (if computed)
[ -f work/bm_t1_it2.npz ] && f1 bev_it2 $T1 --bev work/bm_t1_it2.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2
[ -f work/bm_t1_acc30.npz ] && f1 bev_acc30 $T1 --bev work/bm_t1_acc30.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2
# Track2
T2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_bias 0.22 --xy_bias 0.041 0.038 --time_shift -0.004 --rts"
f2(){ n=$1; shift; python3 tools/fuse.py $DATA/Track2 $C2/$n.csv "$@" >/dev/null; }
f2 q0.8   $T2 --yaw_cf_sigma 30 --rts_qpos 0.8
f2 q1.6   $T2 --yaw_cf_sigma 30 --rts_qpos 1.6
f2 cf15   $T2 --yaw_cf_sigma 15 --rts_qpos 0.4
f2 cf20   $T2 --yaw_cf_sigma 20 --rts_qpos 0.4
f2 cf45   $T2 --yaw_cf_sigma 45 --rts_qpos 0.4
f2 qa0.5  $T2 --yaw_cf_sigma 30 --rts_qpos 0.4 --rts_qacc 0.5
f2 qa2    $T2 --yaw_cf_sigma 30 --rts_qpos 0.4 --rts_qacc 2.0
f2 gs0.15 $T2 --yaw_cf_sigma 30 --rts_qpos 0.4 --rts_sigma 0.15
python3 - <<'PY'
import glob, os
import numpy as np, pandas as pd
for d, best in [('track1', 'submissions/track1/1.00423__t1_bev_l05f2.csv'), ('track2', 'submissions/track2/0.33283__t2_bias_lag.csv')]:
    a = pd.read_csv(best)
    print(f'== {d} (vs {os.path.basename(best)})')
    for f in sorted(glob.glob(f'submissions/candidates/{d}/*.csv')):
        b = pd.read_csv(f)
        dp = np.hypot(b.x - a.x, b.y - a.y)
        dy = np.degrees((b.yaw - a.yaw + np.pi) % (2 * np.pi) - np.pi)
        print(f'  {os.path.basename(f):22s} dpos {np.sqrt(np.mean(dp**2)):.3f} m  dyaw {np.sqrt(np.mean(dy**2)):.3f} deg')
PY
