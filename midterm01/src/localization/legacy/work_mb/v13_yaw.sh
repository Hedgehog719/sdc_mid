#!/bin/bash
# v13: camera-heading self-calibration in the yaw smoother (fuse --yaw_selfcal): T2 near / lane headings sit +0.14 /
# +0.29 deg off the course of the fused path (T1 -0.01 / +0.18) -> T2 yaw +0.13 deg off its own direction of travel
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --bev work/v10g_bm_t2m.npz --near work/nf11g_t2.npz --bev_fwd_skip 0:330"
python3 tools/fuse.py /root/data/Track2 work/_v13_t2.csv $C2 --yaw_selfcal 2>&1 | grep "selfcal"
export SDC_ROT_FIX=work/cam_fix_v10t1.json
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --near_yaw_sigma 0.3 --bev work/v10g_bm_t1.npz --near work/nf10_t1.npz"
python3 tools/fuse.py /root/data/Track1 work/_v13_t1.csv $B1 --yaw_selfcal 2>&1 | grep "selfcal"
chown 1000:1000 work/_v13_t*
python3 - <<'PY'
import numpy as np, pandas as pd
from scipy.ndimage import uniform_filter1d
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
for T, fs, best in (('T2', ('work/_v11g_t2.csv', 'work/_v13_t2.csv'), 'submissions/track2/0.17429__t2_opt10.csv'),
                    ('T1', ('work/_v10_t1.csv', 'work/_v13_t1.csv'), 'submissions/track1/0.41123__t1_opt11.csv')):
    B = pd.read_csv(best)
    for f in fs:
        D = pd.read_csv(f); xs, ys = uniform_filter1d(D.x.values, 11, mode='nearest'), uniform_filter1d(D.y.values, 11, mode='nearest')
        c = np.arctan2(np.gradient(ys), np.gradient(xs)); m = np.hypot(np.gradient(xs), np.gradient(ys)) > 0.15
        d = np.degrees(wrap(D.yaw.values - c)); db = np.degrees(wrap(D.yaw.values - B.yaw.values))
        pos = np.hypot(D.x.values - pd.read_csv(fs[0]).x.values, D.y.values - pd.read_csv(fs[0]).y.values).max()
        print(f'{T} {f}: yaw - course mean {d[m].mean():+.3f} rms {np.sqrt(np.mean(d[m]**2)):.3f} | yaw - best (330+) mean {db[330:].mean():+.3f} rms {np.sqrt(np.mean(db[330:]**2)):.3f} | max position change {pos:.3f} m')
PY
python3 work/mb/est_score.py T2 work/_v11g_t2.csv work/_v13_t2.csv | grep 'estimated'
python3 work/mb/est_score.py T1 work/_v10_t1.csv work/_v13_t1.csv | grep 'estimated'
