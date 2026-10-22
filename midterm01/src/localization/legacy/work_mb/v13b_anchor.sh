#!/bin/bash
# v13b: one constant forward offset per track for the lane fixes, anchored to the near fixes (direct projection; the
# front camera's 5-14 m bands agree with each other, i.e. pitch / height fine, and put T2 ~+0.06 m ahead of the
# lane-fix track); fix_anchor --win 100000 = whole-run median of (lane fix - near fix) forward, frames >= 330 only
set -e
cd /root/midterm/src/localization
python3 - <<'PY'
import numpy as np, pandas as pd
for T, lane, near, ref in (('T2', 'work/v10g_bm_t2m.npz', 'work/nf11g_t2.npz', 'work/_v13_t2.csv'), ('T1', 'work/v10g_bm_t1.npz', 'work/nf10_t1.npz', 'work/_v13_t1.csv')):
    R = pd.read_csv(ref)
    def rel(z):
        i = z['idx'].astype(int); yw = R.yaw.values[i]; dx, dy = z['mx'] - R.x.values[i], z['my'] - R.y.values[i]
        return i, np.cos(yw) * dx + np.sin(yw) * dy
    li, lf = rel(dict(np.load(lane))); ni, nf = rel(dict(np.load(near)))
    d = {k: v for k, v in zip(ni, nf)}
    pairs = np.array([lf[j] - d[k] for j, k in enumerate(li) if k in d and k >= 330])
    print(f'{T}: lane - near forward at the same frames (>= 330): median {np.median(pairs):+.3f} mean {pairs.mean():+.3f} (n {len(pairs)}), per 600: ' +
          ' '.join(f'{s0}:{np.median([lf[j] - d[k] for j, k in enumerate(li) if k in d and s0 <= k < s0 + 600]):+.2f}' for s0 in range(330, len(R), 600)))
PY
python3 tools/fix_anchor.py work/v10g_bm_t2m.npz work/nf11g_t2.npz work/_v13_t2.csv work/v13a_bm_t2m.npz --win 100000 --gap 0
python3 tools/fix_anchor.py work/v10g_bm_t1.npz work/nf10_t1.npz work/_v13_t1.csv work/v13a_bm_t1.npz --win 100000 --gap 0
export SDC_ROT_FIX=work/cam_fix_v10t2.json
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --near work/nf11g_t2.npz --bev_fwd_skip 0:330 --yaw_selfcal"
python3 tools/fuse.py /root/data/Track2 work/_v13b_t2.csv $C2 --bev work/v13a_bm_t2m.npz > /dev/null 2>&1
export SDC_ROT_FIX=work/cam_fix_v10t1.json
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --near_yaw_sigma 0.3 --near work/nf10_t1.npz --yaw_selfcal"
python3 tools/fuse.py /root/data/Track1 work/_v13b_t1.csv $B1 --bev work/v13a_bm_t1.npz > /dev/null 2>&1
chown 1000:1000 work/_v13b_t* work/v13a_*
python3 work/mb/est_score.py T2 work/_v13_t2.csv work/_v13b_t2.csv | grep 'estimated\|after'
python3 work/mb/est_score.py T1 work/_v13_t1.csv work/_v13b_t1.csv | grep 'estimated\|after'
