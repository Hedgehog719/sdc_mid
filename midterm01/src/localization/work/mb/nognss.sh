#!/bin/bash
# GNSS-free rehearsal for Track03: GNSS only for the first second (initial guess), heading = IMU aligned at the start
# then refined by the yaw smoother (lane-fix headings + course of the fused path); positions from lane / near / pole /
# kerb fixes + IMU acceleration. Same fixes as the submitted GNSS versions (t2_pipe_v5n 0.21, t1_pipe_v6bn2 0.43).
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_vert.json
L=work/mb/nognss; mkdir -p $L
NG="--gnss_keep_s 1 --yaw_src imu --rts"
# ---- Track2
C2="$NG --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --bev_yaw_skip 0:330"
python3 tools/fuse.py /root/data/Track2 work/_ng_p1_t2.csv $C2 --bev work/c16p20_10_bm_t2m.npz --rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1 > $L/p1_t2.log 2>&1
python3 tools/fix_consistency.py work/c16p20_10_bm_t2m.npz work/_ng_p1_t2.csv work/ngg_bm_t2.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 > $L/gate_t2.log
python3 tools/fuse.py /root/data/Track2 work/_ng_t2.csv $C2 --bev work/ngg_bm_t2.npz --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_ng_p1_t2.csv --imu_acc --rts_qacc 0.15 --near work/nf_t2.npz --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --nis > $L/f_t2.log 2>&1
# ---- Track1
B1="$NG --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --start_hold 1.5 --map_bias_len 1000 --map_bias_sigma 0.03 --bev_yaw_skip 0:220"
python3 tools/fuse.py /root/data/Track1 work/_ng_p1_t1.csv $B1 --bev work/v6b_bm_t1.npz --rts_sigma 0.5 --rts_qpos 0.4 --rts_qlat 0.1 > $L/p1_t1.log 2>&1
python3 tools/fix_consistency.py work/v6b_bm_t1.npz work/_ng_p1_t1.csv work/ngg_bm_t1.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 > $L/gate_t1.log
python3 tools/fuse.py /root/data/Track1 work/_ng_t1.csv $B1 --bev work/ngg_bm_t1.npz --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_ng_p1_t1.csv --imu_acc --rts_qacc 0.15 --near work/nf2_t1.npz --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --nis > $L/f_t1.log 2>&1
chown -R 1000:1000 work/_ng_* work/ngg_* $L
grep -h "GNSS-free\|NIS\|kept" $L/*.log | cut -c1-100
python3 - <<'PY'
import numpy as np, pandas as pd
for T, a, b in (('T2', 'work/_v5n_t2.csv', 'work/_ng_t2.csv'), ('T1', 'work/_v6bn2_t1.csv', 'work/_ng_t1.csv')):
    A, B = pd.read_csv(a), pd.read_csv(b); yw = A.yaw.values
    dx, dy = B.x.values - A.x.values, B.y.values - A.y.values
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
    da = np.degrees((B.yaw.values - yw + np.pi) % (2 * np.pi) - np.pi)
    print(f'{T} GNSS-free vs GNSS version: fwd rms {np.sqrt(np.mean(fw**2)):.3f} (max {np.abs(fw).max():.2f}) lat rms {np.sqrt(np.mean(lt**2)):.3f} (max {np.abs(lt).max():.2f}) yaw rms {np.sqrt(np.mean(da**2)):.3f} deg')
    print('   per 300 frames fwd/lat mean: ' + ' | '.join(f'{s0}: {fw[s0:s0+300].mean():+.2f}/{lt[s0:s0+300].mean():+.2f}' for s0 in range(0, len(A), 300)))
PY
