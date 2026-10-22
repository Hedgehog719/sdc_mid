#!/bin/bash
# v10: fr re-calibrated on the near range (cam_fix_v10$V); near fixes redone with it, lane fixes still v8 (gated v8g);
# fusion = the v8y recipes unchanged
set -e
cd /root/midterm/src/localization
V=${1:-a}
export SDC_ROT_FIX=work/cam_fix_v10$V.json
python3 tools/near_fix.py /root/data/Track2 work/mk16v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:-1:3 --workers 4 --yaw_search 1.0 --out work/nf10${V}_t2.npz 2>&1 | tail -1
python3 tools/near_fix.py /root/data/Track1 work/mk1_15v_D.npz submissions/day7/track1/t1_pipe_v5.csv --frames 0:-1:3 --workers 4 --bg_weight 1.0 --yaw_search 1.0 --out work/nf10${V}_t1.npz 2>&1 | tail -1
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --bev work/v8g_bm_t2m.npz --near_yaw_sigma 0.3"
python3 tools/fuse.py /root/data/Track2 work/_v10${V}_t2.csv $C2 --near work/nf10${V}_t2.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --bev work/v8g_bm_t1.npz --near_yaw_sigma 0.3"
python3 tools/fuse.py /root/data/Track1 work/_v10${V}_t1.csv $B1 --near work/nf10${V}_t1.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
chown 1000:1000 work/_v10${V}_t* work/nf10${V}_t*
python3 - <<PY
import numpy as np, pandas as pd
for T in ('t2', 't1'):
    A, B = pd.read_csv(f'work/_v8y_{T}.csv'), pd.read_csv(f'work/_v10${V}_{T}.csv'); yw = A.yaw.values
    dx, dy = B.x.values - A.x.values, B.y.values - A.y.values
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
    print(f'{T} v10${V} - v8y: fwd mean {fw.mean():+.3f} rms {np.sqrt(np.mean(fw**2)):.3f} | lat mean {lt.mean():+.3f} rms {np.sqrt(np.mean(lt**2)):.3f}')
    print('   per 300 fwd/lat: ' + ' | '.join(f'{s0}: {fw[s0:s0+300].mean():+.2f}/{lt[s0:s0+300].mean():+.2f}' for s0 in range(0, len(A), 300)))
PY
python3 work/mb/est_score.py T2 work/_v8y_t2.csv work/_v10${V}_t2.csv | grep estimated
python3 work/mb/est_score.py T1 work/_v8y_t1.csv work/_v10${V}_t1.csv | grep estimated
