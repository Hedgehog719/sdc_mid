#!/bin/bash
# v14m T1 (pipeline, global parameters): v13 recipe (lane v10g, near nf10, kerb, yaw selfcal) with the GNSS bias
# sigma 0.4 -> 0.15 (tau 5 s, white 0.02); map bias isotropic 1000 m / 0.03 as before
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t1.json
OUT=${1:-work/_v14m_t1.csv}
python3 tools/fuse.py /root/data/Track1 $OUT --yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --near_yaw_sigma 0.3 --near work/nf10_t1.npz --yaw_selfcal --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --gnss_bias_tau 5 --gnss_bias_sigma 0.15 --gnss_white 0.02 --bev work/v10g_bm_t1.npz --nis 2>&1 | grep "NIS\|selfcal" | cut -c1-90
chown 1000:1000 $OUT
