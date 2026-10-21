#!/bin/bash
# v16 T2 (v14m + GNSS tau 10 s, near fixes accel-corrected against GNSS with more forward weight, path-frame map bias) (pipeline, global parameters): lane fixes = BEV match on the clean map / mk_S marks (cam_fix_vert),
# near fixes = nf11g (front camera at the start on the plate map, all cameras after 330), fusion model:
#  - GNSS: white 0.08 m + Gauss-Markov bias (tau 20 s, sigma 0.07 m)
#  - map fixes share a slowly varying error: along-track sigma 0.05 m / cross-track 0.02 m, length 1000 m
#  - start (0-330, stationary / repaired map plate): lane fixes lateral-only, no camera headings
#  - camera heading offsets self-calibrated against the course of the fused path
# usage: v14m_fuse.sh <lane_fix_npz> <out.csv>
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
LANE=${1:-work/vert_bm_t2m.npz}; OUT=${2:-work/_v14m_t2.csv}
python3 tools/fuse.py /root/data/Track2 $OUT --yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_lat_sigma 0.6 --pole_fwd_sigma 0.25 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 2.0 --yaw_rts_base 2 --near_yaw_sigma 0.3 --gnss_bias_tau 10 --gnss_white 0.08 --gnss_bias_sigma 0.07 --map_bias_len 500 --map_bias_sigma 0.05 --map_bias_sigma_lat 0.0 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.2 --near_lat_sigma 0.15 --near_aniso_k 0.002 --near work/nf11ga_t2.npz --bev_fwd_skip 0:330 --near_yaw_skip 0:330 --yaw_selfcal --bev $LANE --nis 2>&1 | grep "NIS\|selfcal" | cut -c1-90
chown 1000:1000 $OUT
