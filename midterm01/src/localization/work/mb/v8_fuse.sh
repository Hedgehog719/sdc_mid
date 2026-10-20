#!/bin/bash
# v8: re-calibrated side cameras (cam_fix_v8) for lane + near fixes; recipes of t2_pipe_v5n (0.21) / t1 v6bn2 nohold (0.43)
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v8.json
python3 tools/fix_consistency.py work/v8_bm_t2m.npz work/_c16_p1.csv work/v8g_bm_t2m.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
python3 tools/fix_consistency.py work/v8_bm_t1.npz work/_c15_p1.csv work/v8g_bm_t1.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004"
python3 tools/fuse.py /root/data/Track2 work/_v8_t2.csv $C2 --bev work/v8g_bm_t2m.npz --near work/nf8_t2.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5"
python3 tools/fuse.py /root/data/Track1 work/_v8_t1.csv $B1 --bev work/v8g_bm_t1.npz --near work/nf8_t1.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
chown 1000:1000 work/_v8_t* work/v8g_*
python3 work/mb/est_score.py T2 work/_v5n_t2.csv work/_v8n_t2.csv work/_v8_t2.csv | grep -v "vs best, all"
python3 work/mb/est_score.py T1 work/_t1nh.csv work/_t1n8.csv work/_v8_t1.csv | grep -v "vs best, all"
python3 tools/near_align.py /root/data/Track2 work/mk16v_D.npz work/_v5n_t2.csv work/_v8n_t2.csv work/_v8_t2.csv --frames 0:-1:9 --workers 4 2>&1 | tail -3
python3 tools/near_align.py /root/data/Track1 work/mk1_15v_D.npz work/_t1nh.csv work/_t1n8.csv work/_v8_t1.csv --frames 0:-1:9 --workers 4 2>&1 | tail -3
