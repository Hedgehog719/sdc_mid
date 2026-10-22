#!/bin/bash
# v6 = v5 recipe with the road-voted / near-weighted lane fixes; v7 = v6 + near-range image fixes (near_fix.py)
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_vert.json
L=work/mb/v6; mkdir -p $L
# ---- Track2
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330"
python3 tools/fuse.py /root/data/Track2 work/_v6_p1_t2.csv $C2 --bev work/v6_bm_t2m.npz --rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1 > $L/f_p1_t2.log 2>&1
python3 tools/fix_consistency.py work/v6_bm_t2m.npz work/_v6_p1_t2.csv work/v6g_bm_t2m.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 > $L/gate_t2.log
F2="--rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_v6_p1_t2.csv --imu_acc --rts_qacc 0.15 --nis"
python3 tools/fuse.py /root/data/Track2 work/_v6_t2.csv $C2 --bev work/v6g_bm_t2m.npz $F2 > $L/f_v6_t2.log 2>&1
[ -f work/nf_t2.npz ] && python3 tools/fuse.py /root/data/Track2 work/_v7_t2.csv $C2 --bev work/v6g_bm_t2m.npz $F2 --near work/nf_t2.npz --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 > $L/f_v7_t2.log 2>&1
# ---- Track1
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --start_hold 1.5 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --bev_yaw_skip 0:220"
python3 tools/fuse.py /root/data/Track1 work/_v6_p1_t1.csv $B1 --bev work/v6_bm_t1.npz --rts_sigma 0.5 --rts_qpos 0.4 --rts_qlat 0.1 > $L/f_p1_t1.log 2>&1
python3 tools/fix_consistency.py work/v6_bm_t1.npz work/_v6_p1_t1.csv work/v6g_bm_t1.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 > $L/gate_t1.log
F1="--rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_v6_p1_t1.csv --imu_acc --rts_qacc 0.15 --nis"
python3 tools/fuse.py /root/data/Track1 work/_v6_t1.csv $B1 --bev work/v6g_bm_t1.npz $F1 > $L/f_v6_t1.log 2>&1
[ -f work/nf_t1.npz ] && python3 tools/fuse.py /root/data/Track1 work/_v7_t1.csv $B1 --bev work/v6g_bm_t1.npz $F1 --near work/nf_t1.npz --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 > $L/f_v7_t1.log 2>&1
chown -R 1000:1000 work/_v6* work/_v7* work/v6g* $L
grep -h "NIS\|near fixes\|kept" $L/gate_t2.log $L/f_v6_t2.log $L/f_v7_t2.log $L/gate_t1.log $L/f_v6_t1.log $L/f_v7_t1.log 2>/dev/null | cut -c1-110
echo fuse done
