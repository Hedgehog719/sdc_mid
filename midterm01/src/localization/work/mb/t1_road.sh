#!/bin/bash
# T1: kerb (road) fixes from the old clean6 map pull right ~0.24 m; variants without / weaker, + vert lane fixes
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t1.json
mkdir -p work/t1r
B0="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --near_yaw_sigma 0.3 --near work/nf10_t1.npz --yaw_selfcal"
run() { python3 tools/fuse.py /root/data/Track1 work/t1r/$1.csv $B0 $2 > /dev/null 2>&1; }
run base "--road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --bev work/v10g_bm_t1.npz" &
run noroad "--bev work/v10g_bm_t1.npz" &
run road1.0 "--road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 1.0 --bev work/v10g_bm_t1.npz" &
run road_p80 "--road work/c6_rds_t1.npz --road_pct 80 --road_lat_sigma 0.5 --bev work/v10g_bm_t1.npz" &
wait
run vert_noroad "--bev work/vert_bm_t1.npz" &
run vert_road "--road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --bev work/vert_bm_t1.npz" &
run v6b_noroad "--bev work/v6bg_bm_t1.npz" &
run mk1S_noroad "--bev work/mk1S_bm_t1.npz" &
wait
chown -R 1000:1000 work/t1r
python3 work/mb/est_score.py T1 $(ls work/t1r/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.41123\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n
