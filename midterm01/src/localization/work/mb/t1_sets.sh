#!/bin/bash
# T1 lane-fix sets under the new GNSS model (tau 5, sigma 0.15, white 0.02)
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t1.json
mkdir -p work/t1s
B0="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --near_yaw_sigma 0.3 --near work/nf10_t1.npz --yaw_selfcal --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --gnss_bias_tau 5 --gnss_bias_sigma 0.15 --gnss_white 0.02"
run() { python3 tools/fuse.py /root/data/Track1 work/t1s/$1.csv $B0 --bev work/$1.npz > /dev/null 2>&1; }
N=0
for S in v10g_bm_t1 v8g_bm_t1 v6bg_bm_t1 v9g_bm_t1 vert_bm_t1 mk1S_bm_t1 c15g_bm_t1 c15gc_bm_t1; do
  [ -f work/$S.npz ] || continue
  run $S &
  N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
done
wait
chown -R 1000:1000 work/t1s
python3 work/mb/est_score.py T1 $(ls work/t1s/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.41123\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $6, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n
