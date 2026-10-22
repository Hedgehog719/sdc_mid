#!/bin/bash
# T2: forward-only map-bias state (lane / near / pole / kerb fixes share a slowly varying along-track error, GNSS can
# correct it); grid over correlation length, along-track sigma, GNSS bias sigma; evaluation: estimate (reference only)
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
mkdir -p work/gmb
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --near work/nf11g_t2.npz --bev_fwd_skip 0:330 --yaw_selfcal --bev work/v10g_bm_t2m.npz"
run() { python3 tools/fuse.py /root/data/Track2 work/gmb/$1.csv $C2 $2 > /dev/null 2>&1; }
N=0
for GS in 0.14 0.07; do
  for L in 30 80 200; do
    for SF in 0.05 0.10 0.15; do
      run "g${GS}_L${L}_f${SF}" "--gnss_bias_sigma $GS --map_bias_len $L --map_bias_sigma $SF --map_bias_sigma_lat 0.02" &
      N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
    done
  done
  run "g${GS}_none" "--gnss_bias_sigma $GS" &
  N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
done
wait
chown -R 1000:1000 work/gmb
python3 work/mb/est_score.py T2 work/_v13_t2.csv $(ls work/gmb/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.17429\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $7, $11, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n
