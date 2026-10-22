#!/bin/bash
# T2 pole weight grid on the vert + map-bias model
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
mkdir -p work/gp
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 2.0 --yaw_rts_base 2 --near_yaw_sigma 0.3 --gnss_bias_tau 20 --gnss_white 0.08 --gnss_bias_sigma 0.07 --map_bias_len 1000 --map_bias_sigma 0.05 --map_bias_sigma_lat 0.02 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near work/nf11g_t2.npz --bev_fwd_skip 0:330 --bev work/vert_bm_t2m.npz --yaw_selfcal --near_yaw_skip 0:330"
run() { python3 tools/fuse.py /root/data/Track2 work/gp/$1.csv $C2 $2 > /dev/null 2>&1; }
N=0
for PP in 40 60 80; do
  for PF in 0.15 0.25 0.4; do
    for PL in 0.3 0.6 1.0; do
      run "pct${PP}_f${PF}_l${PL}" "--pole_pct $PP --pole_fwd_sigma $PF --pole_lat_sigma $PL" &
      N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
    done
  done
done
wait
chown -R 1000:1000 work/gp
python3 work/mb/est_score.py T2 $(ls work/gp/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.17429\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n | head -10
