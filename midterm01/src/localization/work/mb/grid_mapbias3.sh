#!/bin/bash
# round 3 (T2): GNSS noise model (white vs bias), map bias on / off, pole forward weight
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
mkdir -p work/gmb3
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --near work/nf11g_t2.npz --bev_fwd_skip 0:330 --yaw_selfcal --bev work/v10g_bm_t2m.npz"
run() { python3 tools/fuse.py /root/data/Track2 work/gmb3/$1.csv $C2 $2 > /dev/null 2>&1; }
N=0
for GW in 0.03 0.08 0.15; do
  for GS in 0.03 0.05 0.07; do
    for MB in on off; do
      for PF in 0.4 0.25; do
        X="--gnss_white $GW --gnss_bias_sigma $GS --pole_fwd_sigma $PF"
        [ $MB = on ] && X="$X --map_bias_len 500 --map_bias_sigma 0.05 --map_bias_sigma_lat 0.02"
        run "w${GW}_g${GS}_mb${MB}_p${PF}" "$X" &
        N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
      done
    done
  done
done
wait
chown -R 1000:1000 work/gmb3
python3 work/mb/est_score.py T2 $(ls work/gmb3/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.17429\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n | head -20
