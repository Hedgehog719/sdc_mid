#!/bin/bash
# T1 round 2: GNSS bias sigma / tau / white with the isotropic map bias (1000 m, 0.03)
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t1.json
mkdir -p work/t1m2
B0="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --near_yaw_sigma 0.3 --near work/nf10_t1.npz --yaw_selfcal --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --bev work/v10g_bm_t1.npz"
run() { python3 tools/fuse.py /root/data/Track1 work/t1m2/$1.csv $B0 $2 > /dev/null 2>&1; }
N=0
for TAU in 5 20; do
  for GS in 0.15 0.1 0.07; do
    for GW in 0.02 0.08 0.15; do
      run "t${TAU}_g${GS}_w${GW}" "--gnss_bias_tau $TAU --gnss_bias_sigma $GS --gnss_white $GW" &
      N=$((N+1)); if [ $((N % 4)) -eq 0 ]; then wait; fi
    done
  done
done
wait
chown -R 1000:1000 work/t1m2
python3 work/mb/est_score.py T1 $(ls work/t1m2/*.csv) 2>&1 | grep "estimated\|after 330" | paste - - | sed 's/(in span [0-9]* %)//g; s/\[current best 0.41123\]//; s/   vs best, after 330   ://' | awk '{print $1, $4, $6, $7, $13, $14, $15, $16, $17, $18, $19, $20, $21, $22}' | sort -k2 -n | head -12
