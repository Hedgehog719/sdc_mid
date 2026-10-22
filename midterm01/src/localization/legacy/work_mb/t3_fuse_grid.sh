#!/bin/bash
# GNSS-free pass-2 fusion variants on the rehearsal views (fixes already computed)
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10a.json
mkdir -p work/t3/fg
for N in T3sim2 T3sim1; do
  T=/root/data/$N; P1=work/t3/${N}_p1.csv
  base="--gnss_keep_s 1 --yaw_src imu --rts --bev work/t3/${N}_bmg.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --near work/t3/${N}_nf.npz --near_lat_sigma 0.15 --near_yaw_sigma 0.3 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 2.0 --yaw_rts_base 2 --yaw_selfcal --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv $P1 --imu_acc --rts_qacc 0.15"
  for V in "f0.3_k0.004 --near_fwd_sigma 0.3 --near_aniso_k 0.004" "f0.2_k0.002 --near_fwd_sigma 0.2 --near_aniso_k 0.002" "f0.15_k0.001 --near_fwd_sigma 0.15 --near_aniso_k 0.001" "f0.2_k0.002_skip --near_fwd_sigma 0.2 --near_aniso_k 0.002 --bev_fwd_skip 0:330 --bev_yaw_skip 0:330 --near_yaw_skip 0:330"; do
    set -- $V; name=$1; shift
    python3 tools/fuse.py $T work/t3/fg/${N}_$name.csv $base "$@" > /dev/null 2>&1 &
  done
  wait
done
chown -R 1000:1000 work/t3/fg
for f in work/t3/fg/T3sim2_*.csv; do python3 work/mb/t3_eval.py $f submissions/day8/track2/t2_pipe_v16.csv | head -1 | cut -c1-200; done
for f in work/t3/fg/T3sim1_*.csv; do python3 work/mb/t3_eval.py $f submissions/day8/track1/t1_pipe_v14m.csv | head -1 | cut -c1-200; done
