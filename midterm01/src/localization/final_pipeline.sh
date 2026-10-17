#!/bin/bash
# "Most complete" pipeline (all refinements found so far) -> submissions/final/final_{1,2}.csv
# Run inside the container. Heavy BEV matching is cached in work/bm_t{1,2}_final.npz.
set -e
cd "$(dirname "$0")"
DATA=${DATA:-/root/data}
OUT=submissions/final; mkdir -p $OUT work
BEVOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1"

# ---------- Track1: GNSS drifts (urban multipath) -> map-to-image fixes matter ----------
# prior trajectory (GNSS + IMU, RTS) for rendering the map around the vehicle
python3 tools/fuse.py $DATA/Track1 work/prior_1.csv --yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qpos 0.2 --yaw_bias 0.29
[ -f work/bm_t1_final.npz ] || python3 tools/bev_match.py $DATA/Track1 work/prior_1.csv work/bm_t1_final.npz --frames 0:3011:3 $BEVOPT
#  positions: GNSS + BEV fixes (lat 0.5 / fwd 2.0 m, scaled by match quality, Huber 1.5) with a no-side-slip RTS
#  yaw: IMU yaw + RTS offset from the speed-weighted heading of the fused path + BEV lane-alignment yaw
python3 tools/fuse.py $DATA/Track1 $OUT/final_1.csv --yaw_src imu_cf --yaw_cf_sigma 10 --yaw_bias 0.29 \
    --rts --rts_qpos 0.4 --rts_qlat 0.1 \
    --bev work/bm_t1_final.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 \
    --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 1.5
python3 tools/align_yaw_mean.py $OUT/final_1.csv submissions/track1/1.00423__t1_bev_l05f2.csv

# ---------- Track2: GNSS already accurate -> sensor fusion refinements only ----------
python3 tools/fuse.py $DATA/Track2 $OUT/final_2.csv --yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 \
    --yaw_bias 0.22 --xy_bias 0.041 0.038 --time_shift -0.004 \
    --rts --rts_qpos 0.8 --rts_qlat 0.2 \
    --yaw_rts gnss --yaw_rts_q 0.03
python3 tools/align_yaw_mean.py $OUT/final_2.csv submissions/track2/0.33283__t2_bias_lag.csv
echo "wrote $OUT/final_1.csv $OUT/final_2.csv"
