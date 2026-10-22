#!/bin/bash
# Online (causal) localization of one track, end to end. Competition rules of 10/22: the pose of frame k is estimated
# from frames 1..k only and written once; no backward smoothing, no offline correction.
#
# usage: tools/run_online.sh <track_dir> <T1|T2|T3> [out.csv]
#   track_dir  the Kaggle data folder: raw_ply/ (frame times), <name>_images/ (3 cameras + camera_info), map.pcd,
#              init_guess.txt, and the bag (<name>.db3 + metadata.yaml) with /imu/data [and the GNSS pose, T1/T2]
#   out.csv    Kaggle format idx,x,y,yaw (default work/runs/<T>/<T>_online.csv)
#
# Steps (a step is skipped when its output already exists):
#   1. map.pcd -> map.npy + map_intensity.npy; IMU (and GNSS) messages from the bag -> sensors/*.csv
#   2. per image, each frame on its own: SegFormer road probability (road/), YOLO vehicle masks (masks/)
#   3. road-marking maps from the whole LiDAR map (no trajectory involved)
#   4. road-surface visual odometry of consecutive frame pairs (k-1, k)
#   5. tools/localize_online.py: one forward Kalman filter over the frames in time order
set -e
cd "$(dirname "$0")/.."
TD=$(realpath "$1")
TR=$2
W=work/runs/$TR
OUT=${3:-$W/${TR}_online.csv}
mkdir -p "$W"
N=$(basename "$TD")

# per-track settings: camera mount corrections (calibration), GNSS use, gyro bias (calibration) and measurement noise
# filter: joint EKF, standstill update, no heading from the near alignment (it returns its prior), lane heading
# sigma 0.7 deg with its calibrated +0.30 deg offset, forward sigma 0.05/sqrt(sharpness), Huber weights
COMMON="--step 3 --joint --zupt --near_yaw_sigma 0 --yaw_sigma 0.7 --lane_yaw_off 0.3 --near_yaw_off 0
  --lane_fc 0.05 --min_sharp_f 0.02 --min_sharp_f_lo 0.005 --lo_sharp_sd 0.35 --huber 2.5
  --gyro_bias_sd 0.01 --gyro_bias_walk 0.0005 --floor 0.2 0.1 0.15 --start_dist 0 --near_k 0.006"
case $TR in
  T1) export SDC_ROT_FIX=calib/cam_fix_v10t1.json
      P="--gnss --gnss_sigma 1.0 0.2 --gyro_bias -0.220 --lane_k 0.005 0.005 --vo_noise 0.02 --acc_noise 0.05" ;;
  T2) export SDC_ROT_FIX=calib/cam_fix_v10t2.json
      P="--gnss --gnss_sigma 0.15 0.2 --gyro_bias -0.247 --lane_k 0.02 0.01 --vo_noise 0.08 --acc_noise 0.05" ;;
  T3) export SDC_ROT_FIX=calib/cam_fix_v10a.json   # no GNSS: forward fixes down to sharpness 0.005 always (stop drift)
      P="--gyro_bias -0.23 --lane_k 0.005 0.005 --vo_noise 0.08 --acc_noise 0.1 --min_sharp_f 0.005 --v0_sd 10" ;;
      # T3 starts moving (~8 m/s at frame 0): unknown initial speed, the first VO pair sets it
  *) echo "track must be T1, T2 or T3"; exit 1 ;;
esac

echo "[$(date +%H:%M:%S)] 1. map and sensor data"
[ -f "$TD/map.npy" ] || python3 tools/map_to_npy.py "$TD/map.pcd" "$TD/map.npy"
if [ ! -s "$TD/sensors/imu.csv" ] && ls "$TD"/*.db3 > /dev/null 2>&1; then
  python3 tools/extract_bag.py "$TD" "$TD/sensors" || echo "bag extraction failed: running without IMU"
fi
IMU=""; [ -s "$TD/sensors/imu.csv" ] || IMU="--no_imu"
if [ "$TR" != T3 ] && [ ! -s "$TD/sensors/gnss.csv" ]; then echo "no GNSS for $TR"; exit 1; fi

echo "[$(date +%H:%M:%S)] 2. road segmentation and vehicle masks (per image)"
[ -f "$TD/road/f.npz" ] || python3 tools/segment_road.py "$TD"
YW=${YOLO_WEIGHTS:-yolo11x-seg.pt}   # ultralytics downloads it when it is not a local file
[ -z "$YOLO_WEIGHTS" ] && [ -f /root/data/weights/yolo11x-seg.pt ] && YW=/root/data/weights/yolo11x-seg.pt
[ -f "$TD/masks/f.npz" ] || python3 tools/segment_vehicles.py "$TD" --model "$YW"

echo "[$(date +%H:%M:%S)] 3. road-marking maps (whole map)"
[ -f "$W/mk_D.npz" ] || python3 tools/marking_map.py "$TD" ALL "$W/mk_D.npz" --hi 2.5 --lo 1.2 --cell 0.7 > "$W/mk_D.log" 2>&1
[ -f "$W/mk_S.npz" ] || python3 tools/marking_map.py "$TD" ALL "$W/mk_S.npz" --mode soft > "$W/mk_S.log" 2>&1

echo "[$(date +%H:%M:%S)] 4. visual odometry"
[ -f "$W/vo.npz" ] || python3 tools/bev_vo.py "$TD" "$W/vo.npz" $IMU | tail -n 1

echo "[$(date +%H:%M:%S)] 5. online localization ($N)"
python3 tools/localize_online.py "$TD" "$W/mk_D.npz" "$W/mk_S.npz" "$OUT" --vo "$W/vo.npz" $IMU $COMMON $P \
  --debug "$W/debug.csv" > "$W/localize.log" 2>&1
tail -n 1 "$W/localize.log"
echo "[$(date +%H:%M:%S)] done -> $OUT"
