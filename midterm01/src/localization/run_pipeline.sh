#!/bin/bash
# Reproduce the submission CSVs inside the container:  bash /root/midterm/src/localization/run_pipeline.sh
# Optional heavy stages (map matching experiments) are enabled with FULL=1.
set -e
cd "$(dirname "$0")"
DATA=${DATA:-/root/data}
mkdir -p result work

for T in 1 2; do
  TD=$DATA/Track$T
  [ -f $TD/sensors/gnss.csv ] || python3 tools/extract_bag.py $TD $TD/sensors
  if [ "${FULL:-0}" = 1 ]; then
    [ -f $TD/map.npy ] || python3 tools/map_to_npy.py $TD/map.pcd $TD/map.npy
    [ -f $TD/masks/f.npz ] || python3 tools/segment_vehicles.py $TD --model $DATA/weights/yolo11x-seg.pt
    python3 tools/register.py $TD work/t${T}_full.npz --masks --mcds 1.5 1.0 0.5
  fi
done

# Best fusion settings found on the leaderboard (see README.md)
python3 tools/fuse.py $DATA/Track1 result/result_1.csv --yaw_src imu_cf --yaw_cf_sigma 10 --rts --rts_qpos 0.2
python3 tools/fuse.py $DATA/Track2 result/result_2.csv --yaw_src imu_cf --yaw_cf_sigma 30 --yaw_cf_ref motion --rts --rts_qpos 0.4
echo "wrote result/result_1.csv result/result_2.csv"
