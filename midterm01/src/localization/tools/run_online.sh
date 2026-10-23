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
# sigma 0.7 deg with its calibrated +0.30 deg offset, forward sigma 0.05/sqrt(sharpness), Huber weights;
# --imu_causal: IMU integrals from the last sample at or before the frame time (extrapolated with its rate)
# --still_freeze (T3): no map measurements while standing (40 s red light: pose wandered 0.55 m / 2.2 deg)
# --terrain (T3): the image BEV is projected onto the map road surface instead of a flat plane
COMMON="--step 3 --joint --zupt --near_yaw_sigma 0 --yaw_sigma 0.7 --lane_yaw_off 0.3 --near_yaw_off 0
  --lane_fc 0.05 --min_sharp_f 0.02 --min_sharp_f_lo 0.005 --lo_sharp_sd 0.35 --huber 2.5
  --gyro_bias_sd 0.01 --gyro_bias_walk 0.0005 --floor 0.2 0.1 0.15 --start_dist 0 --near_k 0.006"
case $TR in
  T1) export SDC_ROT_FIX=calib/cam_fix_v10t1.json
      P="--gnss --gnss_sigma 1.0 0.2 --gyro_bias -0.220 --lane_k 0.005 0.005 --vo_noise 0.02 --acc_noise 0.05 --imu_causal" ;;
  T2) export SDC_ROT_FIX=calib/cam_fix_v10t2.json
      P="--gnss --gnss_sigma 0.15 0.2 --gyro_bias -0.247 --lane_k 0.02 0.01 --vo_noise 0.15 --acc_noise 0.05 --floor 0.1 0.1 0.15
         --stand_search 30 --stand_search_sigma 2.5 --imu_causal" ;;
      # T2 starts standing in a queue (bus alongside, markings hidden): the frame-0 heading search is 1.8 deg off ->
      # repeat it every 30 frames while standing with an uncertain heading (later frames see other markings)
  T3) export SDC_ROT_FIX=calib/cam_fix_v10a.json   # no GNSS
      P="--gyro_bias -0.23 --lane_k 0.005 0.005 --vo_noise 0.08 --acc_noise 0.1 --v0_sd 10 --min_peak_lo 0.13 --lo_peak_sd 0.5 99
         --still_freeze --imu_causal --terrain --dyn_att --dyn_roll_scale 0 --pitch_per_accel -0.3 --init_pos 2.5 --vo_dr_init
         --att_imu_map 1.5 --att_resid --att_pitch_only --roll_imu_map 120" ;;
      # --roll_imu_map 120: static body roll relative to the road, estimated online as the running mean of IMU roll minus
      # the map road-plane roll over the frames so far (+0.32 deg at frame 100, +0.36..+0.42 after; the T1 / T2 camera
      # calibration assumes none): lane lat 0.106 -> 0.099 m, lane fwd 0.096 -> 0.085, near lat 0.066 -> 0.064, lane
      # heading 0.43 -> 0.37 deg, more matches pass the gates; -0.37 deg (wrong sign) is worse in all of them
      # --att_imu_map 1.5 --att_resid --att_pitch_only: fast body pitch on uneven road (a wheel over a bump, under-damped
      # suspension; gyro: 0.9 deg nose-up at 617): IMU pitch minus the map road plane under the prior pose, minus the
      # dive model, high-passed with a 1.5 s causal average (the AHRS pitch drifts 0.5 deg after braking -> slow part
      # not used), added to the dive model; fair innovations near lateral 0.082 -> 0.066 m, lane heading 0.77 -> 0.46
      # deg, same stop pose (roll from the IMU and longer / shorter averages were worse)
      # --vo_dr_init: until 5 VO pairs give the initial speed (frame 5), move by each pair's VO displacement (the position
      # stood still for 4 frames while the car drove ~1.1 m; the first frame intervals are irregular)
      # --dyn_att --dyn_roll_scale 0 --pitch_per_accel -0.3: body pitch from the IMU longitudinal acceleration of the past
      # 0.5 s (braking dive / squat, 0.3 deg per m/s^2, a typical car value); on top of the terrain it raises the map
      # likelihood of the braking sections, and the frame-0 heading search sees the squat (the car accelerates at
      # +0.9 m/s^2 at frame 0); --init_pos 2.5: frame-0 position from the lane match at the searched heading
      # (init_guess is off by ~1 m)
      # --terrain: lane matching on the map road surface (vertical curves ahead of +-0.1..0.2 m at 15-20 m stretch a
      # flat-ground BEV by 1-2 m); the Track03 map road profile is smooth (slopes < 2 deg), unlike the Track01/02 maps
      # (start areas warped by up to 12 deg), where it lowers the lane-match peaks -> T3 only
      # (taking every low-sharpness forward fix, --min_sharp_f 0.005, let the position creep 0.8 m during a 40 s red-light
      # stop - the same image measured hundreds of times - and had the lower map likelihood where the two versions differ)
      # T3 starts moving (~8 m/s at frame 0): unknown initial speed (median of the first VO pairs); while the position is
      # uncertain (start) weaker lane matches count for the forward direction (peak > 0.13): forward lock at frame 6
      # instead of 60 (for the lateral direction they caused a 1 m wrong lock in the T1 rehearsal)
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
LOC="python3 tools/localize_online.py $TD $W/mk_D.npz $W/mk_S.npz"
if [ "$TR" = T3 ]; then
  # no GNSS and init_guess off by ~17 deg: the frame-0 heading search is flat over several degrees and a filter started
  # a few degrees off may never lock -> bank of heading hypotheses (frame-0 search + offsets) for the first NH frames,
  # per-frame causal selection by accepted map measurements, the winner continues (tools/hyp_select.py)
  NH=600; HYP="0 -1.5 1.5 -3 3"; PAIRS=""; i=0
  for o in $HYP; do
    $LOC $W/hyp_$o.csv --vo $W/vo.npz $IMU $COMMON $P --init_offset $o --frames $NH --debug $W/hyp_${o}_dbg.csv > $W/hyp_$o.log 2>&1 &
    PAIRS="$PAIRS $W/hyp_$o.csv:$W/hyp_${o}_dbg.csv"; i=$((i + 1))
    [ $((i % 3)) -eq 0 ] && wait     # at most 3 map-loading runs at a time (memory)
  done
  wait
  OFFS=$(echo $HYP | tr ' ' ',')
  BEST=$(python3 tools/hyp_select.py select $NH $OFFS $PAIRS 2> $W/hyp_select.log)
  OB=$(echo $HYP | cut -d' ' -f$((BEST + 1)))
  echo "heading hypothesis $OB deg selected at frame $NH ($(cat $W/hyp_select.log))"
  $LOC $W/full.csv --vo $W/vo.npz $IMU $COMMON $P --init_offset $OB --debug $W/debug.csv > $W/localize.log 2>&1
  python3 tools/hyp_select.py combine "$OUT" $NH $W/full.csv $OFFS $PAIRS 2>> $W/hyp_select.log
  tail -n 1 $W/hyp_select.log
else
  $LOC "$OUT" --vo "$W/vo.npz" $IMU $COMMON $P --debug "$W/debug.csv" > "$W/localize.log" 2>&1
fi
tail -n 1 "$W/localize.log"
echo "[$(date +%H:%M:%S)] done -> $OUT"
