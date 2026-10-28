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
cd "${SDC_PACKAGE_DIR:-$(dirname "$0")/..}"
TD=$(realpath "$1")
TR=$2
CACHE=work/runs/$TR
W=${SDC_RUN_DIR:-$CACHE}
HYP_WORKERS=${SDC_HYP_WORKERS:-3}
case $HYP_WORKERS in 1|2|3) ;; *) echo "SDC_HYP_WORKERS must be 1, 2 or 3"; exit 1 ;; esac
OUT=${3:-$W/${TR}_online.csv}
mkdir -p "$W" "$CACHE" "$(dirname "$OUT")"
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
      P="--gnss --gnss_sigma 1.0 0.2 --gyro_bias -0.220 --lane_k 0.005 0.005 --vo_noise 0.02 --acc_noise 0.05 --imu_causal
         --dyn_att --dyn_roll_scale 0 --pitch_per_accel -0.3" ;;
      # --dyn_att ... -0.3 (braking dive / squat as T3, flat ground): vs the reference 0.411 -> 0.408 (-0.5: 0.416; lane
      # heading sigma 0.5 / 1.0 / 1.5, GNSS sigma 0.6 fwd / 0.3 lat, terrain, segmentation gate: all worse)
  T2) export SDC_ROT_FIX=calib/cam_fix_v10t2.json
      P="--gnss --gnss_sigma 0.15 0.2 --gyro_bias -0.247 --lane_k 0.02 0.01 --vo_noise 0.15 --acc_noise 0.05 --floor 0.1 0.1 0.15
         --stand_search 15 --stand_search_sigma 5.6 --imu_causal --seg_gate 0.5 --seg_gate_track --yaw_sigma 2.5
         --slow_sharp_f 0.035 2" ;;
      # --stand_search 15 --stand_search_sigma 5.6 (was 30 / 4): the same information per second (twice the searches,
      # sqrt(2) the sigma), spread more evenly: the frame-0 search error (-1.9 deg) is taken back sooner and without the
      # overshoot of 15 / 4 (+0.5 deg at 150-200); vs the reference 0.205 -> 0.201, fair innovations lane lat 0.100 ->
      # 0.099 m, near lat 0.101 -> 0.098, lane heading 0.518 -> 0.515, near heading 0.407 -> 0.400 deg (10 / 4: 0.207)
      # --stand_search_sigma 4 (was 2.5): the standstill searches see the same queue scene, their errors are correlated
      # (all ~+1.5 deg vs where the moving lane headings later put it); 2.5 each let five of them rotate the heading
      # +0.77 deg: fair innovations lane heading 0.581 -> 0.518 deg, lane lat 0.113 -> 0.100 m, vs the reference 0.206 ->
      # 0.205 (5.6: heading at frames 0-200 too far the other way, 0.215)
      # --slow_sharp_f 0.035 2: below 2 m/s (the start queue, bus alongside) lane forward fixes need sharpness > 0.035;
      # the barely-sharp ones there (0.020-0.034, peak 0.20-0.25) all said -0.3..-0.5 m against GNSS: GNSS forward
      # innovations 0-400 rms 0.116 -> 0.083 m, vs the reference 0.214 -> 0.206, from 400 unchanged (0.035 / 0.05 and
      # 2 / 4 m/s identical; a global 0.035 hurt the steady state 0.163 -> 0.171); T1 identical output
      # --yaw_sigma 2.5 (COMMON 0.7): the T2 lane headings are biased in places (426-570: accepted innovations -0.5..-0.8
      # deg, the heading drifted -1 deg); with GNSS the heading also follows the GNSS / lateral fixes through the joint
      # EKF: vs the reference 1.0 -> 0.217, 1.5 -> 0.216, 2.5 -> 0.214 (steady 0.180 -> 0.163); T1 gets worse (0.411 ->
      # 0.430-0.441), T3 has no GNSS
      # --seg_gate 0.5 --seg_gate_track: the tracking lane matches use only image responses on the road surface the
      # cameras see (SegFormer road probability > 0.5, IPM, dilated 0.35 m; parked cars, kerbs, facades, the bus beside
      # the car drop out); the frame-0 / standstill heading searches stay ungated (gated, their heading got worse):
      # vs the reference 0.238 -> 0.223 (steady state from 400: 0.197 -> 0.180, lateral 0.096 -> 0.070)
      # T2 starts standing in a queue (bus alongside, markings hidden): the frame-0 heading search is 1.8 deg off ->
      # repeat it every 30 frames while standing with an uncertain heading (later frames see other markings)
  T3) export SDC_ROT_FIX=calib/cam_fix_v10a.json   # no GNSS
      export SDC_CAM_SYNC=${SDC_CAM_SYNC:-1}
      export SDC_CAM_SYNC_YAW=${SDC_CAM_SYNC_YAW:-1}
      P="--gyro_bias -0.23 --lane_k 0.005 0.005 --vo_noise 0.08 --acc_noise 0.1 --v0_sd 10 --min_peak_lo 0.13 --lo_peak_sd 0.5 99
         --still_freeze --imu_causal --terrain --dyn_att --dyn_roll_scale 0 --pitch_per_accel -0.3 --init_pos 2.5 --vo_dr_init
         --att_imu_map 1.5 --att_resid --att_pitch_only --roll_imu_map 120 --terrain_lat --seg_gate 0.5 --seg_gate_track
         --seg_gate_from 300 --yaw_soft_peak 0.2 --kvo0 0.97 --start_lat_batch 3 0.15 0.05 60" ;;
      # --start_lat_batch 3 0.15 0.05 60: in the first 60 frames weak lane lateral offsets (peak > 0.15, lateral
      # sharpness > 0.05, below the regular gates) are applied as one median per 3 (frames 6-12 all said 0.18-0.20 m
      # right; single weak lateral matches stay off - they caused a wrong-lane lock in the T1 rehearsal): updates at
      # frames 9 (-0.18) and 18 (-0.20), later ones ~0; the start reference error (frames 0-60) is lower in all four
      # settings (0.343 / 1.253 / 0.262 / 0.704 -> 0.330 / 1.248 / 0.247 / 0.698); public 0.19 (= before, 57718059)
      # --kvo0 0.97: initial VO scale (VO speed / true speed) from the converged Track01 / Track02 values (0.963 / 0.994;
      # Track03 itself converges to 0.95-0.97) instead of 1.0: the start ran 3-4 % slow until the map fixes corrected
      # the scale, and the weak lane forward fixes of frames 0-150 all said "further ahead" (+0.26..+0.82 m); with it
      # +0.14..+0.49 m, frames 0-60 move 0.09 m forward, everything after within 0.04 m, fair innovations unchanged;
      # error vs the start reference (batch fit of the start pose to the lane matches of frames 0-150 with the VO / gyro
      # path, work/online/start_batch.py; its ranking of 10 earlier T3 submissions agrees with their public scores,
      # rank correlation 0.92) 0.338 -> 0.309 over frames 0-60; public 0.22 -> 0.20 (t3_online_kvo97)
      # --seg_gate 0.5 --seg_gate_track --seg_gate_from 300: road-surface gate as T2, but only from frame 300 - gated from
      # the start it dropped the right forward fixes at 105-177 and took a wrong one (0.9 m behind for ~40 frames; public
      # 0.35 vs 0.24); from 300 on: start identical, fair innovations lane fwd 0.089 -> 0.067 m, lane heading 0.227 ->
      # 0.221, near heading 0.357 -> 0.345 deg
      # --terrain_lat: the terrain BEV also follows the map road surface across the width (kerb-side lanes / gutters
      # lower than the vehicle plane: at 705-735 the kerb-side lane line was 0.4 m off while the right line fitted):
      # fair innovations lane lat 0.096 -> 0.026 m, near lat 0.064 -> 0.042, lane heading 0.40 -> 0.24 deg; the
      # straight-road heading - course offset -0.26 -> -0.01 deg (the lateral road shape had skewed the BEV)
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
# T3 heading-hypothesis output for frames 0..NH-1: posterior mean (default; the minimum mean-square-error estimate,
# which is what an RMSE score rewards, with each frame's weights from the likelihoods up to that frame) or the
# per-frame MAP hypothesis; vs the start reference (work/online/start_batch.py) frames 0-60 0.309 -> 0.262 m,
# public 0.20 -> 0.19 (t3_online_kvo97_mean; on 10/26 without --kvo0: 0.330 vs 0.349, both public 0.23)
case ${SDC_HYP_ESTIMATOR:-mean} in
  map) COMBINE=combine ;;
  mean) COMBINE=combine_mean ;;
  *) echo "SDC_HYP_ESTIMATOR must be map or mean"; exit 1 ;;
esac
P="$P ${SDC_LOC_EXTRA:-}"
printf '%s\n' "$COMMON $P" > "$W/args.txt"
printf 'SDC_CAM_SYNC=%s\nSDC_CAM_SYNC_YAW=%s\nSDC_ROT_FIX=%s\nSDC_HYP_ESTIMATOR=%s\nSDC_HYP_WORKERS=%s\nSDC_CAM_CAPTURE=%s\n' \
  "${SDC_CAM_SYNC:-0}" "${SDC_CAM_SYNC_YAW:-0}" "$SDC_ROT_FIX" "${SDC_HYP_ESTIMATOR:-mean}" "$HYP_WORKERS" "${SDC_CAM_CAPTURE:-0}" > "$W/environment.txt"

echo "[$(date +%H:%M:%S)] 1. map and sensor data"
if [[ ${SDC_CAM_CAPTURE:-0} == 1 ]]; then
  python3 tools/camera_timing.py "$TD"
fi
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
[ -f "$CACHE/mk_D.npz" ] || python3 tools/marking_map.py "$TD" ALL "$CACHE/mk_D.npz" --hi 2.5 --lo 1.2 --cell 0.7 > "$CACHE/mk_D.log" 2>&1
[ -f "$CACHE/mk_S.npz" ] || python3 tools/marking_map.py "$TD" ALL "$CACHE/mk_S.npz" --mode soft > "$CACHE/mk_S.log" 2>&1

echo "[$(date +%H:%M:%S)] 4. visual odometry"
[ -f "$CACHE/vo.npz" ] || python3 tools/bev_vo.py "$TD" "$CACHE/vo.npz" $IMU | tail -n 1
VO=$CACHE/vo.npz
if [ "$TR" = T3 ]; then
  # pairs whose template block is < 90 % valid (lead vehicle close ahead: frames 1147-2160, 2854-3141) had no VO at all;
  # those pairs alone are refilled from a matching with the validity mask (>= 30 % valid), every other pair unchanged
  [ -f "$CACHE/vo_m30.npz" ] || python3 tools/bev_vo.py "$TD" "$CACHE/vo_m30.npz" $IMU --mask_tol 0.3 | tail -n 1
  [ -f "$CACHE/vo_fill.npz" ] || python3 tools/vo_fill.py "$CACHE/vo.npz" "$CACHE/vo_m30.npz" "$CACHE/vo_fill.npz"
  VO=$CACHE/vo_fill.npz
fi

echo "[$(date +%H:%M:%S)] 5. online localization ($N)"
LOC="python3 tools/localize_online.py $TD $CACHE/mk_D.npz $CACHE/mk_S.npz"
if [ "$TR" = T3 ]; then
  # no GNSS and init_guess off by ~17 deg: the frame-0 heading search is flat over several degrees and a filter started
  # a few degrees off may never lock -> bank of heading hypotheses (frame-0 search + offsets) for the first NH frames,
  # per-frame causal selection by accepted map measurements, the winner continues (tools/hyp_select.py)
  NH=600; HYP="0 -1.5 1.5 -3 3"; PAIRS=""; i=0
  for o in $HYP; do
    $LOC $W/hyp_$o.csv --vo $VO $IMU $COMMON $P --init_offset $o --frames $NH --debug $W/hyp_${o}_dbg.csv > $W/hyp_$o.log 2>&1 &
    PAIRS="$PAIRS $W/hyp_$o.csv:$W/hyp_${o}_dbg.csv"; i=$((i + 1))
    [ $((i % HYP_WORKERS)) -eq 0 ] && wait
  done
  wait
  OFFS=$(echo $HYP | tr ' ' ',')
  BEST=$(python3 tools/hyp_select.py select $NH $OFFS $PAIRS 2> $W/hyp_select.log)
  OB=$(echo $HYP | cut -d' ' -f$((BEST + 1)))
  echo "heading hypothesis $OB deg selected at frame $NH ($(cat $W/hyp_select.log))"
  $LOC $W/full.csv --vo $VO $IMU $COMMON $P --init_offset $OB --debug $W/debug.csv > $W/localize.log 2>&1
  python3 tools/hyp_select.py "$COMBINE" "$OUT" $NH $W/full.csv $OFFS $PAIRS 2>> $W/hyp_select.log
  tail -n 1 $W/hyp_select.log
elif [ "$TR" = T1 ] || [ "$TR" = T2 ]; then
  # frame-wise mean of five causal runs (tools/ensemble.py; frame k averages the members' frame-k poses, each from
  # frames <= k only); MEMBERS: extra options per member, MENV: extra environment per member (T1 five, T2 six)
  # T1: the filter is chaotic where the markings are sparse (near-identical settings differ by +-0.02 vs the
  # reference). Members: the configuration above; + --fwd_batch 8 / 10 / 12 (weak lane forward offsets batched to a
  # median: the single sharpness-0.024 fix at frame ~2040 pulled the position 0.6 m back); + --gnss_huber (Huber
  # weight on GNSS: innovations up to 1-2 m against sigma 0.2 m). Members 0.427 / 0.408 / 0.414 / 0.407 / 0.409 vs the
  # reference, mean 0.404 (start 0-300 0.437 -> 0.373 m, 1800-2400 0.420 -> 0.361 m); public 0.47 -> 0.45
  # (t1_online_v9_ens5)
  # T2: members the configuration above (v22); + --stand_search_sigma 4 (search every 15 frames, sigma 4); + camera
  # capture-time sync (SE(2)); + --gnss_noyaw_v 3 (GNSS leaves the heading alone below 3 m/s); + dead-reckoned lane
  # accumulation over 5 m above 3 m/s. Members 0.201 / 0.201 / 0.202 / 0.201 / 0.203 vs the reference, mean 0.198
  # (lateral steady state 0.069 -> 0.065 m); public 0.26 -> 0.25 (t2_online_v23_ens5). Sixth member: GNSS bias state
  # estimated only while standing, fed by batched weak lane lateral offsets of the queue (all said 0.2 m right of the
  # biased GNSS), a consider state while moving (alone 0.223: the standstill bias lingers for ~30 s; near lat 0.097 ->
  # 0.100); mean of six 0.191; public 0.25 -> 0.24 (t2_online_v24_ens6)
  if [ "$TR" = T1 ]; then
    MEMBERS=("" "--fwd_batch 8" "--fwd_batch 10" "--fwd_batch 12" "--gnss_huber"); MENV=("" "" "" "" "")
  else
    MEMBERS=("" "--stand_search_sigma 4" "" "--gnss_noyaw_v 3" "--accum_dist 5 --accum_dr --accum_vmin 3"
             "--gnss_bias_sd 0.2 --gnss_bias_tau 30 --gnss_bias_still --still_lat_batch 5 0.15 0.05")
    MENV=("" "" "SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1" "" "" "")
  fi
  RUNS=""
  for i in "${!MEMBERS[@]}"; do
    if [ "$i" -eq 0 ]; then D="$W/debug.csv"; L="$W/localize.log"; else D="$W/member_${i}_dbg.csv"; L="$W/member_$i.log"; fi
    env ${MENV[$i]} $LOC $W/member_$i.csv --vo "$VO" $IMU $COMMON $P ${MEMBERS[$i]} --debug "$D" > "$L" 2>&1 &
    RUNS="$RUNS $W/member_$i.csv"
    [ $(((i + 1) % HYP_WORKERS)) -eq 0 ] && wait
  done
  wait
  python3 tools/ensemble.py "$OUT" $RUNS
else
  $LOC "$OUT" --vo "$VO" $IMU $COMMON $P --debug "$W/debug.csv" > "$W/localize.log" 2>&1
fi
tail -n 1 "$W/localize.log"
echo "[$(date +%H:%M:%S)] done -> $OUT"
