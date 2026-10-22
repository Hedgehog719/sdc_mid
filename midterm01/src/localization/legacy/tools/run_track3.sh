#!/bin/bash
# Track03 end to end, GNSS-free: images (3 cameras) + raw_ply (frame times) + map.pcd + init_guess.txt [+ IMU in the
# bag]. usage: run_track3.sh <track_dir e.g. /root/data/Track3> <name e.g. T3> [--no_imu]
#  0. prep: map.pcd -> map.npy; IMU from the bag (if any); road segmentation; YOLO vehicle masks
#  1. marking maps from the whole map (no trajectory needed)
#  2. initial pose: init_guess.txt refined by a wide lane match (heading +-25 deg, position +-1 m)
#  3. road-surface visual odometry (speed)
#  4. pass 1: sequential tracker (gyro heading + bias, IMU accel + VO speed, single-frame lane matches + near
#     alignment as measurements)
#  5. pass 2: lane / near fixes around pass 1, RTS fusion with the initial pose only (no GNSS); start segment
#     (path < 15 m) lateral-only; pass 3 = the same around pass 2
#  6. Kaggle csv (idx, x, y, yaw)
set -e
cd /root/midterm/src/localization
T=$1; N=$2; NOIMU=$3
export SDC_ROT_FIX=${SDC_ROT_FIX:-work/cam_fix_v10a.json}
mkdir -p work/t3
echo "[$(date +%H:%M:%S)] prep"
[ -f $T/map.npy ] || python3 tools/map_to_npy.py $T/map.pcd $T/map.npy
if [ ! -f $T/sensors/imu.csv ] && ls $T/*.db3 > /dev/null 2>&1 && [ "$NOIMU" != "--no_imu" ]; then
  python3 tools/extract_bag.py $T $T/sensors || echo "bag extraction failed (no IMU)"
fi
[ -f $T/road/f.npz ] || python3 tools/segment_road.py $T
[ -f $T/masks/f.npz ] || python3 tools/segment_vehicles.py $T
IMUOPT=""; [ -s $T/sensors/imu.csv ] && [ "$NOIMU" != "--no_imu" ] || IMUOPT="--no_imu"
echo "[$(date +%H:%M:%S)] marking maps"
[ -f work/t3/${N}_mk_D.npz ] || python3 tools/marking_map.py $T ALL work/t3/${N}_mk_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > work/t3/${N}_mkD.log 2>&1
[ -f work/t3/${N}_mk_S.npz ] || python3 tools/marking_map.py $T ALL work/t3/${N}_mk_S.npz --mode soft > work/t3/${N}_mkS.log 2>&1
echo "[$(date +%H:%M:%S)] initial pose"
python3 tools/init_search.py $T work/t3/${N}_mk_S.npz work/t3/${N}_init.txt --max_shift 1.0 --frames 0:30:3 | tail -2
echo "[$(date +%H:%M:%S)] visual odometry"
python3 tools/bev_vo.py $T work/t3/${N}_vo.npz $IMUOPT | tail -1
echo "[$(date +%H:%M:%S)] pass 1 (tracker)"
python3 tools/track_seq.py $T work/t3/${N}_mk_D.npz work/t3/${N}_p1.csv --lane_marks work/t3/${N}_mk_S.npz --vo work/t3/${N}_vo.npz --near_too --workers 3 $IMUOPT --init=$(cat work/t3/${N}_init.txt) > work/t3/${N}_p1.log 2>&1
tail -n 1 work/t3/${N}_p1.log
echo "[$(date +%H:%M:%S)] pass 2"
bash work/mb/t3_pass2.sh $T $N
# pass 3: the same fixes recomputed around the pass-2 trajectory (a better prior sharpens the accumulated lane
# response; rehearsal vs the GNSS-aided best: T1 0.250 -> 0.203 m, T2 0.202 -> 0.152 m)
echo "[$(date +%H:%M:%S)] pass 3"
cp work/t3/${N}_p2.csv work/t3/${N}p3_p1.csv
for f in mk_S mk_D; do ln -sf ${N}_$f.npz work/t3/${N}p3_$f.npz; done
bash work/mb/t3_pass2.sh $T ${N}p3 work/t3/${N}p3_p1.csv
cp work/t3/${N}p3_p2.csv work/t3/${N}_result.csv
chown -R 1000:1000 work/t3
echo "[$(date +%H:%M:%S)] done -> work/t3/${N}_result.csv (Kaggle format idx,x,y,yaw)"
