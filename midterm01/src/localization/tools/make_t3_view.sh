#!/bin/bash
# Track03 rehearsal: a view of a Track01/02 folder with only what Track03 provides (camera images + camera_info,
# raw_ply, map.pcd -> map.npy, init_guess.txt; IMU optional) and NO GNSS. Image-derived caches (road segmentation,
# YOLO masks) are linked since Track03 gets them from its own images.
# usage: make_t3_view.sh /root/data/Track2 /root/data/T3sim2 [--no_imu]
set -e
SRC=$1; DST=$2; NOIMU=$3
S=$(basename $SRC); D=$(basename $DST)
mkdir -p $DST/${D}_images $DST/sensors
for c in f fl fr; do ln -sfn $SRC/${S}_images/${S}_${c}_images $DST/${D}_images/${D}_${c}_images; done
ln -sfn $SRC/${S}_images/camera_info $DST/${D}_images/camera_info
for f in raw_ply map.npy map_intensity.npy init_guess.txt road masks; do ln -sfn $SRC/$f $DST/$f; done
[ "$NOIMU" = "--no_imu" ] || ln -sfn $SRC/sensors/imu.csv $DST/sensors/imu.csv
rm -f $DST/sensors/gnss.csv
ls -la $DST $DST/sensors | head -20
