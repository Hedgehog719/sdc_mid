#!/bin/bash
# side-camera re-calibration on the improved poses (near-fixed trajectories) and road-voted marking maps, T1 + T2 jointly
set -e
cd /root/midterm/src/localization
L=work/mb/calib8; mkdir -p $L
cp work/cam_fix_vert.json work/cam_fix_v8.json
for cam in fl fr; do
  SDC_ROT_FIX=work/cam_fix_v8.json python3 tools/calib_img2.py /root/data/Track2 work/_v5n2y_t2.csv work/mk16v_D.npz --cam $cam --out work/cam_fix_v8.json --frames 450:-1:20 --params yaw,pitch,roll --also /root/data/Track1,work/_t1nh.csv,work/mk1_15v_D.npz > $L/$cam.log 2>&1
  tail -6 $L/$cam.log
done
# evaluation on each track separately: before (vert) vs after (v8)
for cam in fl fr; do
  for T in "Track2 work/_v5n2y_t2.csv work/mk16v_D.npz" "Track1 work/_t1nh.csv work/mk1_15v_D.npz"; do set -- $T
    python3 tools/calib_img2.py /root/data/$1 $2 $3 --cam $cam --out /tmp/x.json --frames 300:-1:15 --eval work/cam_fix_vert.json,work/cam_fix_v8.json 2>&1 | grep score | sed "s/^/$1 /"
  done
done
chown 1000:1000 work/cam_fix_v8.json; chown -R 1000:1000 $L
echo calib done
