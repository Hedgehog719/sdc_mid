#!/bin/bash
# fr re-calibration on the near range (1..15 m, as the near fixes use), poses = final v8y, T1 + T2 jointly.
# percam_align showed fr alone wants +0.1..+0.45 m forward / -0.1..-0.2 m lateral while fl / f agree with the pose
# -> fit fr so that it agrees with the pose (= with fl / f). A: angles + height, B: + horizontal mount offset.
set -e
cd /root/midterm/src/localization
L=work/mb/calib10; mkdir -p $L
COMMON="/root/data/Track2 work/_v8y_t2.csv work/mk16v_D.npz --cam fr --frames 450:-1:10 --rmax 15 --xmin 1 --min_pts 30 --also /root/data/Track1,work/_v8y_t1.csv,work/mk1_15v_D.npz"
SDC_ROT_FIX=work/cam_fix_v8.json python3 tools/calib_img2.py $COMMON --params yaw,pitch,roll,tz --out work/cam_fix_v10a.json > $L/a.log 2>&1 &
SDC_ROT_FIX=work/cam_fix_v8.json python3 tools/calib_img2.py $COMMON --params yaw,pitch,roll,tx,ty,tz --out work/cam_fix_v10b.json > $L/b.log 2>&1 &
wait
chown -R 1000:1000 $L work/cam_fix_v10*.json
tail -7 $L/a.log; tail -7 $L/b.log
