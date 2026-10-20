#!/bin/bash
# round 2: front camera pitch / roll / height, then side cameras pitch / roll / height, T1 + T2 jointly, poses = v8 / nohold
set -e
cd /root/midterm/src/localization
L=work/mb/calib9; mkdir -p $L
cp work/cam_fix_v8.json work/cam_fix_v9.json
for cp in "f pitch,roll,tz" "fl pitch,roll,tz" "fr pitch,roll,tz"; do set -- $cp
  SDC_ROT_FIX=work/cam_fix_v9.json python3 tools/calib_img2.py /root/data/Track2 work/_v8_t2.csv work/mk16v_D.npz --cam $1 --out work/cam_fix_v9.json --frames 450:-1:20 --params $2 --also /root/data/Track1,work/_t1nh.csv,work/mk1_15v_D.npz > $L/$1.log 2>&1
  tail -3 $L/$1.log
done
for cam in f fl fr; do
  for T in "Track2 work/_v8_t2.csv work/mk16v_D.npz" "Track1 work/_t1nh.csv work/mk1_15v_D.npz"; do set -- $T
    python3 tools/calib_img2.py /root/data/$1 $2 $3 --cam $cam --out /tmp/x.json --frames 300:-1:15 --eval work/cam_fix_v8.json,work/cam_fix_v9.json 2>&1 | grep score | sed "s/^/$1 /"
  done
done
chown 1000:1000 work/cam_fix_v9.json; chown -R 1000:1000 $L
echo calib9 done
