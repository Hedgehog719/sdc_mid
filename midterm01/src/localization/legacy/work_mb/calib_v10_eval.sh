#!/bin/bash
# check of the fr re-calibration: (1) image score on each track, near (1-15 m) and full range (to 30 m), v8 vs v10;
# (2) per-camera alignment with v10: fr should now agree with fl / f (shift ~ 0)
cd /root/midterm/src/localization
V=${1:-a}
for R in "15 1" "30 -8"; do set -- $R
  for T in "Track2 work/_v8y_t2.csv work/mk16v_D.npz" "Track1 work/_v8y_t1.csv work/mk1_15v_D.npz"; do set -- $T $R
    SDC_ROT_FIX=work/cam_fix_v8.json python3 tools/calib_img2.py /root/data/$1 $2 $3 --cam fr --out /tmp/x.json --frames 300:-1:15 --rmax $4 --xmin $5 --min_pts 30 --eval work/cam_fix_v8.json,work/cam_fix_v10$V.json 2>&1 | grep score | sed "s/^/$1 r<$4 /" &
  done
done
wait
SDC_ROT_FIX=work/cam_fix_v10$V.json python3 work/mb/percam_align.py /root/data/Track1 work/mk1_15v_D.npz submissions/day8/track1/t1_pipe_v8y.csv 0:-1:30 2>&1 | tail -12
SDC_ROT_FIX=work/cam_fix_v10$V.json python3 work/mb/percam_align.py /root/data/Track2 work/mk_c16p20_10v_D.npz submissions/day8/track2/t2_pipe_v8y.csv 0:-1:30 2>&1 | tail -13
