#!/bin/bash
# v14 T2 lane fixes on the vert setup (map clean, marks mk_S): (a) v10t2 calibration, (b) vert calibration with a
# longer accumulation window (40 frames / 20 m: more transverse markings per forward search)
set -e
cd /root/midterm/src/localization
L=work/mb/v14; mkdir -p $L
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
B="--frames 0:3299:3 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --map_marks work/mk_S.npz --yaw_fine 0.05"
echo "[$(date +%H:%M:%S)] a"
SDC_ROT_FIX=work/cam_fix_v10t2.json SDC_MAP_TAG=clean python3 tools/bev_match.py /root/data/Track2 $T2TR work/v14a_bm_t2.npz $B --accum 20 --accum_dist 10 > $L/a.log 2>&1
echo "[$(date +%H:%M:%S)] b"
SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=clean python3 tools/bev_match.py /root/data/Track2 $T2TR work/v14b_bm_t2.npz $B --accum 40 --accum_dist 20 > $L/b.log 2>&1
chown -R 1000:1000 work/v14* $L
echo "[$(date +%H:%M:%S)] done"
