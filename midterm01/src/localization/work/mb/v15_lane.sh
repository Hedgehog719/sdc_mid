#!/bin/bash
# v15 T2 lane fixes on the paint-voted soft marks (clean map, mk_Spv: cells bright in LiDAR but not paint in the
# aligned camera projections removed), vert calibration as the best set; waits for the v14 run (<= 4 map workers)
cd /root/midterm/src/localization
while pgrep -f "tools/bev_match.py" > /dev/null; do sleep 10; done
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
echo "[$(date +%H:%M:%S)] v15 start"
SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=clean python3 tools/bev_match.py /root/data/Track2 $T2TR work/v15_bm_t2.npz --frames 0:3299:3 --accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --map_marks work/mk_Spv.npz --yaw_fine 0.05 > work/mb/v14/v15.log 2>&1
chown 1000:1000 work/v15_bm_t2.npz work/mb/v14/v15.log
bash work/mb/v14m_fuse.sh work/v15_bm_t2.npz work/ens/v15.csv > /dev/null 2>&1
python3 work/mb/est_score.py T2 work/_v14m_t2.csv work/ens/v15.csv | grep "estimated\|start\|after"
echo "[$(date +%H:%M:%S)] v15 done"
