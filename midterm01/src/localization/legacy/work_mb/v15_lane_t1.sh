#!/bin/bash
# v15 T1 lane fixes on the paint-voted marks (mk1_15vpv_S), v10 recipe otherwise; queued after the T2 runs
cd /root/midterm/src/localization
sleep 30
while pgrep -f "tools/bev_match.py|v15_lane.sh" > /dev/null; do sleep 10; done
echo "[$(date +%H:%M:%S)] v15 T1 start"
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
SDC_ROT_FIX=work/cam_fix_v10t1.json SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv work/v15_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15vpv_S.npz > work/mb/v14/v15_t1.log 2>&1
python3 tools/fix_consistency.py work/v15_bm_t1.npz work/_c15_p1.csv work/v15g_bm_t1.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
sed 's#--bev work/v10g_bm_t1.npz#--bev work/v15g_bm_t1.npz#' work/mb/v14m_fuse_t1.sh > /tmp/v15_t1.sh
bash /tmp/v15_t1.sh work/t1s/v15.csv > /dev/null 2>&1
sed 's#--bev work/v10g_bm_t1.npz#--bev work/v15g_bm_t1.npz#; s#--gnss_bias_sigma 0.15#--gnss_bias_sigma 0.4#' work/mb/v14m_fuse_t1.sh > /tmp/v15_t1_g04.sh
bash /tmp/v15_t1_g04.sh work/t1s/v15_g0.4.csv > /dev/null 2>&1
chown 1000:1000 work/v15*_bm_t1.npz work/t1s/v15*.csv work/mb/v14/v15_t1.log
python3 work/mb/est_score.py T1 work/_v14m_t1.csv work/t1s/v15.csv work/_v13_t1.csv work/t1s/v15_g0.4.csv | grep "estimated\|after"
echo "[$(date +%H:%M:%S)] v15 T1 done"
