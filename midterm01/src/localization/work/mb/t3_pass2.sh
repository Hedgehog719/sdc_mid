#!/bin/bash
# GNSS-free pass 2 on a Track03(-like) folder: lane / near fixes around the pass-1 trajectory, then the RTS fusion
# with the initial pose only (no GNSS) + IMU. usage: t3_pass2.sh <track_dir> <name> [<pass1.csv>]
set -e
cd /root/midterm/src/localization
T=$1; N=$2; P1=${3:-work/t3/${N}_p1.csv}
export SDC_ROT_FIX=${SDC_ROT_FIX:-work/cam_fix_v10a.json}
L=work/t3/log_$N; mkdir -p $L
python3 tools/t3_fake_gnss.py $T $P1
echo "[$(date +%H:%M:%S)] lane fixes"
python3 tools/bev_match.py $T $P1 work/t3/${N}_bm.npz --frames 0:-1:3 --accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05 --map_marks work/t3/${N}_mk_S.npz --max_shift 3.0 > $L/bm.log 2>&1
python3 tools/fix_consistency.py work/t3/${N}_bm.npz $P1 work/t3/${N}_bmg.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-70
echo "[$(date +%H:%M:%S)] near fixes"
python3 tools/near_fix.py $T work/t3/${N}_mk_D.npz $P1 --frames 0:-1:3 --workers 4 --yaw_search 1.0 --out work/t3/${N}_nf.npz 2>&1 | tail -1
echo "[$(date +%H:%M:%S)] fusion"
python3 tools/fuse.py $T work/t3/${N}_p2.csv --gnss_keep_s 1 --yaw_src imu --rts --bev work/t3/${N}_bmg.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --near work/t3/${N}_nf.npz --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 2.0 --yaw_rts_base 2 --yaw_selfcal --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv $P1 --imu_acc --rts_qacc 0.15 --nis > $L/fuse.log 2>&1
grep "NIS\|selfcal\|GNSS-free" $L/fuse.log | cut -c1-90
chown -R 1000:1000 work/t3
echo "[$(date +%H:%M:%S)] pass 2 done -> work/t3/${N}_p2.csv"
