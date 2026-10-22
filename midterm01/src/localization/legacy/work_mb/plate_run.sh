#!/bin/bash
# After algo_run.sh: start fixes on the forward-shifted start plate (+0.40 / +0.55 / +0.70 m), shift chosen by the
# median match peak (image-map agreement, no leaderboard), fused like t2_algo_c16. Then comparison-only variants with
# the old probe-derived constants (yaw_bias 0.22, xy_bias 0.041 0.038, time_shift -0.004).
set -e
cd /root/midterm/src/localization
until grep -q done work/mb/algo_run.log && [ -f work/mk16sh70_S.npz ]; do sleep 30; done
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
L=work/mb/algo
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
for sh in 40 55 70; do
  echo "[$(date +%H:%M:%S)] plate +0.$sh m start matching"
  [ -f work/c16sh${sh}a_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=c16sh$sh python3 tools/bev_match.py /root/data/Track2 $T2TR work/c16sh${sh}a_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk16sh${sh}_S.npz --max_yaw 6 > $L/c16sh${sh}a.log 2>&1
  [ -f work/c16sh${sh}b_bm_t2.npz ] || SDC_ROT_FIX=work/cam_fix_vert.json SDC_MAP_TAG=c16sh$sh python3 tools/bev_match.py /root/data/Track2 $T2TR work/c16sh${sh}b_bm_t2.npz --frames 450:700:3 $BMOPT --map_marks work/mk16sh${sh}_S.npz > $L/c16sh${sh}b.log 2>&1
done
SH=$(python3 - <<'PY'
import numpy as np
best = None
for sh in (40, 55, 70):
    z = np.load(f'work/c16sh{sh}a_bm_t2.npz'); p = np.median(z['peak'])
    zb = np.load(f'work/c16sh{sh}b_bm_t2.npz')
    print(f'plate +0.{sh}: start median peak {p:.4f} (450-700: {np.median(zb["peak"]):.4f})', file=__import__('sys').stderr)
    if best is None or p > best[0]: best = (p, sh)
z0 = np.load('work/c16s_bm_t2.npz'); print(f'plate +0.00: start median peak {np.median(z0["peak"]):.4f}', file=__import__('sys').stderr)
print(best[1] if best[0] > np.median(z0['peak']) else 0)
PY
)
echo "chosen plate shift: 0.$SH m (max start match peak)"
if [ "$SH" != "0" ]; then
python3 - $SH <<'PY'
import sys, numpy as np
sh = sys.argv[1]
a = dict(np.load(f'work/c16sh{sh}a_bm_t2.npz')); b = dict(np.load(f'work/c16sh{sh}b_bm_t2.npz')); c = dict(np.load('work/c16_bm_t2.npz'))
keep = c['idx'] >= 700
out = {k: np.concatenate([a[k], b[k], c[k][keep]]) for k in c if k in a and k in b}
o = np.argsort(out['idx']); out = {k: v[o] for k, v in out.items()}
np.savez(f'work/c16sh{sh}_bm_t2m.npz', **out); print('merged', len(out['idx']))
PY
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev work/c16sh${SH}_bm_t2m.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 10 --gnss_bias_sigma 0.2 --gnss_white 0.03"
python3 tools/fuse.py /root/data/Track2 work/_c16sh${SH}_p1.csv $C2 --rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1 > $L/fuse_t2sh_p1.log 2>&1
python3 tools/fuse.py /root/data/Track2 work/_c16sh${SH}_gm.csv $C2 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16sh${SH}_p1.csv --nis > $L/fuse_t2sh.log 2>&1
cp work/_c16sh${SH}_gm.csv submissions/day7/track2/t2_algo_c16sh${SH}.csv
python3 tools/fuse.py /root/data/Track2 work/_c16sh${SH}_gmK.csv $C2 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16sh${SH}_p1.csv --yaw_bias 0.22 --xy_bias 0.041 0.038 --time_shift -0.004 > /dev/null 2>&1
cp work/_c16sh${SH}_gmK.csv submissions/day7/track2/t2_algo_c16sh${SH}_boardconst.csv
fi
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev work/c16_bm_t2m.npz --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 10 --gnss_bias_sigma 0.2 --gnss_white 0.03"
python3 tools/fuse.py /root/data/Track2 work/_c16_gmK.csv $C2 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --yaw_bias 0.22 --xy_bias 0.041 0.038 --time_shift -0.004 > /dev/null 2>&1
cp work/_c16_gmK.csv submissions/day7/track2/t2_algo_c16_boardconst.csv
chown -R 1000:1000 submissions/day7 work/_c16* work/c16* $L
echo "[$(date +%H:%M:%S)] plate run done"
