#!/bin/bash
# v12: the start plate shift (0.55 m along the path, s < 20 m, taper 10 m) applied to the whole start area instead of
# the road band only (half_width 14 / road_half 8 left the curb / parked cars / walls unshifted: on v11f the right
# camera wanted -0.55..-0.60 m = the shift, the front camera +0.1). Start fixes redone on it; cameras should agree.
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
L=work/mb/v12; mkdir -p $L
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
echo "[$(date +%H:%M:%S)] maps"
# check: the road-band version rebuilt with the assumed parameters must equal the existing c16p20_10
python3 tools/splice_level.py /root/data/Track2 $T2TR --tag_in clean16 --tag_out c16chk --max_dz 0 --shift_fwd 0.55 --hseam 20 --hband 10 > $L/chk.log
python3 -c "
import numpy as np
a = np.load('/root/data/Track2/map_c16p20_10.npy', mmap_mode='r'); b = np.load('/root/data/Track2/map_c16chk.npy', mmap_mode='r')
print('rebuild check: max |diff| %.4f m' % float(np.abs(a[:, :2] - b[:, :2]).max()))"
rm -f /root/data/Track2/map_c16chk.npy /root/data/Track2/map_intensity_c16chk.npy
python3 tools/splice_level.py /root/data/Track2 $T2TR --tag_in clean16 --tag_out c16p20w --max_dz 0 --shift_fwd 0.55 --hseam 20 --hband 10 --half_width 80 --road_half 79 | tail -1
SDC_MAP_TAG=c16p20w python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk_c16p20w_S.npz --mode soft > /dev/null
SDC_MAP_TAG=c16p20w python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk_c16p20w_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > /dev/null
python3 tools/marking_road_vote.py /root/data/Track2 work/_v10_t2.csv work/mk_c16p20w_D.npz work/mk_c16p20wv_D.npz --thr 0.5 | tail -1
echo "[$(date +%H:%M:%S)] start fixes"
python3 tools/near_fix.py /root/data/Track2 work/mk_c16p20wv_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:450:3 --workers 4 --yaw_search 1.0 --out work/nf12s_t2.npz 2>&1 | tail -1
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
SDC_MAP_TAG=c16p20w python3 tools/bev_match.py /root/data/Track2 $T2TR work/v12s_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_c16p20w_S.npz --max_yaw 6 > $L/s_t2.log 2>&1
python3 - <<'PY'
import numpy as np
for new, old, out in (('work/v12s_bm_t2.npz', 'work/v10_bm_t2m.npz', 'work/v12_bm_t2m.npz'), ('work/nf12s_t2.npz', 'work/nf10_t2.npz', 'work/nf12_t2.npz')):
    a = dict(np.load(new)); b = dict(np.load(old)); keep = b['idx'] >= 450
    o_ = {k: np.concatenate([a[k], b[k][keep]]) for k in b if k in a}
    o = np.argsort(o_['idx']); np.savez(out, **{k: v[o] for k, v in o_.items()})
PY
python3 tools/fix_consistency.py work/v12_bm_t2m.npz work/_c16_p1.csv work/v12g_bm_t2m.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3 --bev work/v12g_bm_t2m.npz --near work/nf12_t2.npz"
python3 tools/fuse.py /root/data/Track2 work/_v12_t2.csv $C2 > /dev/null 2>&1
python3 tools/fuse.py /root/data/Track2 work/_v12f_t2.csv $C2 --bev_fwd_skip 0:330 > /dev/null 2>&1
chown -R 1000:1000 work/mk_c16p20w* work/v12* work/nf12* work/_v12* $L
echo "[$(date +%H:%M:%S)] checks"
python3 - <<'PY'
import numpy as np, pandas as pd
A = pd.read_csv('work/_v10_t2.csv'); yw = A.yaw.values
for f in ('work/_v11g_t2.csv', 'work/_v12_t2.csv', 'work/_v12f_t2.csv'):
    B = pd.read_csv(f); dx, dy = B.x.values - A.x.values, B.y.values - A.y.values
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
    print(f'{f} - v10 fwd/lat: ' + ' | '.join(f'{s0}-{s1}: {fw[s0:s1].mean():+.3f}/{lt[s0:s1].mean():+.3f}' for s0, s1 in ((0, 150), (150, 330), (330, 450), (450, 3300))))
for nm, f in (('near v12', 'work/nf12_t2.npz'), ('lane v12', 'work/v12g_bm_t2m.npz')):
    z = dict(np.load(f)); F = pd.read_csv('work/_v12_t2.csv'); i = z['idx'].astype(int); y_ = F.yaw.values[i]
    dx, dy = z['mx'] - F.x.values[i], z['my'] - F.y.values[i]
    fw, lt = np.cos(y_) * dx + np.sin(y_) * dy, -np.sin(y_) * dx + np.cos(y_) * dy
    print(f'{nm} fix - fused v12: ' + ' | '.join(f'{s0}-{s1}: {np.median(fw[(i>=s0)&(i<s1)]):+.2f}/{np.median(lt[(i>=s0)&(i<s1)]):+.2f} (n{((i>=s0)&(i<s1)).sum()})' for s0, s1 in ((0, 150), (150, 330), (330, 450))))
PY
python3 work/mb/est_score.py T2 work/_v10_t2.csv work/_v12_t2.csv work/_v12f_t2.csv | grep 'estimated\|start'
for f in work/_v12_t2.csv work/_v12f_t2.csv; do for fr in 0:150:3 150:330:3 330:450:3; do python3 work/mb/seg_fwd_curve.py /root/data/Track2 work/mk_c16p20wv_D.npz $f $fr 2>&1 | grep -v Warn | sed 's/  curve.*//' | grep -v "marks  "; done; done
echo "[$(date +%H:%M:%S)] done"
