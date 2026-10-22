#!/bin/bash
# v10: fr re-calibrated on the near range per track (height +9-10 cm on both tracks; pitch / roll differ per track,
# held-out frames prefer each track's own fit): T2 cam_fix_v10t2, T1 cam_fix_v10t1 (f, fl = v8).
# Lane fixes + near fixes redone with it; gating and fusion = the v8y recipes unchanged.
set -e
cd /root/midterm/src/localization
L=work/mb/v10; mkdir -p $L
BMOPT="--accum 20 --accum_step 4 --workers 4 --yaw_mount -1.5 --percam --accum_dist 10 --accum_ds 0.7 --pitch_per_accel -0.8 --dyn_attitude --dyn_roll_only --dyn_sign 1 --ground_mode profile --yaw_fine 0.05"
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
echo "[$(date +%H:%M:%S)] T2 start"
export SDC_ROT_FIX=work/cam_fix_v10t2.json
python3 tools/near_fix.py /root/data/Track2 work/mk16v_D.npz submissions/day7/track2/t2_pipe_v5.csv --frames 0:-1:3 --workers 4 --yaw_search 1.0 --out work/nf10_t2.npz 2>&1 | tail -1
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2TR work/v10s_bm_t2.npz --frames 0:450:3 $BMOPT --map_marks work/mk_c16p20_10_S.npz --max_yaw 6 > $L/s_t2.log 2>&1
SDC_MAP_TAG=c16p20_10 python3 tools/bev_match.py /root/data/Track2 $T2TR work/v10r_bm_t2.npz --frames 450:3299:3 $BMOPT --map_marks work/mk_c16p20_10_S.npz > $L/r_t2.log 2>&1
python3 - <<'PY'
import numpy as np
a, b = dict(np.load('work/v10s_bm_t2.npz')), dict(np.load('work/v10r_bm_t2.npz'))
out = {k: np.concatenate([a[k], b[k]]) for k in a if k in b}
o = np.argsort(out['idx']); np.savez('work/v10_bm_t2m.npz', **{k: v[o] for k, v in out.items()}); print('T2 merged', len(o))
PY
python3 tools/fix_consistency.py work/v10_bm_t2m.npz work/_c16_p1.csv work/v10g_bm_t2m.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
C2="--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 --pole_fwd_sigma 0.4 --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 --yaw_rts_bev 0.7 --gnss_bias_tau 20 --gnss_bias_sigma 0.14 --gnss_white 0.03 --bev_yaw_skip 0:330 --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.15 --near_aniso_k 0.004 --near_yaw_sigma 0.3"
python3 tools/fuse.py /root/data/Track2 work/_v10_t2.csv $C2 --bev work/v10g_bm_t2m.npz --near work/nf10_t2.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
echo "[$(date +%H:%M:%S)] T1"
export SDC_ROT_FIX=work/cam_fix_v10t1.json
python3 tools/near_fix.py /root/data/Track1 work/mk1_15v_D.npz submissions/day7/track1/t1_pipe_v5.csv --frames 0:-1:3 --workers 4 --bg_weight 1.0 --yaw_search 1.0 --out work/nf10_t1.npz 2>&1 | tail -1
SDC_MAP_TAG=clean15 python3 tools/bev_match.py /root/data/Track1 submissions/day7/track1/t1_pipe_v5.csv work/v10_bm_t1.npz --frames 0:3011:3 $BMOPT --map_marks work/mk1_15v_S.npz > $L/t1.log 2>&1
python3 tools/fix_consistency.py work/v10_bm_t1.npz work/_c15_p1.csv work/v10g_bm_t1.npz --win 45 --gate_fwd 0.6 --gate_lat 0.35 | cut -c1-60
B1="--yaw_src imu_cf --yaw_cf_sigma 10 --rts --bev_lat_sigma 0.5 --bev_fwd_sigma 2 --bev_huber 1.5 --bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k 0.01 --bev_aniso_min 0.45 --bev_aniso_lat --yaw_rts traj --yaw_rts_q 0.1 --yaw_rts_bev 1.0 --gnss_bias_tau 5 --gnss_bias_sigma 0.4 --gnss_white 0.02 --map_bias_len 1000 --map_bias_sigma 0.03 --rts_sigma 0.5 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c15_p1.csv --bev_yaw_skip 0:220 --imu_acc --rts_qacc 0.15 --near_fwd_sigma 0.3 --near_lat_sigma 0.10 --near_aniso_k 0.003 --road work/c6_rds_t1.npz --road_pct 50 --road_lat_sigma 0.5 --near_yaw_sigma 0.3"
python3 tools/fuse.py /root/data/Track1 work/_v10_t1.csv $B1 --bev work/v10g_bm_t1.npz --near work/nf10_t1.npz --nis 2>&1 | grep "NIS bev\|NIS near" | cut -c1-80
chown -R 1000:1000 work/v10* work/_v10_t* work/nf10_t* $L
echo "[$(date +%H:%M:%S)] fused"
python3 - <<'PY'
import numpy as np, pandas as pd
for T in ('t2', 't1'):
    for nm in ('bm', 'traj'):
        if nm == 'bm':
            a = dict(np.load(f'work/v8g_bm_{T}{"m" if T == "t2" else ""}.npz')); b = dict(np.load(f'work/v10g_bm_{T}{"m" if T == "t2" else ""}.npz'))
            ia = {int(k): n for n, k in enumerate(a['idx'])}; ib = {int(k): n for n, k in enumerate(b['idx'])}
            com = np.array(sorted(set(ia) & set(ib))); na = np.array([ia[k] for k in com]); nb = np.array([ib[k] for k in com])
            yw = a['myaw'][na]; dx, dy = b['mx'][nb] - a['mx'][na], b['my'][nb] - a['my'][na]; fr = com
        else:
            A, B = pd.read_csv(f'work/_v8y_{T}.csv'), pd.read_csv(f'work/_v10_{T}.csv'); yw = A.yaw.values
            dx, dy = B.x.values - A.x.values, B.y.values - A.y.values; fr = np.arange(len(A))
        fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
        print(f'{T} {nm:4s} v10 - v8y: fwd mean {fw.mean():+.3f} rms {np.sqrt(np.mean(fw**2)):.3f} | lat mean {lt.mean():+.3f} rms {np.sqrt(np.mean(lt**2)):.3f}')
        print('   per 300 fwd/lat: ' + ' | '.join(f'{s0}: {fw[(fr>=s0)&(fr<s0+300)].mean():+.2f}/{lt[(fr>=s0)&(fr<s0+300)].mean():+.2f}' for s0 in range(0, 3300, 300) if ((fr>=s0)&(fr<s0+300)).any()))
PY
python3 work/mb/est_score.py T2 work/_v8y_t2.csv work/_v10_t2.csv | grep estimated
python3 work/mb/est_score.py T1 work/_v8y_t1.csv work/_v10_t1.csv | grep estimated
SDC_ROT_FIX=work/cam_fix_v10t2.json python3 tools/near_align.py /root/data/Track2 work/mk16v_D.npz work/_v8y_t2.csv work/_v10_t2.csv --frames 0:-1:9 --workers 4 2>&1 | tail -2
SDC_ROT_FIX=work/cam_fix_v10t1.json python3 tools/near_align.py /root/data/Track1 work/mk1_15v_D.npz work/_v8y_t1.csv work/_v10_t1.csv --frames 0:-1:9 --workers 4 2>&1 | tail -2
echo "[$(date +%H:%M:%S)] done"
