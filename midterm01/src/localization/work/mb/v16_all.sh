#!/bin/bash
# v16 T2 end-to-end from the existing fix sets: GNSS-only fusion -> near-fix acceleration correction -> fusion
set -e
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v10t2.json
python3 tools/fuse.py /root/data/Track2 work/_gnssonly_t2.csv --yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv work/_c16_p1.csv --imu_acc --rts_qacc 0.15 > /dev/null 2>&1
python3 tools/near_accel_corr.py work/nf11g_t2.npz work/_gnssonly_t2.csv work/gt/t10_g0.07.csv work/nf11ga_t2.npz
bash work/mb/v16_fuse.sh work/vert_bm_t2m.npz work/_v16r_t2.csv > /dev/null 2>&1
python3 -c "
import pandas as pd, numpy as np
a, b = pd.read_csv('work/_v16_t2.csv'), pd.read_csv('work/_v16r_t2.csv')
print('reproduction max |dx| %.2e |dy| %.2e |dyaw| %.2e' % tuple(np.abs(a[c].values - b[c].values).max() for c in ('x', 'y', 'yaw')))"
chown 1000:1000 work/_v16r_t2.csv work/nf11ga_t2.npz work/_gnssonly_t2.csv
