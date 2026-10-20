#!/bin/bash
# Summary of the day-7 candidates: estimate (reference only), differences to the current best, NIS, file checks.
cd /root/midterm/src/localization
echo "== file checks"
python3 - <<'PY'
import glob, numpy as np, pandas as pd
for f in sorted(glob.glob('submissions/day7/track*/*.csv')):
    d = pd.read_csv(f)
    st = np.hypot(np.diff(d.x), np.diff(d.y))
    print(f'{f}: rows {len(d)}, cols {list(d.columns)}, nan {int(d.isna().sum().sum())}, max step {st.max():.3f} m')
PY
echo "== Track2 estimates (reference only)"
python3 work/mb/est_score.py T2 $(ls submissions/day7/track2/*.csv)
echo "== Track1 estimates (reference only)"
python3 work/mb/est_score.py T1 $(ls submissions/day7/track1/*.csv)
echo "== NIS"
grep -h NIS work/mb/algo/fuse_t2.log work/mb/algo/fuse_t2sh.log work/mb/algo/fuse_t1.log 2>/dev/null
echo "== T2 start fixes"
python3 work/mb/start_fix_cmp.py work/c16s_bm_t2.npz work/c16sh40a_bm_t2.npz work/c16sh55a_bm_t2.npz work/c16sh70a_bm_t2.npz 2>/dev/null
