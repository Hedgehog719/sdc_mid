#!/bin/bash
# T3 full run (hypothesis 0, as the final) with extra options; frames 0-599 come from the final bank output
cd /root/midterm/src/localization
TAG=$1; shift; X="$@"
W=work/runs/$TAG; mkdir -p $W
export SDC_ROT_FIX=calib/cam_fix_v10a.json SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1
A=$(cat work/runs/FINAL_T3/args.txt)
python3 tools/localize_online.py /root/data/Track3 work/runs/T3/mk_D.npz work/runs/T3/mk_S.npz $W/full.csv --vo work/runs/T3/vo_fill.npz $A $X --init_offset=0 --debug $W/debug.csv > $W/localize.log 2>&1
python3 - $W <<'PY'
import sys, pandas as pd
W = sys.argv[1]
f = pd.read_csv('work/runs/FINAL_T3/T3_online.csv'); n = pd.read_csv(W + '/full.csv')
f.iloc[600:] = n.iloc[600:].values
f.to_csv(W + '/T3_mean.csv', index=False)
PY
chown -R 1000:1000 $W; echo done > $W/DONE
