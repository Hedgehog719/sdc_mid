#!/bin/bash
# one T2 ensemble member outside the runner (validation): runner T2 arguments (v22 configuration) + extra options
# usage: t2_member.sh <tag> [extra localize_online options]
cd /root/midterm/src/localization
TAG=$1; shift; X="$@"
O=work/runs/T2m; mkdir -p $O
export SDC_ROT_FIX=calib/cam_fix_v10t2.json
A=$(cat work/runs/T2_v23/args.txt)
python3 tools/localize_online.py /root/data/Track2 work/runs/T2/mk_D.npz work/runs/T2/mk_S.npz $O/$TAG.csv --vo work/runs/T2/vo.npz $A \
  $X --debug $O/${TAG}_dbg.csv > $O/$TAG.log 2>&1
chown -R 1000:1000 $O
