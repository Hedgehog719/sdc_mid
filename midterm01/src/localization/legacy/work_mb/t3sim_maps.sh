#!/bin/bash
# marking maps for the GNSS-free rehearsal views, built without any trajectory (every occupied map tile)
cd /root/midterm/src/localization
for T in T3sim2 T3sim1; do
  echo "[$(date +%H:%M:%S)] $T"
  python3 tools/marking_map.py /root/data/$T ALL work/t3/${T}_mk_D.npz --hi 2.5 --lo 1.2 --cell 0.7 > work/t3/${T}_mkD.log 2>&1
  python3 tools/marking_map.py /root/data/$T ALL work/t3/${T}_mk_S.npz --mode soft > work/t3/${T}_mkS.log 2>&1
  tail -1 work/t3/${T}_mkD.log; tail -1 work/t3/${T}_mkS.log
done
chown -R 1000:1000 work/t3
echo "[$(date +%H:%M:%S)] maps done"
