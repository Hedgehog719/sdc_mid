#!/bin/bash
# T2 start plate shifted forward (horizontal only, full up to s 35 m, linear to 0 at s 60 m) on top of map_clean16,
# + soft marking maps; the shift is then chosen by the image-map match peak of the start fixes (no leaderboard).
set -e
cd /root/midterm/src/localization
T2TR=submissions/day2/track2/yaw_traj_q0.05_bev.csv
for sh in 40 55 70; do
  [ -f /root/data/Track2/map_c16sh$sh.npy ] || python3 tools/splice_level.py /root/data/Track2 $T2TR --tag_in clean16 --tag_out c16sh$sh --max_dz 0 --shift_fwd 0.$sh --hseam 35 --hband 25 | tail -2
  [ -f work/mk16sh${sh}_S.npz ] || SDC_MAP_TAG=c16sh$sh python3 tools/marking_map.py /root/data/Track2 $T2TR work/mk16sh${sh}_S.npz --mode soft | tail -1
done
chown 1000:1000 work/mk16sh*_S.npz
echo plate maps done
