#!/bin/bash
cd /root/midterm/src/localization
export SDC_ROT_FIX=work/cam_fix_v8.json
python3 tools/render_paint.py /root/data/Track2 submissions/day8/track2/t2_pipe_v8y.csv work/mk16v_D.npz --frames 0:-1:3 --name paint_v8y > work/mb/rf_p2.log 2>&1
python3 tools/render_paint.py /root/data/Track1 submissions/day8/track1/t1_pipe_v8y.csv work/mk1_15v_D.npz --frames 0:-1:3 --name paint_v8y > work/mb/rf_p1.log 2>&1
python3 tools/render_topdown.py /root/data/Track2 submissions/day8/track2/t2_pipe_v8y.csv --b submissions/track2/0.17429__t2_opt10.csv --label_a 'pipeline v8y' --label_b '0.17' --layout dual --size 700 --name dual_v8y --frames 0:-1:3 --tag clean16 > work/mb/rf_d2.log 2>&1
python3 tools/render_topdown.py /root/data/Track1 submissions/day8/track1/t1_pipe_v8y.csv --b submissions/track1/0.41123__t1_opt11.csv --label_a 'pipeline v8y' --label_b '0.41' --layout dual --size 700 --name dual_v8y --frames 0:-1:3 --tag clean15 > work/mb/rf_d1.log 2>&1
chown -R 1000:1000 work/viewer work/video
echo render final done > work/mb/render_final.done
