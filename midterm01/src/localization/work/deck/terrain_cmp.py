# BEV at the final pose with three projections: flat / terrain profile / terrain + lateral shape; red = map markings
import os, sys
import cv2, numpy as np, pandas as pd
sys.path.insert(0, 'tools')
from bev import ImageBEV, lane_response
from marking_map import MarkingMap
TD = '/root/data/Track3'
tr = pd.read_csv('submissions/final/result_3.csv')
dd = pd.read_csv('work/runs/FINAL_T3/debug.csv')
DY = np.zeros((len(tr), 2))
for j, c in enumerate(('att_p', 'att_r')):
    v = dd[c].values[:len(tr)]; DY[:len(v), j] = np.nan_to_num(v.astype(float))
MK = MarkingMap('work/runs/T3/mk_D.npz')
B = ImageBEV(TD, 1.5, 0, 0, -1.5); B.dyn = DY
rows = []
for k in [int(v) for v in sys.argv[1].split(',')]:
    x, y, yaw = tr.x[k], tr.y[k], tr.yaw[k]
    lanes = MK.bev(np.array([x, y]), yaw)[0] > 0.5
    tiles = []
    for name, ter, lat in (('flat', None, False), ('terrain profile', MK.height, False), ('profile + lateral (terrain_lat)', MK.height, True)):
        B.terrain = ter; B.TERRAIN_LAT = lat; B.pose = (x, y, yaw)
        resp, valid = B.render_resp(k, lane_response)
        g = (90 + 165 * np.clip(resp / 2.0, 0, 1)).astype(np.uint8); g[~valid] = 0
        im = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        im[lanes] = (0.4 * im[lanes] + 0.6 * np.array([40, 40, 230])).astype(np.uint8)
        im = cv2.rotate(cv2.resize(im, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST), cv2.ROTATE_90_CLOCKWISE)
        cv2.rectangle(im, (0, 0), (560, 40), (0, 0, 0), -1)
        cv2.putText(im, f'frame {k}: {name}', (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        tiles.append(im)
    rows.append(np.vstack(tiles))
cv2.imwrite(sys.argv[2], np.hstack([np.pad(r, ((0, 0), (0, 8), (0, 0)), constant_values=255) for r in rows]), [cv2.IMWRITE_JPEG_QUALITY, 88])
