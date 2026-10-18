#!/usr/bin/env python3
"""Rasterise the hand-drawn ego-vehicle masks (pole_review_server /hood) into PNGs.

Points within `snap` px of an image border are moved onto the border (the mouse cannot reach the very edge),
then each camera's polygons are filled: work/ego_mask_<cam>.png (255 = own vehicle, excluded).
"""
import argparse
import json
import os

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--work', default=os.path.join(os.path.dirname(__file__), '..', 'work'))
ap.add_argument('--snap', type=int, default=30)
a = ap.parse_args()
for c in ['fl', 'f', 'fr']:
    f = os.path.join(a.work, f'ego_mask_{c}.json')
    if not os.path.exists(f):
        continue
    j = json.load(open(f))
    w, h = j['w'], j['h']
    polys = []
    for p in j['polys']:
        q = []
        for x, y in p:
            x = 0 if x < a.snap else (w - 1 if x > w - 1 - a.snap else x)
            y = 0 if y < a.snap else (h - 1 if y > h - 1 - a.snap else y)
            q.append([x, y])
        polys.append(q)
    j['polys_snapped'] = polys
    json.dump(j, open(f, 'w'))
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [np.array(p, np.int32) for p in polys], 255)
    cv2.imwrite(os.path.join(a.work, f'ego_mask_{c}.png'), m)
    print(c, 'masked %.1f%% of the image' % (100 * (m > 0).mean()))
