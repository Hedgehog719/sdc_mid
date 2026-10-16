#!/usr/bin/env python3
"""YOLO instance segmentation of road users (cars, trucks, buses, bikes, people) on the 3 camera images.

Masks are stored at half resolution, bit-packed, one array per camera: masks/<cam>.npz
(keys: names, masks[N, H/2, W/2 packed along W]). Points projecting into a mask are removed before
registration, which handles vehicles stopped at traffic lights that temporal filtering cannot detect.
"""
import argparse
import os

import cv2
import numpy as np
from ultralytics import YOLO

CLASSES = [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('track_dir')
    parser.add_argument('--model', default='yolo11x-seg.pt')
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--dilate', type=int, default=7, help='mask dilation in full-res pixels')
    parser.add_argument('--batch', type=int, default=16)
    args = parser.parse_args()
    track = os.path.basename(os.path.normpath(args.track_dir))
    out_dir = os.path.join(args.track_dir, 'masks')
    os.makedirs(out_dir, exist_ok=True)
    model = YOLO(args.model)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * args.dilate + 1,) * 2)
    for cam in ['f', 'fl', 'fr']:
        img_dir = os.path.join(args.track_dir, f'{track}_images', f'{track}_{cam}_images')
        names = sorted(os.listdir(img_dir))
        packed = []
        for b in range(0, len(names), args.batch):
            batch = [os.path.join(img_dir, n) for n in names[b:b + args.batch]]
            results = model.predict(batch, classes=CLASSES, conf=args.conf, retina_masks=True, verbose=False, half=True)
            for r in results:
                h, w = r.orig_shape
                m = np.zeros((h, w), np.uint8)
                if r.masks is not None:
                    m = (r.masks.data.cpu().numpy().max(0) > 0.5).astype(np.uint8)
                m = cv2.dilate(m, kernel)
                m = cv2.resize(m, (w // 2, h // 2), interpolation=cv2.INTER_NEAREST)
                packed.append(np.packbits(m.astype(bool), axis=1))
            print(cam, b + len(batch), '/', len(names), flush=True)
        np.savez_compressed(os.path.join(out_dir, f'{cam}.npz'), names=np.array(names), masks=np.stack(packed))


if __name__ == '__main__':
    main()
