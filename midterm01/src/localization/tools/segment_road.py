#!/usr/bin/env python3
"""Drivable-road probability for every camera image with SegFormer-B2 (Cityscapes: road / sidewalk / ...).

The road/sidewalk boundary is where kerbs and median islands are; matching the image road region with the map's
road surface (bounded by kerbs) constrains lateral position, and island ends / corners at intersections constrain
the along-track position. Saved at quarter resolution, uint8 (0..255), one npz per camera: road/<cam>.npz with
  road  = P(road) + P(vehicles: car, truck, bus, train, motorcycle, bicycle)  (vehicles stand on the road, so they
          must not punch holes into the drivable area; YOLO masks are therefore not applied to this feature)
  side  = P(sidewalk)
"""
import argparse
import os

import cv2
import numpy as np
import torch
from transformers import (AutoImageProcessor, Mask2FormerForUniversalSegmentation, SegformerForSemanticSegmentation,
                          SegformerImageProcessor)

MODEL = 'nvidia/segformer-b2-finetuned-cityscapes-1024-1024'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--model', choices=['segformer', 'mask2former'], default='segformer')
    ap.add_argument('--m2f_path', default='/root/data/weights/m2f_cs', help='local Mask2Former Swin-L Cityscapes')
    ap.add_argument('--out', default='road')
    a = ap.parse_args()
    track = os.path.basename(os.path.normpath(a.track_dir))
    out_dir = os.path.join(a.track_dir, a.out)
    os.makedirs(out_dir, exist_ok=True)
    if a.model == 'segformer':
        proc = SegformerImageProcessor.from_pretrained(MODEL)
        model = SegformerForSemanticSegmentation.from_pretrained(MODEL).cuda().eval().half()
    else:
        proc = AutoImageProcessor.from_pretrained(a.m2f_path)
        model = Mask2FormerForUniversalSegmentation.from_pretrained(a.m2f_path).cuda().eval().half()

    def class_probs(x):
        if a.model == 'segformer':
            return torch.softmax(model(pixel_values=x).logits.float(), 1)
        o = model(pixel_values=x)
        cls = o.class_queries_logits.float().softmax(-1)[..., :-1]
        msk = o.masks_queries_logits.float().sigmoid()
        p = torch.einsum('bqc,bqhw->bchw', cls, msk)
        return p / p.sum(1, keepdim=True).clamp(min=1e-6)
    for cam in ['f', 'fl', 'fr']:
        d = os.path.join(a.track_dir, f'{track}_images', f'{track}_{cam}_images')
        names = sorted(os.listdir(d))
        res, res_side = [], []
        for b in range(0, len(names), a.batch):
            imgs = [np.ascontiguousarray(cv2.imread(os.path.join(d, n))[:, :, ::-1]) for n in names[b:b + a.batch]]
            h, w = imgs[0].shape[:2]
            with torch.no_grad():
                x = proc(images=imgs, return_tensors='pt')['pixel_values'].cuda().half()
                p = class_probs(x)
                p = torch.nn.functional.interpolate(p, size=(h // 4, w // 4), mode='bilinear')
                road = p[:, 0] + p[:, 13:19].sum(1)   # road + car, truck, bus, train, motorcycle, bicycle
                side = p[:, 1]
            res.append((road.clamp(0, 1).cpu().numpy() * 255).astype(np.uint8))
            res_side.append((side.cpu().numpy() * 255).astype(np.uint8))
            if b % 400 == 0:
                print(cam, b, '/', len(names), flush=True)
        np.savez_compressed(os.path.join(out_dir, f'{cam}.npz'), names=np.array(names), road=np.concatenate(res),
                            side=np.concatenate(res_side))
        print('saved', cam, flush=True)


if __name__ == '__main__':
    main()
