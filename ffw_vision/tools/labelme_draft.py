#!/usr/bin/env python3
"""Labelme drafts to correct by hand.

Rule-based (default): non-blue blobs inside the basket; blobs longer than --max-len mm on the table
become split_me. With --model: the YOLO-seg masks; low scores and touching masks (one object may be
split in two) become check_me. labelme_to_yolo.py refuses split_me / check_me until relabeled.
"""
import argparse
import glob
import json
import os
import shutil

import cv2
import numpy as np


def fit_plane(points, rng, tol=0.004, iters=300):
    best = None
    for _ in range(iters):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inliers = np.abs((points - a) @ n) < tol
        if best is None or inliers.sum() > best.sum():
            best = inliers
    q = points[best]
    return np.linalg.svd(q - q.mean(0))[2][2]


def model_shapes(model, img, args):
    """labelme polygons from the model's masks (largest piece of each)."""
    r = model.predict(img, conf=args.conf, imgsz=args.imgsz, retina_masks=True, verbose=False)[0]
    if r.masks is None:
        return []
    masks = [m > 0.5 for m in r.masks.data.cpu().numpy()]
    scores = r.boxes.conf.cpu().numpy().tolist()
    grown = [cv2.dilate(m.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0 for m in masks]
    shapes = []
    for i, (m, score) in enumerate(zip(masks, scores)):
        cnts = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if not cnts or cv2.contourArea(max(cnts, key=cv2.contourArea)) < 30:
            continue
        touching = any((grown[i] & m2).any() for j, m2 in enumerate(masks) if j != i)
        notes = [f'{score:.2f}'] + (['touching'] if touching else [])
        check = score < args.check_below or touching
        shapes.append({'label': 'check_me' if check else 'dynamixel',
                       'points': cv2.approxPolyDP(max(cnts, key=cv2.contourArea), 1.0, True)
                       .reshape(-1, 2).tolist(),
                       'group_id': None, 'description': ' '.join(notes),
                       'shape_type': 'polygon', 'flags': {}, 'mask': None})
    return shapes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', help='extract_frames.py output folder')
    parser.add_argument('out', help='labelme folder to write')
    parser.add_argument('--frame', default='0', help='frame index suffix to use')
    parser.add_argument('--max-len', type=float, default=62.0)
    parser.add_argument('--model', help='YOLO-seg weights: draft from its masks instead of rules')
    parser.add_argument('--conf', type=float, default=0.25)  # low: deleting is easier than drawing
    parser.add_argument('--check-below', type=float, default=0.5)  # lower scores become check_me
    parser.add_argument('--imgsz', type=int, default=448)  # same as training
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(0)
    model = None
    if args.model:
        from ultralytics import YOLO  # only for --model
        model = YOLO(args.model)

    for f in sorted(glob.glob(os.path.join(args.captures, f'*_{args.frame}_color.png'))):
        stem = f[:-len('_color.png')]
        name = os.path.basename(stem)
        img = cv2.imread(f)
        H, W = img.shape[:2]
        if model is not None:
            write_draft(f, args.out, name, model_shapes(model, img, args), H, W)
            continue
        Z = cv2.imread(stem + '_depth.png', cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000
        k = json.load(open(stem + '.json'))['k']
        h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        blue = ((h >= 95) & (h <= 125) & (s >= 120) & (v >= 80)).astype(np.uint8)
        _, lab, st, _ = cv2.connectedComponentsWithStats(blue)
        cnts, _ = cv2.findContours((lab == 1 + np.argmax(st[1:, 4])).astype(np.uint8),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        basket = np.zeros_like(blue)
        cv2.drawContours(basket, [max(cnts, key=cv2.contourArea)], -1, 1, -1)
        inner = cv2.erode(basket, np.ones((5, 5), np.uint8))
        obj = cv2.morphologyEx(((inner > 0) & ~blue.astype(bool)).astype(np.uint8),
                               cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        obj = cv2.morphologyEx(obj, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        # camera-frame points are enough for sizes on the table plane
        vv, uu = np.mgrid[0:H, 0:W]
        valid = (Z > 0.07) & (Z < 1.5)
        pc = np.dstack(((uu - k[2]) * Z / k[0], (vv - k[5]) * Z / k[4], Z))
        ring = (cv2.dilate(basket, np.ones((41, 41), np.uint8)) > 0) & \
            (cv2.dilate(basket, np.ones((9, 9), np.uint8)) == 0) & valid
        n = fit_plane(pc[ring], rng)
        a = np.cross(n, [1, 0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(n, a)

        shapes = []
        for c in cv2.findContours(obj, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if cv2.contourArea(c) < 30:
                continue
            m = np.zeros_like(obj)
            cv2.drawContours(m, [c], -1, 1, -1)
            q = pc[(m > 0) & valid]
            length = max(cv2.minAreaRect((np.column_stack((q @ a, q @ b)) * 1000).astype(np.float32))[1]) \
                if len(q) >= 5 else 0
            shapes.append({'label': 'dynamixel' if length <= args.max_len else 'split_me',
                           'points': cv2.approxPolyDP(c, 1.0, True).reshape(-1, 2).tolist(),
                           'group_id': None, 'description': f'{length:.0f} mm',
                           'shape_type': 'polygon', 'flags': {}, 'mask': None})
        write_draft(f, args.out, name, shapes, H, W)
    print('wrote', args.out)


def write_draft(image, out, name, shapes, H, W):
    shutil.copy(image, os.path.join(out, name + '.png'))
    with open(os.path.join(out, name + '.json'), 'w') as fp:
        json.dump({'version': '5.5.0', 'flags': {}, 'shapes': shapes, 'imagePath': name + '.png',
                   'imageData': None, 'imageHeight': H, 'imageWidth': W}, fp, indent=1)
    print(name, ' '.join(f'{s["label"]}({s["description"]})' for s in shapes))


if __name__ == '__main__':
    main()
