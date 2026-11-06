#!/usr/bin/env python3
"""Labelme drafts of DYNAMIXEL top faces (4-point polygons) to correct by hand.

Rules: notes/top_face_label_rules.md. Center and direction come from seg_detector (YOLO-seg + color
edges); the size is the known face for the measured top height (measured size for tilted objects).
Corners sit on the support plane at the top height and are projected, so the oblique view shows.
Labels: top, top_tilted (lying, top 36-42 mm), check_me (seg_detector found it ambiguous: fix the
label or delete). The description holds top height, score, angle source and second/first peak.
Input frames come from extract_frames.py (<stem>_color.png, <stem>_depth.png or .npy, <stem>.json).
Output: <name>.png + labelme <name>.json per frame; with --coco also _annotations.coco.json (polygons,
categories top / top_tilted / check_me) for uploading the images with drafts to Roboflow.
"""
import argparse
import glob
import json
import math
import os
import shutil

from angle_debug import load_frame, load_task, on_plane, votes
import cv2
from ffw_vision.seg_detector import detect_seg, pixel_rays, project, yolo_segmenter
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLAT_HIGH = 0.031  # m; below: 28.5 mm up, top face 46.5 x 34
FLAT_34 = 0.036  # below: 34 mm up, top face 46.5 x 28.5; above (lying): tilted
STANDING = 0.042  # standing: top face 34 x 28.5
LONG, MID, SHORT = 0.0465, 0.034, 0.0285
CATEGORIES = ('top', 'top_tilted', 'check_me')


def face_size(obj, top):
    """(long, short) of the top face and the label for the measured top height."""
    if obj.ambiguous:
        return obj.length_m, obj.width_m, 'check_me'
    if top >= STANDING:
        return MID, SHORT, 'top'
    if top < FLAT_HIGH:
        return LONG, MID, 'top'
    if top < FLAT_34:
        return LONG, SHORT, 'top'
    return (min(max(obj.length_m, MID), LONG), min(max(obj.width_m, SHORT), MID), 'top_tilted')


def corners(obj, top, plane, k, d):
    """Top face corners in pixels: known size around the measured center and direction."""
    n, plane_d = plane
    center, toward = on_plane([[obj.u, obj.v], [obj.u + 10 * math.cos(obj.angle),
                                                obj.v + 10 * math.sin(obj.angle)]], k, d, n, plane_d, top)
    axis = (toward - center) / np.linalg.norm(toward - center)
    across = np.cross(n, axis)
    length, width, label = face_size(obj, top)
    pts = [center + sa * length / 2 * axis + sb * width / 2 * across
           for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    return project(pts, k, d), label


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('frames', nargs='+', help='*_color.png files or folders with them')
    parser.add_argument('--task', default=os.path.join(HERE, '..', 'config', 'dynamixel_basket_seg.yaml'))
    parser.add_argument('--model', default='', help='default: models/<model_file of the task>')
    parser.add_argument('--suffix', default='', help='only frames whose name ends with this, e.g. _0')
    parser.add_argument('--out', default='top_face_draft')
    parser.add_argument('--coco', action='store_true', help='also write _annotations.coco.json')
    args = parser.parse_args()

    params, model_file, imgsz, depth_scale = load_task(args.task)
    segment = yolo_segmenter(args.model or os.path.join(HERE, '..', 'models', model_file), imgsz)
    paths = []
    for f in args.frames:
        paths += sorted(glob.glob(os.path.join(f, '*_color.png'))) if os.path.isdir(f) else [f]
    paths = [p for p in paths if p[:-len('_color.png')].endswith(args.suffix)]
    os.makedirs(args.out, exist_ok=True)

    rays_cache, counts, names = {}, {}, set()
    coco = {'images': [], 'annotations': [],
            'categories': [{'id': i + 1, 'name': c, 'supercategory': 'dynamixel'} for i, c in enumerate(CATEGORIES)]}
    for path in paths:
        name, color, depth, k, d = load_frame(path, depth_scale)
        if name in names:
            raise SystemExit(f'two frames named {name}: image names must be unique')
        names.add(name)
        key = (color.shape, tuple(k), tuple(d))
        if key not in rays_cache:
            rays_cache[key] = pixel_rays(color.shape[1], color.shape[0], k, d)
        debug = []
        objects, plane = detect_seg(color, depth, k, d, segment(color, params.conf), params,
                                    rays_cache[key], debug)
        shapes = []
        for obj, info in zip(objects, debug):
            top = info['top_level']
            pts, label = corners(obj, top, plane, k, d)
            second = votes(info['edge_deg'], info['edge_w'])[3] if 'edge_deg' in info else 0.0
            shapes.append({'label': label, 'points': [[float(x), float(y)] for x, y in pts],
                           'group_id': None, 'shape_type': 'polygon', 'flags': {},
                           'description': f'h{top * 1000:.0f} score{obj.score:.2f} {obj.angle_source}'
                                          f' 2nd{second:.2f}'})
            counts[label] = counts.get(label, 0) + 1
        h, w = color.shape[:2]
        image_id = len(coco['images']) + 1
        coco['images'].append({'id': image_id, 'file_name': name + '.png', 'width': w, 'height': h})
        for shape in shapes:
            pts = np.array(shape['points'])
            x0, y0 = pts.min(axis=0)
            x1, y1 = pts.max(axis=0)
            coco['annotations'].append({
                'id': len(coco['annotations']) + 1, 'image_id': image_id,
                'category_id': CATEGORIES.index(shape['label']) + 1,
                'segmentation': [[round(float(v), 2) for v in pts.ravel()]],
                'bbox': [round(float(v), 2) for v in (x0, y0, x1 - x0, y1 - y0)],
                'area': round(float(cv2.contourArea(pts.astype(np.float32))), 2), 'iscrowd': 0})
        shutil.copy(path, os.path.join(args.out, name + '.png'))
        with open(os.path.join(args.out, name + '.json'), 'w') as fp:
            json.dump({'version': '5.5.0', 'flags': {}, 'shapes': shapes, 'imagePath': name + '.png',
                       'imageData': None, 'imageHeight': h, 'imageWidth': w}, fp, indent=1)
        note = '' if plane is not None else ' (no basket/plane)'
        print(name, ' '.join(f'{s["label"]}({s["description"]})' for s in shapes) + note)
    if args.coco:
        with open(os.path.join(args.out, '_annotations.coco.json'), 'w') as fp:
            json.dump(coco, fp)
    print(f'{len(paths)} frames, {counts}, wrote {args.out}/')


if __name__ == '__main__':
    main()
