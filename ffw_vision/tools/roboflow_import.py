#!/usr/bin/env python3
"""Roboflow COCO export of top-face polygons -> labelme folders under the original image names.

Roboflow renames images on export (<name>_png.rf.<hash>.jpg); the original PNG with the same name is
taken from --originals (the folders that were uploaded, e.g. top_face_train top_face_tilt_test).
Frames named *tilt_test* go to <out>/tilt_test, the rest to <out>/train. Refuses when an export
image size differs from its original (Roboflow preprocessing such as Resize was left on) or when
check_me labels remain. Polygons with more than 4 points are cut down to 4 corners and listed.
Rules: notes/top_face_label_rules.md
"""
import argparse
import glob
import json
import os
import re
import shutil

import cv2
import numpy as np

LABELS = ('top', 'top_tilted')
RF_NAME = re.compile(r'^(.+?)_(png|jpg|jpeg)\.rf\.[0-9a-f]+\.(jpg|jpeg|png)$')


def original_name(image):
    """Original file stem of a Roboflow export image."""
    extra = image.get('extra', {}).get('name')
    if extra:
        return os.path.splitext(os.path.basename(extra))[0]
    m = RF_NAME.match(os.path.basename(image['file_name']))
    return m.group(1) if m else os.path.splitext(os.path.basename(image['file_name']))[0]


def four_corners(points):
    """4 corners of a polygon: as is when it has 4, else simplified (None if that fails)."""
    pts = np.asarray(points, np.float32).reshape(-1, 2)
    if len(pts) == 4:
        return pts, False
    if len(pts) < 4:
        return None, True
    hull = cv2.convexHull(pts)
    length = cv2.arcLength(hull, True)
    for eps in np.linspace(0.01, 0.2, 40):
        approx = cv2.approxPolyDP(hull, eps * length, True).reshape(-1, 2)
        if len(approx) == 4:
            return approx.astype(np.float32), True
    return None, True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('export', help='unzipped Roboflow COCO export (searched for *.coco.json)')
    parser.add_argument('--originals', nargs='+', required=True, help='folders with the uploaded <name>.png')
    parser.add_argument('--out', default='top_face_labels')
    args = parser.parse_args()

    originals = {}
    for folder in args.originals:
        for path in glob.glob(os.path.join(folder, '*.png')):
            originals[os.path.splitext(os.path.basename(path))[0]] = path
    frames, problems, fixed = {}, [], []
    for coco_path in sorted(glob.glob(os.path.join(args.export, '**', '*.coco.json'), recursive=True)):
        coco = json.load(open(coco_path))
        names = {c['id']: c['name'] for c in coco['categories']}
        by_image = {}
        for ann in coco['annotations']:
            by_image.setdefault(ann['image_id'], []).append(ann)
        for image in coco['images']:
            name = original_name(image)
            if name not in originals:
                problems.append(f'{name}: no original PNG in --originals')
                continue
            h, w = cv2.imread(originals[name]).shape[:2]
            if (image['width'], image['height']) != (w, h):
                problems.append(f'{name}: export {image["width"]}x{image["height"]} vs original {w}x{h}'
                                ' (turn off Roboflow Resize)')
                continue
            shapes = []
            for ann in by_image.get(image['id'], []):
                label = names.get(ann['category_id'], '?')
                if label not in LABELS:
                    if label == 'check_me':
                        problems.append(f'{name}: check_me left')
                    continue  # Roboflow's workspace category (id 0) or unknown
                for seg in ann.get('segmentation') or []:
                    corners, changed = four_corners(seg)
                    if corners is None:
                        problems.append(f'{name}: {label} polygon with {len(seg) // 2} points')
                        continue
                    if changed:
                        fixed.append(f'{name}: {len(seg) // 2} points -> 4')
                    shapes.append({'label': label, 'points': corners.tolist(), 'group_id': None,
                                   'shape_type': 'polygon', 'flags': {}, 'description': ''})
            if name in frames:
                problems.append(f'{name}: in the export twice')
            frames[name] = (shapes, w, h)
    missing = sorted(set(originals) - set(frames))
    if problems:
        print('\n'.join(problems))
        raise SystemExit(f'{len(problems)} problems, nothing written')
    for name, (shapes, w, h) in frames.items():
        out = os.path.join(args.out, 'tilt_test' if 'tilt_test' in name else 'train')
        os.makedirs(out, exist_ok=True)
        shutil.copy(originals[name], os.path.join(out, name + '.png'))
        with open(os.path.join(out, name + '.json'), 'w') as fp:
            json.dump({'version': '5.5.0', 'flags': {}, 'shapes': shapes, 'imagePath': name + '.png',
                       'imageData': None, 'imageHeight': h, 'imageWidth': w}, fp, indent=1)
    if fixed:
        print('cut to 4 corners (check these):\n  ' + '\n  '.join(fixed))
    if missing:
        print(f'{len(missing)} uploaded images not in the export: {" ".join(missing[:10])}')
    counts = {}
    for shapes, _, _ in frames.values():
        for s in shapes:
            counts[s['label']] = counts.get(s['label'], 0) + 1
    print(f'{len(frames)} frames, {counts}, wrote {args.out}/train and {args.out}/tilt_test')


if __name__ == '__main__':
    main()
