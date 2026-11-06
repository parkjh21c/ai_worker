#!/usr/bin/env python3
"""Labelme top-face polygons -> YOLO OBB dataset (one class, dynamixel).

Label line: 0 x1 y1 x2 y2 x3 y3 x4 y4 (normalized corners; ultralytics fits the rotated box).
Validation frames are picked by name (--val, regex on the frame name), the rest is training.
Refuses frames with labels other than top / top_tilted (e.g. check_me) or polygons without 4 points.
"""
import argparse
import glob
import json
import os
import re
import shutil

LABELS = ('top', 'top_tilted')
# whole recordings: dump rounds 06/12/18, the wrist2 set, every 5th tilt scene
DEFAULT_VAL = r'^(dump_r(06|12|18)_|wrist2_|tilt_scene(05|10|15|20)_)'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('labelme', nargs='+', help='folders with <name>.png + labelme <name>.json')
    parser.add_argument('--val', default=DEFAULT_VAL, help='regex: frame names that go to validation')
    parser.add_argument('--out', default='dynamixel_obb')
    args = parser.parse_args()

    val = re.compile(args.val)
    frames, problems = [], []
    for folder in args.labelme:
        for js in sorted(glob.glob(os.path.join(folder, '*.json'))):
            data = json.load(open(js))
            name = os.path.splitext(os.path.basename(js))[0]
            w, h = data['imageWidth'], data['imageHeight']
            lines = []
            for s in data['shapes']:
                if s['label'] not in LABELS:
                    problems.append(f'{name}: label {s["label"]}')
                elif len(s['points']) != 4:
                    problems.append(f'{name}: {len(s["points"])} points')
                else:
                    xy = [min(max(x / w, 0.0), 1.0) for x, _ in s['points']]
                    yx = [min(max(y / h, 0.0), 1.0) for _, y in s['points']]
                    lines.append('0 ' + ' '.join(f'{x:.6f} {y:.6f}' for x, y in zip(xy, yx)))
            frames.append((name, os.path.join(folder, data['imagePath']), lines))
    if problems:
        print('\n'.join(problems))
        raise SystemExit(f'{len(problems)} problems, nothing written')
    counts = {'train': [0, 0], 'val': [0, 0]}
    for name, image, lines in frames:
        split = 'val' if val.search(name) else 'train'
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(args.out, sub, split), exist_ok=True)
        shutil.copy(image, os.path.join(args.out, 'images', split, name + '.png'))
        with open(os.path.join(args.out, 'labels', split, name + '.txt'), 'w') as fp:
            fp.write(''.join(line + '\n' for line in lines))
        counts[split][0] += 1
        counts[split][1] += len(lines)
    with open(os.path.join(args.out, 'data.yaml'), 'w') as fp:
        # no 'path': ultralytics then reads the splits next to this file, wherever it is copied
        fp.write('train: images/train\nval: images/val\nnames:\n  0: dynamixel\n')
    print(f'train {counts["train"][0]} images / {counts["train"][1]} boxes,'
          f' val {counts["val"][0]} images / {counts["val"][1]} boxes, wrote {args.out}/data.yaml')


if __name__ == '__main__':
    main()
