#!/usr/bin/env python3
"""Show how seg_detector picks each object's direction: edge votes, histogram, rect fallback, optional ground truth.

Candidate 'tight': the same edge_angle with the vote region cut to mask pixels that are not basket
blue (+1 px), so mask parts bled onto the basket wall do not vote.
Input frames come from extract_frames.py (<stem>_color.png, <stem>_depth.png or .npy, <stem>.json).
Ground truth (optional): labelme json per frame (<stem>_color.json or <stem>.json in --gt-dir) with
4-point top-face polygons (notes/top_face_label_rules.md, labels top / top_tilted) or 'line' shapes
along a straight top-face edge. A shape belongs to the object whose region holds its center.
Angles are on the support plane, mod 90 deg (box axes), in seg_detector.plane_axes coordinates.
Frames <scene>_0 / <scene>_1 of one recording are paired by object position (frame-to-frame jumps).
--detector obb runs obb_detector with an OBB model (--obb-model) instead; 'rect' is then the raw box
angle and 'current' the output (box angle, or refined by color edges with --obb-refine deg).
--detector hybrid runs hybrid_detector (both models, as the object_detector node with yolo_hybrid):
'rect' is the raw box angle, 'current' the output, source hybrid:seg / hybrid:obb-peak / hybrid:obb.
"""
import argparse
from dataclasses import fields
import glob
import json
import math
import os

import cv2
import time

from ffw_vision.hybrid_detector import detect_hybrid, HybridParams
from ffw_vision.obb_detector import box_frame, detect_obb, ObbParams, yolo_obb_detector
from ffw_vision.seg_detector import basket_masks, detect_seg, edge_angle, pixel_rays, plane_axes
from ffw_vision.seg_detector import project, SegParams, yolo_segmenter
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
FONT = cv2.FONT_HERSHEY_SIMPLEX
CHOSEN, RECT, TIGHT, GT = (0, 255, 0), (0, 255, 255), (0, 140, 255), (255, 255, 255)
PAIR_PX = 5.0  # same object in both frames of a scene when centers are this close


def load_task(path):
    """SegParams + model_file + imgsz + depth_scale from a task yaml."""
    p = yaml.safe_load(open(path))['/**']['ros__parameters']
    names = {f.name for f in fields(SegParams)}
    kwargs = {k: v for k, v in p.items() if k in names}
    return SegParams(**kwargs), p.get('model_file', ''), int(p.get('imgsz', 640)), p.get('depth_scale', 0.001)


def load_frame(color_path, depth_scale):
    stem = color_path[:-len('_color.png')]
    color = cv2.imread(color_path)
    if os.path.exists(stem + '_depth.png'):
        depth = cv2.imread(stem + '_depth.png', cv2.IMREAD_UNCHANGED).astype(np.float32) * depth_scale
        depth[depth <= 0] = np.nan
    else:
        depth = np.load(stem + '_depth.npy').astype(np.float32)
    meta = json.load(open(stem + '.json'))
    return os.path.basename(stem), color, depth, meta['k'], meta['d']


def gt_lines(gt_dir, name):
    for cand in (name + '_color.json', name + '.json'):
        path = os.path.join(gt_dir, cand)
        if os.path.exists(path):
            shapes = json.load(open(path))['shapes']
            return [np.array(s['points'], np.float64) for s in shapes
                    if s.get('shape_type') == 'line'
                    or (s.get('shape_type') == 'polygon' and len(s['points']) == 4
                        and s['label'] in ('top', 'top_tilted'))]
    return []


def on_plane(uv, k, d, n, plane_d, level):
    """Camera-frame points seen at pixels uv on the support plane raised by level."""
    xy = cv2.undistortPoints(np.asarray(uv, np.float64).reshape(-1, 1, 2),
                             np.array(k, np.float64).reshape(3, 3), np.array(d, np.float64))
    rays = np.column_stack((xy.reshape(-1, 2), np.ones(len(xy))))
    return rays * ((level - plane_d) / (rays @ n))[:, None]


def votes(deg, w):
    """Same peak rule as seg_detector.edge_angle: (hist, peak deg, share, second peak / first)."""
    hist, _ = np.histogram(deg, bins=90, range=(0, 90), weights=w)
    if hist.sum() <= 0:
        return hist, None, 0.0, 0.0
    kernel = np.exp(-0.5 * (np.arange(-9, 10) / 3.0) ** 2)
    smooth = np.convolve(np.concatenate((hist[-9:], hist, hist[:9])), kernel, 'same')[9:-9]
    peak = int(np.argmax(smooth))
    near = hist[[(peak + i) % 90 for i in range(-5, 6)]].sum()
    far = [i for i in range(90) if min((i - peak) % 90, (peak - i) % 90) > 10]
    second = smooth[far].max() / smooth[peak] if far else 0.0
    return hist, peak + 0.5, float(near / hist.sum()), float(second)


def diff90(x, y):
    if x is None or y is None:
        return None
    return abs((x - y + 45) % 90 - 45)


def mean90(degs):
    t = np.radians(np.asarray(degs) * 4)
    return math.degrees(math.atan2(np.sin(t).mean(), np.cos(t).mean())) / 4 % 90


def hue(deg):
    """BGR color for an angle mod 90 (0 and 90 share one color)."""
    hsv = np.uint8([[[int(deg / 90 * 180) % 180, 255, 255]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def fmt(x):
    return '-' if x is None else f'{x:.1f}'


def analyze(obj, info, tight, n, plane_d, k, d, a, b, lines):
    """Per-object numbers: chosen/edge/rect/tight angles, vote strength, ground truth."""
    r = {'source': obj.angle_source, 'rect': info['rect_deg'], 'edge': info.get('peak_deg'),
         'share': info.get('share'), 'n_px': len(info.get('edge_w', [])), 'u': obj.u, 'v': obj.v}
    level = info['top_level']
    # the output direction on the plane, whichever detector made it
    r['center'], toward = on_plane([[obj.u, obj.v], [obj.u + 10 * math.cos(obj.angle),
                                                     obj.v + 10 * math.sin(obj.angle)]], k, d, n, plane_d, level)
    r['chosen'] = math.degrees(math.atan2((toward - r['center']) @ b, (toward - r['center']) @ a)) % 90
    # long side direction mod 180 (long and short told apart), output and seg's own
    r['long180'] = math.degrees(math.atan2((toward - r['center']) @ b, (toward - r['center']) @ a)) % 180
    r['seg_long180'] = (math.degrees(math.atan2(info['axis'] @ b, info['axis'] @ a)) % 180
                        if 'axis' in info else None)
    if r['n_px']:
        r['hist_all'], _, _, r['second'] = votes(info['edge_deg'], info['edge_w'])
    r['t_px'], r['t_source'], r['tight'], r['t_share'] = 0, '-', None, None
    if tight is not None:
        # tight: what seg_detector would output with the cut region (rect when edge_angle gives up)
        r['t_px'] = len(tight.get('edge_w', []))
        r['t_source'] = 'edges' if tight.get('ok') else 'min_area_rect'
        r['tight'] = tight['peak_deg'] if tight.get('ok') else r['rect']
        r['t_share'] = tight.get('share')
        if r['t_px']:
            r['t_hist'], _, _, r['t_second'] = votes(tight['edge_deg'], tight['edge_w'])
    r['gt_lines'], mine, centers = [], [], []
    for shape in lines:  # already assigned to this object (assign_gt)
        p = on_plane(shape, k, d, n, plane_d, level)
        # a line gives one edge, a polygon its four (all box axes mod 90)
        edges = [p[1] - p[0]] if len(p) == 2 else [p[(i + 1) % 4] - p[i] for i in range(4)]
        mine += [math.degrees(math.atan2(e @ b, e @ a)) % 90 for e in edges]
        if len(p) == 4:
            centers.append(p.mean(axis=0))
            gt_axis = box_frame(p, n)[1]
            r['gt_long180'] = math.degrees(math.atan2(gt_axis @ b, gt_axis @ a)) % 180
        r['gt_lines'].append(shape)
    r['gt'] = mean90(mine) if mine else None
    gl = r.get('gt_long180')
    d180 = lambda x: None if x is None or gl is None else abs((x - gl + 90) % 180 - 90)
    r['long_swapped'] = None if d180(r['long180']) is None else int(d180(r['long180']) > 45)
    r['seg_long_swapped'] = None if d180(r['seg_long180']) is None else int(d180(r['seg_long180']) > 45)
    r['err_center_mm'] = (float(np.linalg.norm(np.mean(centers, axis=0) - r['center'])) * 1000
                          if centers else None)
    for key in ('chosen', 'rect', 'tight'):
        r['err_' + key] = diff90(r.get(key), r['gt'])
    return r


def draw_axes(img, r, deg, k, d, n, a, b, x0, y0, s, color, thick):
    """Both box axes for a plane angle, 25 mm each way from the top center."""
    if deg is None:
        return
    for extra in (0, 90):
        t = math.radians(deg + extra)
        axis = math.cos(t) * a + math.sin(t) * b
        p, q = project([r['center'] - 0.025 * axis, r['center'] + 0.025 * axis], k, d)
        cv2.line(img, (int((p[0] - x0) * s), int((p[1] - y0) * s)),
                 (int((q[0] - x0) * s), int((q[1] - y0) * s)), color, thick, cv2.LINE_AA)


def tile(color, obj, info, tight, r, k, d, n, a, b, s):
    """[color + axes | votes: gray = dropped by tight, colored = tight | histograms + numbers]."""
    ys, xs = np.nonzero(info['region'])
    x0, y0 = max(xs.min() - 10, 0), max(ys.min() - 10, 0)
    x1, y1 = min(xs.max() + 11, color.shape[1]), min(ys.max() + 11, color.shape[0])
    crop = cv2.resize(color[y0:y1, x0:x1], None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
    left = crop.copy()
    cs, _ = cv2.findContours(info['mask'].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(left, [((c - [x0, y0]) * s + s // 2).astype(np.int32) for c in cs], -1, (128, 128, 128), 1)
    for shape in r['gt_lines']:
        cv2.polylines(left, [((shape - [x0, y0]) * s).astype(np.int32)], len(shape) > 2, GT, 2, cv2.LINE_AA)
    draw_axes(left, r, r['rect'], k, d, n, a, b, x0, y0, s, RECT, 1)
    draw_axes(left, r, r['chosen'], k, d, n, a, b, x0, y0, s, CHOSEN, 2)
    draw_axes(left, r, r['tight'], k, d, n, a, b, x0, y0, s, TIGHT, 1)

    mid = (crop * 0.35).astype(np.uint8)
    for i in range(r['n_px']):
        u, v = (info['edge_u'][i] - x0) * s, (info['edge_v'][i] - y0) * s
        cv2.rectangle(mid, (u, v), (u + s - 1, v + s - 1), (110, 110, 110), -1)
    for i in range(r['t_px']):
        u, v = (tight['edge_u'][i] - x0) * s, (tight['edge_v'][i] - y0) * s
        cv2.rectangle(mid, (u, v), (u + s - 1, v + s - 1), hue(tight['edge_deg'][i]), -1)

    h = max(crop.shape[0], 250)
    right = np.zeros((h, 4 * 90 + 20, 3), np.uint8)
    base = h - 120
    hists = [r[key] for key in ('hist_all', 't_hist') if key in r]
    top = max(max(x.max() for x in hists), 1e-9) if hists else 1.0
    if 't_hist' in r:  # tight votes as colored bars
        for i in range(90):
            x = 10 + 4 * i
            cv2.rectangle(right, (x, base - int(r['t_hist'][i] / top * (base - 10))), (x + 3, base), hue(i + 0.5), -1)
    if 'hist_all' in r:  # current votes as a gray outline
        pts = [(10 + 4 * i + 2, base - int(r['hist_all'][i] / top * (base - 10))) for i in range(90)]
        cv2.polylines(right, [np.int32(pts)], False, (170, 170, 170), 1)
    for deg, c in ((r['gt'], GT), (r['rect'], RECT), (r['tight'], TIGHT), (r['chosen'], CHOSEN)):
        if deg is not None:
            x = int(10 + 4 * deg)
            cv2.line(right, (x, base + 2), (x, base + 12), c, 2)
    rows = [f"current {fmt(r['chosen'])} ({r['source']})  gt {fmt(r['gt'])}  err {fmt(r['err_chosen'])}",
            f"  share {fmt(r['share'])} 2nd/1st {fmt(r.get('second'))} px {r['n_px']}",
            f"tight {fmt(r['tight'])} ({r['t_source']})  err {fmt(r['err_tight'])}",
            f"  share {fmt(r['t_share'])} 2nd/1st {fmt(r.get('t_second'))} px {r['t_px']}",
            f"rect {fmt(r['rect'])} err {fmt(r['err_rect'])}",
            f"{obj.pose_label} h={info['top_level'] * 1000:.0f}mm score {obj.score:.2f}"]
    for i, text in enumerate(rows):
        cv2.putText(right, text, (6, base + 28 + 16 * i), FONT, 0.4, (230, 230, 230), 1, cv2.LINE_AA)

    def fit(img):
        out = np.zeros((h, img.shape[1], 3), np.uint8)
        out[:img.shape[0]] = img
        return out
    return np.hstack((fit(left), fit(mid), right))


def assign_gt(shapes, objects, debug):
    """Shapes per object: each shape goes to the nearest object whose region (+5 px) holds its center."""
    out = [[] for _ in objects]
    for shape in shapes:
        mu, mv = np.round(shape.mean(axis=0)).astype(int)
        best = None
        for i, (obj, info) in enumerate(zip(objects, debug)):
            region = cv2.dilate(info['region'].astype(np.uint8), np.ones((11, 11), np.uint8)).astype(bool)
            if 0 <= mv < region.shape[0] and 0 <= mu < region.shape[1] and region[mv, mu]:
                dist = math.hypot(mu - obj.u, mv - obj.v)
                if best is None or dist < best[0]:
                    best = (dist, i)
        if best is not None:
            out[best[1]].append(shape)
    return out


def tight_votes(info, blue, inner, gray, rays, n, plane_d, params):
    """edge_angle on mask pixels that are not basket blue, +1 px."""
    keep = (info['mask'] & ~blue).astype(np.uint8)
    region = cv2.dilate(keep, np.ones((3, 3), np.uint8)).astype(bool) & inner
    out = {}
    out['ok'] = edge_angle(gray, region, rays, n, plane_d, info['top_level'], params, out) is not None
    return out


def scene_spread(results, key, match_px=8.0):
    """Per object over all frames of a scene (<scene>_<n>): [(scene, u, v, frames, spread deg, angles)].

    Objects are matched to the first frame's by position; spread = largest distance from the
    circular mean (mod 90).
    """
    scenes = {}
    for name in sorted(results, key=lambda s: (s.rsplit('_', 1)[0], int(s.rsplit('_', 1)[1])
                                                if s.rsplit('_', 1)[-1].isdigit() else 0)):
        scenes.setdefault(name.rsplit('_', 1)[0], []).append(name)
    out = []
    for scene, names in scenes.items():
        if len(names) < 3:
            continue
        for r0 in results[names[0]]:
            angles = []
            for name in names:
                near = [r for r in results[name] if math.hypot(r0['u'] - r['u'], r0['v'] - r['v']) < match_px]
                if near:
                    angles.append(near[0][key])
            if len(angles) >= 3:
                mean = mean90(angles)
                out.append((scene, r0['u'], r0['v'], len(angles), max(diff90(x, mean) for x in angles), angles))
    return out


def frame_jumps(results, key):
    """Angle differences of one object between <scene>_0 and <scene>_1 (matched by position)."""
    out = []
    for name, objs in results.items():
        if not name.endswith('_0') or name[:-1] + '1' not in results:
            continue
        for r0 in objs:
            near = [r1 for r1 in results[name[:-1] + '1']
                    if math.hypot(r0['u'] - r1['u'], r0['v'] - r1['v']) < PAIR_PX]
            if near:
                out.append((name[:-2], diff90(r0[key], near[0][key])))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('frames', nargs='+', help='*_color.png files or folders with them')
    parser.add_argument('--task', default=os.path.join(HERE, '..', 'config', 'dynamixel_basket_seg.yaml'))
    parser.add_argument('--model', default='', help='default: models/<model_file of the task>')
    parser.add_argument('--gt-dir', default='', help='labelme jsons: top-face polygons or line shapes')
    parser.add_argument('--detector', choices=('seg', 'obb', 'hybrid'), default='seg')
    parser.add_argument('--hybrid-window', type=float, default=25.0, help='deg, hybrid_window_deg')
    parser.add_argument('--obb-model', default='', help='OBB weights for --detector obb')
    parser.add_argument('--obb-refine', type=float, default=0.0, help='deg window for the color edge refinement')
    parser.add_argument('--imgsz', type=int, default=0, help='network input; default: the task file')
    parser.add_argument('--scale', type=int, default=4)
    parser.add_argument('--out', default='angle_debug')
    args = parser.parse_args()

    params, model_file, imgsz, depth_scale = load_task(args.task)
    imgsz = args.imgsz or imgsz
    obb = args.detector in ('obb', 'hybrid')
    seg_values = {f.name: getattr(params, f.name) for f in fields(SegParams)}
    seg_model = args.model or os.path.join(HERE, '..', 'models', model_file)
    if obb and not args.obb_model:
        raise SystemExit(f'--detector {args.detector} needs --obb-model')
    if args.detector == 'obb':
        params = ObbParams(**seg_values, refine_deg=args.obb_refine)
        segment, detect = yolo_obb_detector(args.obb_model, imgsz), detect_obb
    elif args.detector == 'hybrid':
        params = HybridParams(**seg_values, hybrid_window_deg=args.hybrid_window)
        seg_net, box_net = yolo_segmenter(seg_model, imgsz), yolo_obb_detector(args.obb_model, imgsz)

        def segment(color, conf):
            return seg_net(color, conf), box_net(color, conf)

        def detect(color, depth, k, d, found, p, rays, debug):
            return detect_hybrid(color, depth, k, d, found[0], found[1], p, rays, debug)
    else:
        segment, detect = yolo_segmenter(seg_model, imgsz), detect_seg
    gt_total, gt_matched, infer_ms, raw = 0, 0, [], {}
    paths = []
    for f in args.frames:
        paths += sorted(glob.glob(os.path.join(f, '*_color.png'))) if os.path.isdir(f) else [f]
    os.makedirs(args.out, exist_ok=True)

    cols = ('chosen', 'share', 'second', 'tight', 't_share', 't_second', 'rect', 'gt', 'err_chosen', 'err_tight',
            'err_rect', 'err_center_mm', 'long180', 'seg_long180', 'gt_long180', 'long_swapped',
            'seg_long_swapped')
    head = 'frame,obj,u,v,pose,top_mm,length_mm,width_mm,ambiguous,score,source,t_source,' + ','.join(cols)
    rows, rays_cache, results = [], {}, {}
    for path in paths:
        name, color, depth, k, d = load_frame(path, depth_scale)
        key = (color.shape, tuple(k), tuple(d))
        if key not in rays_cache:
            rays_cache[key] = pixel_rays(color.shape[1], color.shape[0], k, d)
        rays = rays_cache[key]
        debug = []
        start = time.perf_counter()
        found = segment(color, params.conf)
        infer_ms.append((time.perf_counter() - start) * 1000)
        if args.detector == 'obb':  # raw model boxes, to try settings without the model (boxes.json)
            raw[name] = [[np.asarray(c).tolist(), float(sc)] for c, sc in found]
        else:
            raw[name] = []
        objects, plane = detect(color, depth, k, d, found, params, rays, debug)
        if plane is None:
            print(f'{name}: no basket/plane')
            continue
        n, plane_d = plane
        a, b = plane_axes(n)
        # same inputs as detect_seg uses for edge_angle
        basket, blue = basket_masks(color, params)
        inner = cv2.erode(basket, np.ones((params.inner_erode_px,) * 2, np.uint8)).astype(bool)
        gray = cv2.GaussianBlur(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), (3, 3), 0).astype(np.float32)
        shapes = gt_lines(args.gt_dir, name) if args.gt_dir else []
        lines = assign_gt(shapes, objects, debug)
        gt_total += sum(len(sh) > 2 for sh in shapes)
        gt_matched += sum(len(sh) > 2 for per in lines for sh in per)
        results[name] = []
        for i, (obj, info) in enumerate(zip(objects, debug)):
            tight = None if obb else tight_votes(info, blue, inner, gray, rays, n, plane_d, params)
            if not obb:  # edge votes per object, to combine with saved OBB boxes (votes.json)
                raw[name].append({'u': obj.u, 'v': obj.v, 'top': info['top_level'],
                                  'deg': np.round(info.get('edge_deg', []), 2).tolist(),
                                  'w': np.round(info.get('edge_w', []), 1).tolist()})
            r = analyze(obj, info, tight, n, plane_d, k, d, a, b, lines[i])
            results[name].append(r)
            cv2.imwrite(os.path.join(args.out, f'{name}_obj{i}.png'),
                        tile(color, obj, info, tight, r, k, d, n, a, b, args.scale))
            rows.append([name, i, f'{obj.u:.1f}', f'{obj.v:.1f}', obj.pose_label, f"{info['top_level'] * 1000:.1f}",
                         f'{obj.length_m * 1000:.1f}', f'{obj.width_m * 1000:.1f}', int(obj.ambiguous),
                         f'{obj.score:.2f}', r['source'], r['t_source']] + [fmt(r.get(c)) for c in cols])
            print(','.join(map(str, rows[-1])))
    if args.detector != 'hybrid':
        with open(os.path.join(args.out, 'boxes.json' if obb else 'votes.json'), 'w') as f:
            json.dump(raw, f)
    with open(os.path.join(args.out, 'summary.csv'), 'w') as f:
        f.write(head + '\n' + '\n'.join(','.join(map(str, r)) for r in rows) + '\n')

    print(f'\n{len(rows)} objects, detector {args.detector}'
          + (f' (refine {args.obb_refine:g} deg)' if args.detector == 'obb' else '')
          + (f' (window {args.hybrid_window:g} deg)' if args.detector == 'hybrid' else '')
          + f', model inference median {np.median(infer_ms[1:] or infer_ms):.0f} ms')
    if gt_total:
        print(f'ground truth top faces {gt_total}: matched {gt_matched}, missed {gt_total - gt_matched}')
        mm = [r['err_center_mm'] for v in results.values() for r in v if r['err_center_mm'] is not None]
        print(f'center vs ground truth: median {np.median(mm):.1f} mm, >3mm {sum(x > 3 for x in mm)},'
              f' >5mm {sum(x > 5 for x in mm)}, max {max(mm):.1f} mm')
    if gt_total:
        for key, label in (('long_swapped', 'output'), ('seg_long_swapped', 'seg mask')):
            flags = [(name, r) for name, v in results.items() for r in v if r.get(key) is not None]
            bad = [f'{name}({r["u"]:.0f},{r["v"]:.0f})' for name, r in flags if r[key]]
            print(f'long side vs label ({label}): swapped {len(bad)} of {len(flags)}  {" ".join(bad)}')
    keys = (('chosen', 'current'), ('rect', 'obb raw')) if obb else (('chosen', 'current'), ('tight', 'tight'))
    for key, label in keys:
        j = frame_jumps(results, key)
        e = np.array([x for _, x in j])
        if len(e):
            bad = ' '.join(s for s, x in j if x > 5)
            print(f'{label:8s} frame-to-frame (pairs {len(e)}): >3deg {(e > 3).sum()} >5deg {(e > 5).sum()}'
                  f' >10deg {(e > 10).sum()}  [{bad}]')
        if not obb:
            sec = np.array([r.get('second' if key == 'chosen' else 't_second', 0.0)
                            for v in results.values() for r in v])
            print(f'{label:8s} 2nd/1st >= 0.8: {(sec >= 0.8).sum()}')
        gt = [r['err_' + key] for v in results.values() for r in v if r['err_' + key] is not None]
        if gt:
            print(f'{label:8s} vs ground truth: n={len(gt)} mean {np.mean(gt):.1f} median {np.median(gt):.1f}'
                  f' >5deg {sum(x > 5 for x in gt)} >10deg {sum(x > 10 for x in gt)} max {max(gt):.1f}')
    spread = scene_spread(results, 'chosen')
    if spread:
        print(f'\nper object over the frames of a scene (current, >= 3 frames): {len(spread)} objects')
        for scene, u, v, count, worst, angles in sorted(spread, key=lambda x: -x[4]):
            tag = '  <- unstable' if worst > 5 else ''
            print(f'  {scene} ({u:.0f},{v:.0f}) frames {count} spread {worst:.1f} deg:'
                  f' {" ".join(f"{x:.0f}" for x in angles)}{tag}')
    if not obb:
        moved = [r for v in results.values() for r in v if diff90(r['chosen'], r['tight']) > 3]
        print(f'tight differs from current by >3deg: {len(moved)}')
    sources = {}
    for v in results.values():
        for r in v:
            sources[r['source']] = sources.get(r['source'], 0) + 1
    print(f'direction sources: {sources}')
    print(f'wrote {args.out}/ (green = current, orange = tight, yellow = rect / obb raw, white = ground truth)')


if __name__ == '__main__':
    main()
