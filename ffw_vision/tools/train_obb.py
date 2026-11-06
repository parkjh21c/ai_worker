#!/usr/bin/env python3
"""Train a YOLO11-obb model on the DYNAMIXEL top-face dataset (labelme_to_obb.py)."""
import argparse

from ultralytics import YOLO

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--model', default='yolo11n-obb.pt')  # DOTA-pretrained start point, downloaded if missing
parser.add_argument('--data', default='dynamixel_obb/data.yaml')
parser.add_argument('--project', default='runs')
parser.add_argument('--name', default='obb_n640')
parser.add_argument('--epochs', type=int, default=200)
parser.add_argument('--device', default='0')  # GPU index, or 'cpu'
parser.add_argument('--imgsz', type=int, default=640)  # network input, multiple of 32
args = parser.parse_args()

model = YOLO(args.model)
model.train(
    data=args.data,
    epochs=args.epochs,
    patience=60,         # stop early when val has not improved for this many epochs
    imgsz=args.imgsz,
    batch=8,             # lower it if CUDA runs out of memory
    device=args.device,
    workers=2,
    degrees=180,         # any rotation: the box angle is what the model must learn
    fliplr=0.5,
    flipud=0.5,
    mosaic=1.0,          # off for the last 10 epochs
    project=args.project,
    name=args.name,      # weights in runs/<name>/weights/best.pt
    plots=True,
)
