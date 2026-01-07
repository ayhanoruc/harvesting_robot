#!/usr/bin/env python3
"""Run YOLO inference on a static image."""

from ultralytics import YOLO
import argparse
import os

MODEL_PATH = '/mnt/c/Users/ayhan/harvesting_ws/install/orchestrator/share/orchestrator/models/best.pt'
DEFAULT_IMAGE = '/mnt/c/Users/ayhan/harvesting_ws/src/docs/figures/cluster_with_unripe_bolls.png'

def main():
    parser = argparse.ArgumentParser(description='Run YOLO on static image')
    parser.add_argument('image', nargs='?', default=DEFAULT_IMAGE, help='Image path')
    parser.add_argument('--conf', type=float, default=0.3, help='Confidence threshold')
    parser.add_argument('--model', default=MODEL_PATH, help='Model path')
    parser.add_argument('--output', default='/mnt/c/Users/ayhan/harvesting_ws/src/docs/figures', help='Output dir')
    args = parser.parse_args()

    print(f'Model: {args.model}')
    print(f'Image: {args.image}')
    print(f'Confidence: {args.conf}')

    model = YOLO(args.model)
    print(f'Classes: {model.names}')

    results = model.predict(
        source=args.image,
        conf=args.conf,
        save=True,
        project=args.output,
        name='yolo_output'
    )

    print(f'\n{"="*50}')
    print(f'DETECTIONS (conf >= {args.conf}):')
    print(f'{"="*50}')

    for r in results:
        if len(r.boxes) == 0:
            print('No detections found!')
        else:
            for i, box in enumerate(r.boxes):
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                print(f'  [{i}] {cls_name}: conf={conf:.3f}, bbox=({x1},{y1})-({x2},{y2})')

    output_dir = os.path.join(args.output, 'yolo_output')
    print(f'\nSaved to: {output_dir}/')

if __name__ == '__main__':
    main()
