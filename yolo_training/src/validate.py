from __future__ import annotations

import argparse

from ultralytics import YOLO

from common import auto_device, model_arg, patch_ultralytics_offline_font_check, root_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate/test a trained cotton YOLO model.")
    parser.add_argument("--model", default="weights/best.pt", help="Trained .pt file.")
    parser.add_argument("--data", default="configs/data.yaml", help="YOLO data.yaml path.")
    parser.add_argument("--split", default="test", choices=["train", "val", "test"])
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--project", default="runs/val")
    parser.add_argument("--name", default="cotton_test")
    parser.add_argument("--plots", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    patch_ultralytics_offline_font_check()

    model = YOLO(model_arg(args.model))
    metrics = model.val(
        data=str(root_path(args.data)),
        split=args.split,
        imgsz=args.imgsz,
        batch=args.batch,
        device=auto_device(args.device),
        conf=args.conf,
        iou=args.iou,
        project=str(root_path(args.project)),
        name=args.name,
        plots=args.plots,
    )
    print(metrics)


if __name__ == "__main__":
    main()
