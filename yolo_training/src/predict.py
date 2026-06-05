from __future__ import annotations

import argparse

from ultralytics import YOLO

from common import auto_device, model_arg, patch_ultralytics_offline_font_check, root_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run image, video, folder, or webcam inference.")
    parser.add_argument("--model", default="weights/best.pt", help="Trained .pt file.")
    parser.add_argument("--source", required=True, help="Image, folder, video, RTSP URL, or webcam index.")
    parser.add_argument("--conf", type=float, default=0.54, help="Balanced threshold from F1 curve.")
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--project", default="runs/predict")
    parser.add_argument("--name", default="cotton_predictions")
    parser.add_argument("--save", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--save-txt", action="store_true")
    parser.add_argument("--save-conf", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--exist-ok", action="store_true")
    return parser.parse_args()


def source_arg(value: str) -> str | int:
    return int(value) if value.isdigit() else value


def main() -> None:
    args = parse_args()
    patch_ultralytics_offline_font_check()

    model = YOLO(model_arg(args.model))
    results = model.predict(
        source=source_arg(args.source),
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=auto_device(args.device),
        project=str(root_path(args.project)),
        name=args.name,
        save=args.save,
        save_txt=args.save_txt,
        save_conf=args.save_conf,
        show=args.show,
        exist_ok=args.exist_ok,
    )
    print(f"Saved {len(results)} prediction result(s).")


if __name__ == "__main__":
    main()
