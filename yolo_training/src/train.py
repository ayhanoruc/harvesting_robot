from __future__ import annotations

import argparse

from ultralytics import YOLO

from common import auto_device, model_arg, patch_ultralytics_offline_font_check, root_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train the cotton YOLO detector.")
    parser.add_argument("--data", default="configs/data.yaml", help="YOLO data.yaml path.")
    parser.add_argument("--model", default="weights/yolo11n.pt", help="Base model or .pt path.")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--workers", type=int, default=0, help="0 is safest on Windows.")
    parser.add_argument("--device", default="auto", help="auto, cpu, 0, 1, ...")
    parser.add_argument("--name", default="cotton_v5_yolo11n_512_b4_e80")
    parser.add_argument("--project", default="runs/detect")
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cache", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--plots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--amp", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--exist-ok", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    patch_ultralytics_offline_font_check()

    device = auto_device(args.device)
    model = YOLO(model_arg(args.model))
    results = model.train(
        data=str(root_path(args.data)),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=device,
        project=str(root_path(args.project)),
        name=args.name,
        patience=args.patience,
        seed=args.seed,
        deterministic=True,
        cache=args.cache,
        plots=args.plots,
        amp=args.amp,
        exist_ok=args.exist_ok,
    )
    print(results)


if __name__ == "__main__":
    main()
