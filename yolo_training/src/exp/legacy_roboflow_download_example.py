from __future__ import annotations

import os


def main() -> None:
    try:
        from roboflow import Roboflow
    except ImportError as exc:
        raise SystemExit("Install with: pip install roboflow") from exc

    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        raise SystemExit("Set ROBOFLOW_API_KEY before running this example.")

    rf = Roboflow(api_key=api_key)
    project = rf.workspace("deniz-drin5").project("cotton-boll-and-cluster")
    project.version(5).download("yolov11")


if __name__ == "__main__":
    main()
