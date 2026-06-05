from __future__ import annotations

import argparse
import csv
from pathlib import Path

from common import root_path


METRICS = [
    "metrics/precision(B)",
    "metrics/recall(B)",
    "metrics/mAP50(B)",
    "metrics/mAP50-95(B)",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize Ultralytics results.csv.")
    parser.add_argument("--results", default="metrics/results.csv")
    return parser.parse_args()


def as_float(row: dict[str, str], key: str) -> float:
    return float(row[key].strip())


def compact(row: dict[str, str]) -> str:
    parts = [f"epoch {int(as_float(row, 'epoch'))}"]
    for key in METRICS:
        parts.append(f"{key.split('/')[-1]}={as_float(row, key):.5f}")
    return ", ".join(parts)


def main() -> None:
    path = Path(root_path(parse_args().results))
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    if not rows:
        raise SystemExit(f"No rows found in {path}")

    best_map50 = max(rows, key=lambda row: as_float(row, "metrics/mAP50(B)"))
    best_map5095 = max(rows, key=lambda row: as_float(row, "metrics/mAP50-95(B)"))
    final = rows[-1]

    print(f"Loaded {len(rows)} epochs from {path}")
    print(f"Best mAP50:    {compact(best_map50)}")
    print(f"Best mAP50-95: {compact(best_map5095)}")
    print(f"Final epoch:   {compact(final)}")


if __name__ == "__main__":
    main()
