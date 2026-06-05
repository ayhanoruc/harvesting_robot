from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d


ROOT = Path(__file__).resolve().parent
RUN_DIR = ROOT / "runs" / "detect" / "cotton_v5_yolo11n_512_b4_e80"
PRED_DIR = ROOT / "runs" / "predict" / "cotton_v5_test_predictions_conf" / "labels"
GT_DIR = ROOT / "test" / "labels"
OUT_DIR = ROOT / "ayhan_model_package_2026-05-30" / "ultralytics_plots"
PKG_DIR = ROOT / "ayhan_model_package_2026-05-30"
NAMES = {0: "cotton_boll", 1: "unripe-cotton"}


def read_results_csv(path: Path) -> tuple[list[str], np.ndarray]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    columns = [c.strip() for c in rows[0]]
    values = np.array([[float(v.strip()) for v in row] for row in rows[1:] if row], dtype=float)
    return columns, values


def plot_results(file: Path) -> None:
    columns, data = read_results_csv(file)
    loss_keys, metric_keys = [], []
    for c in columns:
        if "loss" in c:
            loss_keys.append(c)
        elif "metric" in c:
            metric_keys.append(c)

    loss_mid, metric_mid = len(loss_keys) // 2, len(metric_keys) // 2
    plot_columns = loss_keys[:loss_mid] + metric_keys[:metric_mid] + loss_keys[loss_mid:] + metric_keys[metric_mid:]
    fig, ax = plt.subplots(2, len(plot_columns) // 2, figsize=(len(plot_columns) + 2, 6), tight_layout=True)
    ax = ax.ravel()
    x = data[:, 0]
    col_index = {name: i for i, name in enumerate(columns)}
    for i, key in enumerate(plot_columns):
        y = data[:, col_index[key]].astype(float)
        ax[i].plot(x, y, marker=".", label=file.stem, linewidth=2, markersize=8)
        ax[i].plot(x, gaussian_filter1d(y, sigma=3), ":", label="smooth", linewidth=2)
        ax[i].set_title(key, fontsize=12)
    ax[1].legend()
    fig.savefig(OUT_DIR / "results.png", dpi=200)
    plt.close(fig)


def read_yolo(path: Path, is_prediction: bool, conf_threshold: float = 0.0) -> list[dict[str, float]]:
    if not path.exists():
        return []
    objects: list[dict[str, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        parts = line.split()
        cls = int(parts[0])
        vals = [float(v) for v in parts[1:]]
        conf = 1.0
        if is_prediction:
            conf = vals[-1]
            vals = vals[:-1]
        if conf < conf_threshold:
            continue
        if is_prediction or len(vals) == 4:
            cx, cy, w, h = vals[:4]
            x1 = max(0.0, cx - w / 2)
            y1 = max(0.0, cy - h / 2)
            x2 = min(1.0, cx + w / 2)
            y2 = min(1.0, cy + h / 2)
        else:
            xs = vals[0::2]
            ys = vals[1::2]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
        objects.append({"cls": cls, "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": conf})
    return objects


def iou(a: dict[str, float], b: dict[str, float]) -> float:
    ix1 = max(a["x1"], b["x1"])
    iy1 = max(a["y1"], b["y1"])
    ix2 = min(a["x2"], b["x2"])
    iy2 = min(a["y2"], b["y2"])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, a["x2"] - a["x1"]) * max(0.0, a["y2"] - a["y1"])
    area_b = max(0.0, b["x2"] - b["x1"]) * max(0.0, b["y2"] - b["y1"])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def match_image(gts: list[dict[str, float]], preds: list[dict[str, float]], iou_threshold: float = 0.5):
    pairs = []
    for gi, gt in enumerate(gts):
        for pi, pred in enumerate(preds):
            ov = iou(gt, pred)
            if ov >= iou_threshold:
                pairs.append((ov, gi, pi))
    pairs.sort(reverse=True)
    gt_used = [False] * len(gts)
    pred_used = [False] * len(preds)
    matches = []
    for ov, gi, pi in pairs:
        if not gt_used[gi] and not pred_used[pi]:
            gt_used[gi] = True
            pred_used[pi] = True
            matches.append((gi, pi, ov))
    return matches, gt_used, pred_used


def confusion_matrix(conf_threshold: float = 0.25) -> np.ndarray:
    matrix = np.zeros((3, 3), dtype=float)  # rows predicted, columns true, Ultralytics style
    for gt_file in GT_DIR.glob("*.txt"):
        gts = read_yolo(gt_file, False)
        preds = read_yolo(PRED_DIR / gt_file.name, True, conf_threshold)
        matches, gt_used, pred_used = match_image(gts, preds)
        for gi, pi, _ in matches:
            matrix[int(preds[pi]["cls"]), int(gts[gi]["cls"])] += 1
        for i, gt in enumerate(gts):
            if not gt_used[i]:
                matrix[2, int(gt["cls"])] += 1
        for i, pred in enumerate(preds):
            if not pred_used[i]:
                matrix[int(pred["cls"]), 2] += 1
    return matrix


def plot_confusion(matrix: np.ndarray, normalize: bool) -> None:
    array = matrix / ((matrix.sum(0).reshape(1, -1) + 1e-9) if normalize else 1)
    array[array < 0.005] = np.nan

    fig, ax = plt.subplots(1, 1, figsize=(12, 9))
    names = list(NAMES.values())
    nc = len(names) + 1
    ticklabels = [*names, "background"]
    xy_ticks = np.arange(len(ticklabels))
    tick_fontsize = max(6, 15 - 0.1 * nc)
    label_fontsize = max(6, 12 - 0.1 * nc)
    title_fontsize = max(6, 12 - 0.1 * nc)
    btm = max(0.1, 0.25 - 0.001 * nc)
    im = ax.imshow(array, cmap="Blues", vmin=0.0, interpolation="none")
    ax.xaxis.set_label_position("bottom")
    color_threshold = 0.45 * (1 if normalize else np.nanmax(array))
    for i, _row in enumerate(array[:nc]):
        for j, _val in enumerate(_row[:nc]):
            val = array[i, j]
            if np.isnan(val):
                continue
            ax.text(
                j,
                i,
                f"{val:.2f}" if normalize else f"{int(val)}",
                ha="center",
                va="center",
                fontsize=10,
                color="white" if val > color_threshold else "black",
            )
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.05)
    title = "Confusion Matrix" + " Normalized" * normalize
    ax.set_xlabel("True", fontsize=label_fontsize, labelpad=10)
    ax.set_ylabel("Predicted", fontsize=label_fontsize, labelpad=10)
    ax.set_title(title, fontsize=title_fontsize, pad=20)
    ax.set_xticks(xy_ticks)
    ax.set_yticks(xy_ticks)
    ax.tick_params(axis="x", bottom=True, top=False, labelbottom=True, labeltop=False)
    ax.tick_params(axis="y", left=True, right=False, labelleft=True, labelright=False)
    ax.set_xticklabels(ticklabels, fontsize=tick_fontsize, rotation=90, ha="center")
    ax.set_yticklabels(ticklabels, fontsize=tick_fontsize)
    for s in {"left", "right", "bottom", "top", "outline"}:
        if s != "outline":
            ax.spines[s].set_visible(False)
        cbar.ax.spines[s].set_visible(False)
    fig.subplots_adjust(left=0, right=0.84, top=0.94, bottom=btm)
    plot_fname = OUT_DIR / f"{title.lower().replace(' ', '_')}.png"
    fig.savefig(plot_fname, dpi=250)
    plt.close(fig)


def smooth(y: np.ndarray, f: float = 0.05) -> np.ndarray:
    nf = round(len(y) * f * 2) // 2 + 1
    p = np.ones(nf // 2)
    yp = np.concatenate((p * y[0], y, p * y[-1]), 0)
    return np.convolve(yp, np.ones(nf) / nf, mode="valid")


def per_class_metrics(thresholds: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    precision = np.zeros((len(NAMES), len(thresholds)), dtype=float)
    recall = np.zeros_like(precision)
    f1 = np.zeros_like(precision)
    for ti, threshold in enumerate(thresholds):
        m = confusion_matrix(float(threshold))
        for cls in NAMES:
            tp = m[cls, cls]
            fp = m[cls, :].sum() - tp
            fn = m[:, cls].sum() - tp
            p = tp / (tp + fp + 1e-9)
            r = tp / (tp + fn + 1e-9)
            precision[cls, ti] = p
            recall[cls, ti] = r
            f1[cls, ti] = 2 * p * r / (p + r + 1e-9)
    return precision, recall, f1


def plot_mc_curve(px: np.ndarray, py: np.ndarray, save_path: Path, ylabel: str) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(9, 6), tight_layout=True)
    for i, y in enumerate(py):
        ax.plot(px, y, linewidth=1, label=f"{NAMES[i]}")
    y = smooth(py.mean(0), 0.1)
    ax.plot(px, y, linewidth=3, color="blue", label=f"all classes {y.max():.2f} at {px[y.argmax()]:.3f}")
    ax.set_xlabel("Confidence")
    ax.set_ylabel(ylabel)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    ax.set_title(f"{ylabel}-Confidence Curve")
    fig.savefig(save_path, dpi=250)
    plt.close(fig)


def plot_pr_curve(precision: np.ndarray, recall: np.ndarray) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(9, 6), tight_layout=True)
    recall_grid = np.linspace(0, 1, 101)
    curves = []
    ap = []
    for i in NAMES:
        order = np.argsort(recall[i])
        r = recall[i][order]
        p = precision[i][order]
        unique_r, unique_idx = np.unique(r, return_index=True)
        unique_p = p[unique_idx]
        curve = np.interp(recall_grid, unique_r, unique_p, left=unique_p[0], right=unique_p[-1])
        curves.append(curve)
        ap.append(np.trapz(curve, recall_grid))
        ax.plot(recall_grid, curve, linewidth=1, label=f"{NAMES[i]} {ap[-1]:.3f}")
    mean_curve = np.mean(np.stack(curves, axis=1), axis=1)
    ax.plot(recall_grid, mean_curve, linewidth=3, color="blue", label=f"all classes {np.mean(ap):.3f} mAP@0.5")
    ax.set_xlabel("Recall")
    ax.set_ylabel("Precision")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    ax.set_title("Precision-Recall Curve")
    fig.savefig(OUT_DIR / "PR_curve.png", dpi=250)
    plt.close(fig)


def copy_to_package() -> None:
    for name in [
        "results.png",
        "confusion_matrix.png",
        "confusion_matrix_normalized.png",
        "PR_curve.png",
        "P_curve.png",
        "R_curve.png",
        "F1_curve.png",
    ]:
        (PKG_DIR / name).write_bytes((OUT_DIR / name).read_bytes())


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plot_results(RUN_DIR / "results.csv")
    matrix = confusion_matrix(0.25)
    plot_confusion(matrix, normalize=True)
    plot_confusion(matrix, normalize=False)

    thresholds = np.linspace(0, 1, 101)
    precision, recall, f1 = per_class_metrics(thresholds)
    plot_mc_curve(thresholds, precision, OUT_DIR / "P_curve.png", "Precision")
    plot_mc_curve(thresholds, recall, OUT_DIR / "R_curve.png", "Recall")
    plot_mc_curve(thresholds, f1, OUT_DIR / "F1_curve.png", "F1")
    plot_pr_curve(precision, recall)

    with (OUT_DIR / "confusion_matrix_counts.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["predicted_label", "true_label", "count"])
        labels = [*NAMES.values(), "background"]
        for pred_i, pred_name in enumerate(labels):
            for true_i, true_name in enumerate(labels):
                writer.writerow([pred_name, true_name, int(matrix[pred_i, true_i])])
    copy_to_package()
    for path in sorted(OUT_DIR.glob("*")):
        print(path)


if __name__ == "__main__":
    main()
