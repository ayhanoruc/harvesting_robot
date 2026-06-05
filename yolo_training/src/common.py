from __future__ import annotations

from pathlib import Path


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def root_path(value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    if path.exists():
        return path.resolve()
    return project_root() / path


def auto_device(requested: str | None = None) -> str | int:
    if requested not in (None, "", "auto"):
        return requested

    try:
        import torch

        return 0 if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def patch_ultralytics_offline_font_check() -> None:
    """Avoid internet font checks in restricted/offline local runs."""
    try:
        import ultralytics.utils.checks as checks

        checks.check_font = lambda *args, **kwargs: True
    except Exception:
        pass

    try:
        import ultralytics.data.utils as data_utils

        data_utils.check_font = lambda *args, **kwargs: True
    except Exception:
        pass


def model_arg(value: str | Path) -> str:
    path = root_path(value)
    return str(path) if path.exists() else str(value)
