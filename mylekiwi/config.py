from __future__ import annotations

import os
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_config() -> dict:
    path = Path(os.environ.get("MYLEKIWI_CONFIG", ROOT / "configs/lekiwi.yaml"))
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def project_path(value: str) -> Path:
    path = Path(os.path.expandvars(value)).expanduser()
    return path if path.is_absolute() else (ROOT / path).resolve()
