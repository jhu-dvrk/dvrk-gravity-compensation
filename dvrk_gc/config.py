from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_json(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: str | Path, payload: dict[str, Any]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")


def arm_value(raw: Any, arm_name: str) -> Any:
    if isinstance(raw, dict) and arm_name in raw:
        return raw[arm_name]
    return raw


def deg_list_to_rad(values: list[float]) -> list[float]:
    import math

    return [math.radians(float(v)) for v in values]


def make_range_deg(min_deg: float, max_deg: float, step_deg: float) -> list[float]:
    vals: list[float] = []
    current = float(min_deg)
    max_value = float(max_deg)
    step = float(step_deg)
    if step <= 0.0:
        raise ValueError("step_deg must be > 0")
    while current <= max_value + 1e-9:
        vals.append(current)
        current += step
    return vals
