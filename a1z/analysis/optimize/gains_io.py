"""Load / save / validate the best_gains.json parameter file.

The canonical JSON schema (v1) is::

    {
      "version": "v1",
      "source": "opt-session-20260730-phaseA",
      "gains": { "kp": [6 floats], "kd": [6 floats] },
      "coulomb_ff": [6 floats],              // optional — Phase B
      "integral":  { ... overrides dict ... } // optional — Phase B
    }

Usage
-----
From CLI (``run_test.py --gains-file`` / ``teleop.py --gains-file``)::

    from a1z.analysis.optimize.gains_io import load_gains
    g = load_gains("best_gains.json")
    kp, kd = g["kp"], g["kd"]        # np.ndarray (6,)
    robot_kw = g["robot_kwargs"]      # ready for get_a1z_robot(**robot_kw)

From the optimiser (write after convergence)::

    save_gains("best_gains.json", kp, kd, source="opt-session-...",
               coulomb_ff=cf, integral_overrides=io)
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

_NUM_JOINTS = 6
_SCHEMA_VERSION = "v1"


def load_gains(path: str | Path) -> Dict[str, Any]:
    """Read ``best_gains.json`` and return a normalised dict.

    Returned keys:
      * ``kp``  — ``np.ndarray`` shape (6,)
      * ``kd``  — ``np.ndarray`` shape (6,)
      * ``coulomb_ff`` — ``np.ndarray | None``
      * ``integral``   — ``dict | None`` (overrides for ``IntegralConfig``)
      * ``source``     — provenance string
      * ``robot_kwargs`` — ``dict`` ready for ``get_a1z_robot(**kw)``
    """
    path = Path(path)
    with open(path) as f:
        raw = json.load(f)

    version = raw.get("version", "v1")
    if version != _SCHEMA_VERSION:
        raise ValueError(f"Unsupported gains file version: {version}")

    gains = raw.get("gains", raw)
    kp = _to_vec(gains, "kp")
    kd = _to_vec(gains, "kd")

    coulomb_ff = None
    if "coulomb_ff" in raw:
        coulomb_ff = _to_vec(raw, "coulomb_ff")

    integral = raw.get("integral", None)

    robot_kw: Dict[str, Any] = {
        "default_kp": kp,
        "default_kd": kd,
    }
    if coulomb_ff is not None:
        robot_kw["coulomb_ff"] = coulomb_ff
    if integral:
        robot_kw["integral_overrides"] = integral

    return {
        "kp": kp,
        "kd": kd,
        "coulomb_ff": coulomb_ff,
        "integral": integral,
        "source": raw.get("source", str(path)),
        "robot_kwargs": robot_kw,
    }


def save_gains(
    path: str | Path,
    kp: np.ndarray,
    kd: np.ndarray,
    *,
    source: str = "",
    coulomb_ff: Optional[np.ndarray] = None,
    integral_overrides: Optional[dict] = None,
) -> Path:
    """Write ``best_gains.json`` from arrays.  Returns the resolved path."""
    path = Path(path)
    doc: Dict[str, Any] = {
        "version": _SCHEMA_VERSION,
        "source": source,
        "gains": {
            "kp": _round_list(kp),
            "kd": _round_list(kd),
        },
    }
    if coulomb_ff is not None:
        doc["coulomb_ff"] = _round_list(coulomb_ff)
    if integral_overrides:
        doc["integral"] = integral_overrides
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
        f.write("\n")
    return path


# ------------------------------------------------------------------
# internals
# ------------------------------------------------------------------

def _to_vec(d: dict, key: str) -> np.ndarray:
    v = np.asarray(d[key], dtype=float)
    if v.shape != (_NUM_JOINTS,):
        raise ValueError(f"'{key}' must be length {_NUM_JOINTS}, got {v.shape}")
    return v


def _round_list(a: np.ndarray, decimals: int = 4) -> list:
    return [round(float(x), decimals) for x in a]
