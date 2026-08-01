#!/usr/bin/env python3
"""Post-process verify.sh runs and emit a metric summary.

Reads result.json files produced by run_test.py and prints/exports the key
metrics needed to evaluate an optimized parameter set against the L0 preset.
Does NOT make pass/fail judgments.

Usage:
    python -m a1z.analysis.script.verify_summary \
        --gains-file best_gains.json \
        --output-json summary.json \
        --output-csv summary.csv \
        --output-text summary.txt \
        --output-format text \
        /path/to/run-01 /path/to/run-02 ...
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


JOINTS = [f"J{i}" for i in range(1, 7)]
WAVES = ["triangle", "square"]


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Summarize verify.sh run directories.")
    ap.add_argument("run_dirs", nargs="+", help="result.json directories")
    ap.add_argument("--gains-file", default=None,
                    help="best_gains.json used for verification "
                         "(optional; absent when runs used SDK defaults "
                         "or --kp/--kd overrides — see each run's meta.json)")
    ap.add_argument("--output-format", default="text",
                    choices=["text", "json", "csv"])
    ap.add_argument("--output-json", default=None,
                    help="Structured JSON summary output path")
    ap.add_argument("--output-csv", default=None,
                    help="CSV summary output path")
    ap.add_argument("--output-text", default=None,
                    help="Plain-text summary output path")
    return ap.parse_args()


def _load_json(path: Path) -> Optional[Dict[str, Any]]:
    if not path.exists():
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except Exception as exc:
        print(f"[verify_summary] warning: failed to read {path}: {exc}",
              file=sys.stderr)
        return None


def _classify_run(result: Dict[str, Any]) -> str:
    """Return 'ee', 'triangle', or 'square' based on result.json contents."""
    if "ee" in result:
        return "ee"
    unit_tests = result.get("unit_tests", {})
    for j in JOINTS:
        if j not in unit_tests:
            continue
        if "triangle" in unit_tests[j]:
            return "triangle"
        if "square" in unit_tests[j]:
            return "square"
    return "unknown"


def _safe_get(d: Dict[str, Any], *keys, default=None):
    for k in keys:
        if not isinstance(d, dict):
            return default
        d = d.get(k, default)
        if d is None:
            return default
    return d


def _extract_joint_metrics(result: Dict[str, Any], wave: str) -> Dict[str, Dict[str, float]]:
    """Extract per-joint metrics for a triangle or square run."""
    out: Dict[str, Dict[str, float]] = {}
    unit_tests = result.get("unit_tests", {})
    for j in JOINTS:
        if j not in unit_tests:
            continue
        wave_data = unit_tests[j].get(wave, {})
        v2 = wave_data.get("v2", {})
        if wave == "triangle":
            out[j] = {
                "lag_at_rate_deg": _safe_get(v2, "lag_at_rate_deg", default=float("nan")),
                "resid_std_deg": _safe_get(v2, "resid_std_deg", default=float("nan")),
            }
        else:  # square
            out[j] = {
                "ts_v2_max_ms": _safe_get(v2, "ts_v2_max_ms", default=float("nan")),
                "ess_max_deg": _safe_get(v2, "ess_max_deg", default=float("nan")),
                "overshoot_max_pct": _safe_get(
                    v2, "overshoot_max_pct",
                    default=_safe_get(wave_data, "overshoot_max_pct", default=float("nan"))
                ),
            }
    return out


def _extract_ee_metrics(result: Dict[str, Any]) -> Dict[str, float]:
    ee = result.get("ee", {})
    return {
        "pos_rmse_mm": _safe_get(ee, "pos_rmse_mm", default=float("nan")),
        "pos_max_mm": _safe_get(ee, "pos_max_mm", default=float("nan")),
        "ang_rmse_deg": _safe_get(ee, "ang_rmse_deg", default=float("nan")),
        "ang_max_deg": _safe_get(ee, "ang_max_deg", default=float("nan")),
        "ik_convergence_rate": _safe_get(ee, "ik_convergence_rate", default=float("nan")),
    }


def _fmt(v: float) -> str:
    if v is None or (isinstance(v, float) and (v != v)):  # NaN
        return "N/A"
    return f"{v:.4f}"


def _build_text_summary(summary: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append("# verify.sh metric summary")
    lines.append("")
    lines.append(f"gains_file: {summary.get('gains_file', 'unknown')}")
    lines.append(f"runs: {len(summary.get('runs', []))}")
    lines.append("")

    # Per-repeat, per-joint table
    lines.append("## Per-joint metrics")
    lines.append("")
    header = (f"{'repeat':>6} {'wave':>10} {'joint':>5} "
              f"{'lag_deg':>10} {'resid_deg':>10} {'ts_ms':>10} "
              f"{'ess_deg':>10} {'overshoot_pct':>14}")
    lines.append(header)
    lines.append("-" * len(header))

    for rep in summary.get("repeats", []):
        rep_idx = rep["repeat"]
        for wave in ("triangle", "square"):
            wave_key = f"{wave}_metrics"
            if wave_key not in rep:
                continue
            first = True
            for j in JOINTS:
                if j not in rep[wave_key]:
                    continue
                m = rep[wave_key][j]
                lag = _fmt(m.get("lag_at_rate_deg", float("nan")))
                resid = _fmt(m.get("resid_std_deg", float("nan")))
                ts = _fmt(m.get("ts_v2_max_ms", float("nan")))
                ess = _fmt(m.get("ess_max_deg", float("nan")))
                over = _fmt(m.get("overshoot_max_pct", float("nan")))
                rep_str = f"r{rep_idx}" if first else ""
                lines.append(
                    f"{rep_str:>6} {wave:>10} {j:>5} "
                    f"{lag:>10} {resid:>10} {ts:>10} "
                    f"{ess:>10} {over:>14}"
                )
                first = False

    # EE table
    lines.append("")
    lines.append("## End-effector tracking metrics")
    lines.append("")
    lines.append(f"{'repeat':>6} {'pos_rmse_mm':>12} {'pos_max_mm':>12} "
                 f"{'ang_rmse_deg':>13} {'ang_max_deg':>12} {'ik_conv':>9}")
    for rep in summary.get("repeats", []):
        if "ee_metrics" not in rep:
            continue
        m = rep["ee_metrics"]
        lines.append(
            f"r{rep['repeat']:>5} "
            f"{_fmt(m.get('pos_rmse_mm', float('nan'))):>12} "
            f"{_fmt(m.get('pos_max_mm', float('nan'))):>12} "
            f"{_fmt(m.get('ang_rmse_deg', float('nan'))):>13} "
            f"{_fmt(m.get('ang_max_deg', float('nan'))):>12} "
            f"{_fmt(m.get('ik_convergence_rate', float('nan'))):>9}"
        )

    # Raw run id list
    lines.append("")
    lines.append("## Run directories")
    lines.append("")
    for run in summary.get("runs", []):
        lines.append(f"- [{run['type']}] {run['dir']}")

    return "\n".join(lines) + "\n"


def _build_csv(summary: Dict[str, Any]) -> str:
    out = io.StringIO()
    writer = csv.writer(out)
    writer.writerow([
        "repeat", "wave", "joint",
        "lag_at_rate_deg", "resid_std_deg",
        "ts_v2_max_ms", "ess_max_deg", "overshoot_max_pct",
    ])
    for rep in summary.get("repeats", []):
        rep_idx = rep["repeat"]
        for wave in ("triangle", "square"):
            wave_key = f"{wave}_metrics"
            if wave_key not in rep:
                continue
            for j in JOINTS:
                if j not in rep[wave_key]:
                    continue
                m = rep[wave_key][j]
                writer.writerow([
                    rep_idx, wave, j,
                    m.get("lag_at_rate_deg", ""),
                    m.get("resid_std_deg", ""),
                    m.get("ts_v2_max_ms", ""),
                    m.get("ess_max_deg", ""),
                    m.get("overshoot_max_pct", ""),
                ])

    # EE rows
    writer.writerow([])
    writer.writerow(["repeat", "pos_rmse_mm", "pos_max_mm",
                     "ang_rmse_deg", "ang_max_deg", "ik_convergence_rate"])
    for rep in summary.get("repeats", []):
        if "ee_metrics" not in rep:
            continue
        m = rep["ee_metrics"]
        writer.writerow([
            rep["repeat"],
            m.get("pos_rmse_mm", ""),
            m.get("pos_max_mm", ""),
            m.get("ang_rmse_deg", ""),
            m.get("ang_max_deg", ""),
            m.get("ik_convergence_rate", ""),
        ])

    return out.getvalue()


def main() -> int:
    args = _parse_args()

    run_records: List[Dict[str, Any]] = []
    repeat_data: Dict[int, Dict[str, Any]] = {}

    for run_dir in args.run_dirs:
        run_path = Path(run_dir)
        result = _load_json(run_path / "result.json")
        if result is None:
            continue
        run_type = _classify_run(result)
        run_records.append({
            "dir": str(run_path),
            "type": run_type,
        })

        # Derive repeat index from run id if possible: prefix-rN-{tri,step,ee}-01
        repeat_idx = 1
        try:
            run_id = run_path.name.replace("-run-", "-").split("-")[-1]
            if run_id.startswith("r"):
                repeat_idx = int(run_id.split("-")[0][1:])
        except Exception:
            pass

        rep = repeat_data.setdefault(repeat_idx, {"repeat": repeat_idx})

        if run_type == "ee":
            rep["ee_metrics"] = _extract_ee_metrics(result)
        elif run_type in ("triangle", "square"):
            rep[f"{run_type}_metrics"] = _extract_joint_metrics(result, run_type)

    summary = {
        "gains_file": (
            str(Path(args.gains_file).resolve()) if args.gains_file
            else "n/a (SDK defaults or --kp/--kd override; see run meta.json)"
        ),
        "runs": run_records,
        "repeats": [repeat_data[k] for k in sorted(repeat_data.keys())],
    }

    # JSON output
    if args.output_json:
        with open(args.output_json, "w") as f:
            json.dump(summary, f, indent=2, default=str)
        print(f"[verify_summary] wrote {args.output_json}")

    # CSV output
    if args.output_csv:
        with open(args.output_csv, "w") as f:
            f.write(_build_csv(summary))
        print(f"[verify_summary] wrote {args.output_csv}")

    # Text output
    text = _build_text_summary(summary)
    if args.output_text:
        with open(args.output_text, "w") as f:
            f.write(text)
        print(f"[verify_summary] wrote {args.output_text}")

    # Console output
    if args.output_format == "text":
        print(text)
    elif args.output_format == "json":
        print(json.dumps(summary, indent=2, default=str))
    elif args.output_format == "csv":
        print(_build_csv(summary))

    return 0


if __name__ == "__main__":
    sys.exit(main())
