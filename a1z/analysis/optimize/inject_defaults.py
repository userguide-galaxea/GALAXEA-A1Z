#!/usr/bin/env python3
"""Level 3 — Inject optimised parameters into SDK source as new defaults.

Reads a best_gains.json file (from BO Phase A/B), patches
``a1z/robots/get_robot.py``'s module-level constants in-place:

  * ``_DEFAULT_KP`` / ``_DEFAULT_KD`` — always (Phase A/B)
  * ``_TAU_C_HAT`` — when ``coulomb_ff`` key is present (Phase B)

and ``a1z/robots/integrator.py`` LEVELS dict:

  * ``LEVELS["K1"]`` t_wind default — when ``integral.t_wind_s`` is present

Usage::

    # Dry-run (show diff only):
    python -m a1z.analysis.optimize.inject_defaults best_gains.json --dry-run

    # Apply (overwrites source in-tree):
    python -m a1z.analysis.optimize.inject_defaults best_gains.json

    # After apply — commit:
    git add open-a1z-t/GALAXEA-A1Z/a1z/robots/get_robot.py \
            open-a1z-t/GALAXEA-A1Z/a1z/robots/integrator.py
    git commit -m "feat(robots): update default gains from BO Phase B"

The script is idempotent.  It refuses to touch a file that doesn't match the
expected pattern.
"""

from __future__ import annotations

import argparse
import difflib
import re
import sys
from pathlib import Path

import numpy as np

_SELF_DIR = Path(__file__).resolve().parent
_ROBOTS_DIR = _SELF_DIR.parent.parent / "robots"
_GET_ROBOT_PATH = _ROBOTS_DIR / "get_robot.py"
_INTEGRATOR_PATH = _ROBOTS_DIR / "integrator.py"

_KP_RE = re.compile(
    r"^(_DEFAULT_KP\s*=\s*np\.array\(\[)([^\]]+)(\]\))$", re.MULTILINE
)
_KD_RE = re.compile(
    r"^(_DEFAULT_KD\s*=\s*np\.array\(\[)([^\]]+)(\]\))$", re.MULTILINE
)
_TAU_C_RE = re.compile(
    r"^(_TAU_C_HAT\s*=\s*np\.array\(\[)([^\]]+)(\]\))$", re.MULTILINE
)
_LEVELS_K1_RE = re.compile(
    r'("K1":\s*)([\d.]+)', re.MULTILINE
)


def _fmt_vec(v: np.ndarray) -> str:
    parts = []
    for x in v:
        s = f"{x:.1f}" if x == int(x) else f"{x:.4g}"
        parts.append(s)
    return ", ".join(parts)


def _patch_get_robot(src: str, kp: np.ndarray, kd: np.ndarray,
                     coulomb_ff=None) -> str:
    kp_str = _fmt_vec(kp)
    kd_str = _fmt_vec(kd)

    new, n1 = _KP_RE.subn(rf"\g<1>{kp_str}\3", src)
    if n1 != 1:
        raise RuntimeError("Cannot locate _DEFAULT_KP in get_robot.py")
    new, n2 = _KD_RE.subn(rf"\g<1>{kd_str}\3", new)
    if n2 != 1:
        raise RuntimeError("Cannot locate _DEFAULT_KD in get_robot.py")

    if coulomb_ff is not None:
        cf_str = _fmt_vec(coulomb_ff)
        new, n3 = _TAU_C_RE.subn(rf"\g<1>{cf_str}\3", new)
        if n3 != 1:
            raise RuntimeError("Cannot locate _TAU_C_HAT in get_robot.py")

    return new


def _patch_integrator(src: str, t_wind_s: float) -> str:
    new, n = _LEVELS_K1_RE.subn(rf'\g<1>{t_wind_s}', src, count=1)
    if n != 1:
        raise RuntimeError("Cannot locate LEVELS K1 value in integrator.py")
    return new


def _show_diff(old: str, new: str, path: Path):
    if old != new:
        diff = difflib.unified_diff(
            old.splitlines(keepends=True),
            new.splitlines(keepends=True),
            fromfile=str(path),
            tofile=str(path),
        )
        sys.stdout.writelines(diff)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gains_file", type=Path,
                    help="Path to best_gains.json (v1 schema)")
    ap.add_argument("--target-get-robot", type=Path, default=_GET_ROBOT_PATH,
                    help="Override get_robot.py path")
    ap.add_argument("--target-integrator", type=Path, default=_INTEGRATOR_PATH,
                    help="Override integrator.py path")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print unified diff, don't write")
    ap.add_argument("--commit-msg", action="store_true",
                    help="Print suggested commit message to stdout")
    args = ap.parse_args()

    from a1z.analysis.optimize.gains_io import load_gains
    g = load_gains(args.gains_file)
    kp, kd = g["kp"], g["kd"]
    coulomb_ff = g["coulomb_ff"]
    integral = g["integral"]

    # --- get_robot.py ---
    target_gr = args.target_get_robot.resolve()
    if not target_gr.exists():
        print(f"ERROR: {target_gr} not found", file=sys.stderr)
        sys.exit(1)
    src_gr = target_gr.read_text()
    patched_gr = _patch_get_robot(src_gr, kp, kd, coulomb_ff=coulomb_ff)

    # --- integrator.py (only if integral.t_wind_s present) ---
    target_int = args.target_integrator.resolve()
    src_int = patched_int = None
    if integral and "t_wind_s" in integral:
        if not target_int.exists():
            print(f"WARNING: {target_int} not found, skipping integral patch",
                  file=sys.stderr)
        else:
            src_int = target_int.read_text()
            patched_int = _patch_integrator(src_int, integral["t_wind_s"])

    # --- output ---
    if args.dry_run or args.commit_msg:
        _show_diff(src_gr, patched_gr, target_gr)
        if src_int is not None and patched_int is not None:
            _show_diff(src_int, patched_int, target_int)
        if src_gr == patched_gr and (src_int is None or src_int == patched_int):
            print("(no change — gains already match)")

    if args.commit_msg:
        phase = "Phase B" if coulomb_ff is not None else "Phase A"
        print(f"\nfeat(robots): update default gains from BO {phase} "
              f"[source: {g['source']}]")
        return

    if args.dry_run:
        return

    # --- write ---
    target_gr.write_text(patched_gr)
    print(f"Patched {target_gr}")
    print(f"  kp = [{_fmt_vec(kp)}]")
    print(f"  kd = [{_fmt_vec(kd)}]")
    if coulomb_ff is not None:
        print(f"  coulomb_ff (→ _TAU_C_HAT) = [{_fmt_vec(coulomb_ff)}]")
    if patched_int is not None and src_int != patched_int:
        target_int.write_text(patched_int)
        print(f"Patched {target_int}")
        print(f"  LEVELS['K1'] t_wind = {integral['t_wind_s']}")
    print(f"  source: {g['source']}")


if __name__ == "__main__":
    main()
