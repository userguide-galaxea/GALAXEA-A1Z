"""Merge per-joint optimisation sessions into a single ``best_gains.json``.

Typical usage (SOP-11 §15.2.7)::

    python -m a1z.analysis.optimize.merge_gains \
        --sessions $TEST_LOG_ROOT/02-a1z/02-para-opt/<date>-run-opt-phaseA-J* \
        --output $TEST_LOG_ROOT/02-a1z/02-para-opt/phaseA-best_gains.json

Each input session is expected to contain a ``best_gains.json`` produced by a
single-joint Phase-A BO run.  The target joint is discovered from
``study.json`` (preferred), the session directory name, or by comparing the
session's gains to the default values.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from a1z.analysis.optimize.cost_spec import DEFAULT_KD, DEFAULT_KP
from a1z.analysis.optimize.gains_io import load_gains, save_gains


_NUM_JOINTS = 6


def _parse_joint_map(text: str) -> Dict[Path, int]:
    """Parse ``path:joint`` pairs from a comma-separated string."""
    mapping: Dict[Path, int] = {}
    for item in text.split(","):
        path_str, joint_str = item.split(":")
        mapping[Path(path_str.strip()).resolve()] = int(joint_str.strip())
    return mapping


def _infer_joint(
    session_dir: Path,
    gains_doc: Dict,
    default_kp: np.ndarray,
    default_kd: np.ndarray,
) -> int:
    """Return 1-based joint index for a single-joint Phase-A session.

    Resolution order:
      1. ``study.json`` ``joint`` field.
      2. Directory name patterns ``J1`` .. ``J6`` or ``joint1`` .. ``joint6``.
      3. Compare the session's 6-vector gains to defaults; accept only if
         exactly one joint differs.
    """
    # 1. study.json
    study_path = session_dir / "study.json"
    if study_path.exists():
        try:
            with open(study_path) as f:
                study = json.load(f)
            joint = study.get("joint")
            if isinstance(joint, int) and 1 <= joint <= _NUM_JOINTS:
                return joint
        except Exception:
            pass

    # 2. directory name
    name = session_dir.name
    for pattern in (r"[Jj](\d+)", r"joint_?(\d+)"):
        m = re.search(pattern, name)
        if m:
            j = int(m.group(1))
            if 1 <= j <= _NUM_JOINTS:
                return j

    # 3. diff against defaults
    kp = np.asarray(gains_doc["kp"], dtype=float)
    kd = np.asarray(gains_doc["kd"], dtype=float)
    if kp.shape != (_NUM_JOINTS,) or kd.shape != (_NUM_JOINTS,):
        raise ValueError(
            f"[{session_dir}] gains vectors must be length {_NUM_JOINTS}, "
            f"got kp{kp.shape} kd{kd.shape}"
        )
    diff = ~(np.isclose(kp, default_kp) & np.isclose(kd, default_kd))
    if diff.sum() == 1:
        return int(np.argmax(diff)) + 1

    raise ValueError(
        f"[{session_dir}] cannot infer target joint: "
        f"study.json missing/invalid, directory name has no J1-J6 marker, "
        f"and {int(diff.sum())} joints differ from defaults (expected exactly 1). "
        f"Use --joint-map to specify it explicitly."
    )


def _extract_session_gains(
    session_dir: Path,
    joint1: int,
) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[dict], str]:
    """Load one session and return the gains for its target joint.

    Returns ``(kp_6, kd_6, coulomb_ff_6_or_none, integral_or_none, source)``.
    The returned vectors are full length-6 arrays where only the target joint
    carries the session's optimised value; this makes downstream merging a
    simple element-wise preference.
    """
    j = joint1 - 1
    path = session_dir / "best_gains.json"
    if not path.exists():
        raise FileNotFoundError(
            f"[{session_dir}] best_gains.json not found")

    g = load_gains(path)
    kp_6 = g["kp"].copy()
    kd_6 = g["kd"].copy()

    # Sanity-check the inferred joint really changed relative to its neighbours.
    # We do not require every other joint to equal the hard default (warm-start
    # sessions may start from a non-default base), but the target joint must be
    # the one we were asked to extract.
    kp_j = kp_6[j]
    kd_j = kd_6[j]

    # Build single-joint contribution vectors.
    kp_out = np.zeros(_NUM_JOINTS)
    kd_out = np.zeros(_NUM_JOINTS)
    kp_out[j] = kp_j
    kd_out[j] = kd_j

    coulomb = g.get("coulomb_ff")
    if coulomb is not None:
        coulomb = coulomb.copy()
        coulomb_out = np.zeros(_NUM_JOINTS)
        coulomb_out[j] = coulomb[j]
        coulomb = coulomb_out

    integral = g.get("integral")
    source = g.get("source", str(session_dir))
    return kp_out, kd_out, coulomb, integral, source


def merge_sessions(
    session_dirs: List[Path],
    *,
    output_path: Path,
    default_kp: Optional[np.ndarray] = None,
    default_kd: Optional[np.ndarray] = None,
    joint_map: Optional[Dict[Path, int]] = None,
    source_note: str = "",
    strict: bool = True,
    write: bool = True,
) -> Dict:
    """Merge several per-joint sessions into one ``best_gains.json``.

    Parameters
    ----------
    session_dirs:
        List of session directories, each containing ``best_gains.json``.
    output_path:
        Where to write the merged ``best_gains.json``.
    default_kp, default_kd:
        Fallback vectors for joints not covered by any session.  Defaults to
        ``cost_spec.DEFAULT_KP/KD``.
    joint_map:
        Optional explicit ``{session_dir: joint1}`` mapping.
    source_note:
        Extra text appended to the generated ``source`` field.
    strict:
        If True, require all six joints to be covered exactly once.

    Returns
    -------
    Summary dict with ``kp``, ``kd``, ``coulomb_ff``, ``integral``,
    ``joint_sources``, ``output``.
    """
    kp_base = (default_kp.copy() if default_kp is not None
               else DEFAULT_KP.copy())
    kd_base = (default_kd.copy() if default_kd is not None
               else DEFAULT_KD.copy())

    if kp_base.shape != (_NUM_JOINTS,) or kd_base.shape != (_NUM_JOINTS,):
        raise ValueError(
            f"default kp/kd must be length {_NUM_JOINTS}, "
            f"got kp{kp_base.shape} kd{kd_base.shape}"
        )

    merged_kp = kp_base.copy()
    merged_kd = kd_base.copy()
    merged_coulomb: Optional[np.ndarray] = None
    merged_integral: Optional[dict] = None
    joint_sources: Dict[int, Path] = {}
    sources: List[str] = []

    for session_dir in session_dirs:
        session_dir = Path(session_dir).resolve()
        if not session_dir.is_dir():
            raise NotADirectoryError(f"not a directory: {session_dir}")

        # Load full file early so inference can diff against defaults if needed.
        g = load_gains(session_dir / "best_gains.json")

        if joint_map and session_dir in joint_map:
            joint1 = joint_map[session_dir]
        else:
            joint1 = _infer_joint(session_dir, g, kp_base, kd_base)

        if joint1 in joint_sources:
            prev = joint_sources[joint1]
            raise ValueError(
                f"joint J{joint1} is covered by multiple sessions: "
                f"{prev} and {session_dir}"
            )

        kp_vec, kd_vec, coulomb, integral, source = _extract_session_gains(
            session_dir, joint1)
        j = joint1 - 1
        merged_kp[j] = kp_vec[j]
        merged_kd[j] = kd_vec[j]
        joint_sources[joint1] = session_dir
        sources.append(f"J{joint1}={source}")

        if coulomb is not None:
            if merged_coulomb is None:
                merged_coulomb = np.zeros(_NUM_JOINTS)
            merged_coulomb[j] = coulomb[j]

        if integral is not None:
            if merged_integral is None:
                merged_integral = dict(integral)
            elif merged_integral != integral:
                # Tolerate floating-point lists by comparing rounded strings.
                def _norm(x):
                    return json.dumps(x, sort_keys=True, default=lambda v: round(v, 6))
                if _norm(merged_integral) != _norm(integral):
                    raise ValueError(
                        f"[{session_dir}] integral config conflicts with previous session"
                    )

    covered = set(joint_sources.keys())
    missing = set(range(1, _NUM_JOINTS + 1)) - covered
    if strict and missing:
        raise ValueError(
            f"strict merge requires all 6 joints; missing J{sorted(missing)}. "
            f"Provide more sessions or drop --strict."
        )

    source = "merged: " + "; ".join(sources)
    if source_note:
        source += f" | {source_note}"

    if write:
        save_gains(
            output_path,
            merged_kp,
            merged_kd,
            source=source,
            coulomb_ff=merged_coulomb,
            integral_overrides=merged_integral,
        )

    return {
        "kp": merged_kp.copy(),
        "kd": merged_kd.copy(),
        "coulomb_ff": (merged_coulomb.copy() if merged_coulomb is not None
                       else None),
        "integral": merged_integral,
        "joint_sources": {f"J{k}": str(v) for k, v in joint_sources.items()},
        "missing_joints": sorted(missing),
        "output": str(output_path),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Merge per-joint optimisation sessions into one best_gains.json",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Merge six Phase-A per-joint sessions
  python -m a1z.analysis.optimize.merge_gains \\
      --sessions 2026-07-30-run-opt-phaseA-J* \\
      --output phaseA-best_gains.json

  # Explicit joint mapping for ambiguous directory names
  python -m a1z.analysis.optimize.merge_gains \\
      --sessions s1 s2 s3 s4 s5 s6 \\
      --joint-map s1:1,s2:2,s3:3,s4:4,s5:5,s6:6 \\
      --output best_gains.json
""")
    parser.add_argument(
        "--sessions", nargs="+", required=True,
        help="Session directories containing best_gains.json (shell glob OK)")
    parser.add_argument(
        "--output", "-o", required=True,
        help="Output best_gains.json path")
    parser.add_argument(
        "--joint-map",
        help="Explicit 'path:joint,path:joint,...' mapping (1-based joint index)")
    parser.add_argument(
        "--default-kp", type=float, nargs=6, metavar="K",
        default=DEFAULT_KP.tolist(),
        help="Fallback kp vector for uncovered joints")
    parser.add_argument(
        "--default-kd", type=float, nargs=6, metavar="K",
        default=DEFAULT_KD.tolist(),
        help="Fallback kd vector for uncovered joints")
    parser.add_argument(
        "--source", default="",
        help="Extra note appended to the output source field")
    parser.add_argument(
        "--no-strict", dest="strict", action="store_false", default=True,
        help="Allow joints to be missing from the merged output")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print summary without writing the output file")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    session_dirs = [Path(s) for s in args.sessions]
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    joint_map: Optional[Dict[Path, int]] = None
    if args.joint_map:
        joint_map = _parse_joint_map(args.joint_map)

    default_kp = np.asarray(args.default_kp, dtype=float)
    default_kd = np.asarray(args.default_kd, dtype=float)

    summary = merge_sessions(
        session_dirs,
        output_path=output_path,
        default_kp=default_kp,
        default_kd=default_kd,
        joint_map=joint_map,
        source_note=args.source,
        strict=args.strict,
        write=not args.dry_run,
    )

    print("Merged gains:")
    print(f"  kp: {summary['kp'].round(4).tolist()}")
    print(f"  kd: {summary['kd'].round(4).tolist()}")
    if summary["coulomb_ff"] is not None:
        print(f"  coulomb_ff: {summary['coulomb_ff'].round(4).tolist()}")
    if summary["integral"]:
        print(f"  integral: {summary['integral']}")
    print("Joint sources:")
    for j, src in summary["joint_sources"].items():
        print(f"  {j}: {src}")
    if summary["missing_joints"]:
        print(f"Missing joints (used defaults): J{summary['missing_joints']}")
    print(f"Output: {summary['output']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
