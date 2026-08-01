"""Offline tests for merge_gains.py (SOP-11 §15.2.7).

Run with ``pytest tests/test_merge_gains.py -v``.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from a1z.analysis.optimize.cost_spec import DEFAULT_KD, DEFAULT_KP
from a1z.analysis.optimize.gains_io import load_gains, save_gains
from a1z.analysis.optimize.merge_gains import (
    _infer_joint,
    merge_sessions,
)


def _make_session(
    tmp_path: Path,
    name: str,
    joint1: int,
    kp: float,
    kd: float,
    *,
    with_study: bool = True,
    coulomb_ff: np.ndarray | None = None,
    integral: dict | None = None,
) -> Path:
    """Create a fake single-joint Phase-A session directory."""
    session_dir = tmp_path / name
    session_dir.mkdir(parents=True)

    kp_vec = DEFAULT_KP.copy()
    kd_vec = DEFAULT_KD.copy()
    j = joint1 - 1
    kp_vec[j] = kp
    kd_vec[j] = kd

    save_gains(
        session_dir / "best_gains.json",
        kp_vec,
        kd_vec,
        source=f"test-{name}-J{joint1}",
        coulomb_ff=coulomb_ff,
        integral_overrides=integral,
    )

    if with_study:
        (session_dir / "study.json").write_text(
            json.dumps({"joint": joint1, "phase": "A"}), encoding="utf-8"
        )

    return session_dir


# --- inference ---------------------------------------------------------------

def test_infer_joint_from_study(tmp_path: Path):
    session = _make_session(tmp_path, "phaseA-J3", 3, 55.0, 3.0)
    g = load_gains(session / "best_gains.json")
    assert _infer_joint(session, g, DEFAULT_KP, DEFAULT_KD) == 3


def test_infer_joint_from_directory_name(tmp_path: Path):
    session = _make_session(tmp_path, "run-opt-phaseA-J5", 5, 20.0, 0.6,
                            with_study=False)
    g = load_gains(session / "best_gains.json")
    assert _infer_joint(session, g, DEFAULT_KP, DEFAULT_KD) == 5


def test_infer_joint_from_diff(tmp_path: Path):
    session = _make_session(tmp_path, "weird-name", 2, 99.0, 2.5,
                            with_study=False)
    g = load_gains(session / "best_gains.json")
    assert _infer_joint(session, g, DEFAULT_KP, DEFAULT_KD) == 2


def test_infer_joint_fails_when_ambiguous(tmp_path: Path):
    session = tmp_path / "ambiguous"
    session.mkdir()
    save_gains(session / "best_gains.json", DEFAULT_KP, DEFAULT_KD)
    g = load_gains(session / "best_gains.json")
    with pytest.raises(ValueError, match="cannot infer target joint"):
        _infer_joint(session, g, DEFAULT_KP, DEFAULT_KD)


# --- merge -------------------------------------------------------------------

def test_merge_all_joints(tmp_path: Path):
    sessions = [
        _make_session(tmp_path, f"J{i}", i, float(10 * i), float(i) * 0.5)
        for i in range(1, 7)
    ]
    out = tmp_path / "merged.json"
    summary = merge_sessions(sessions, output_path=out)

    expected_kp = DEFAULT_KP.copy()
    expected_kd = DEFAULT_KD.copy()
    for i in range(1, 7):
        expected_kp[i - 1] = 10 * i
        expected_kd[i - 1] = i * 0.5

    assert np.allclose(summary["kp"], expected_kp)
    assert np.allclose(summary["kd"], expected_kd)
    assert summary["missing_joints"] == []

    loaded = load_gains(out)
    assert np.allclose(loaded["kp"], expected_kp)
    assert np.allclose(loaded["kd"], expected_kd)


def test_merge_with_joint_map(tmp_path: Path):
    session = _make_session(tmp_path, "phaseA-J1", 1, 200.0, 3.0)
    out = tmp_path / "mapped.json"
    summary = merge_sessions(
        [session],
        output_path=out,
        joint_map={session: 2},
        strict=False,
    )
    # Mapped to J2, so J2 should take the J1-session's J2 value, which is
    # the default because the session only optimised J1.
    assert summary["joint_sources"] == {"J2": str(session)}
    assert summary["missing_joints"] == [1, 3, 4, 5, 6]


def test_merge_strict_missing_joints(tmp_path: Path):
    session = _make_session(tmp_path, "J1-only", 1, 200.0, 3.0)
    out = tmp_path / "incomplete.json"
    with pytest.raises(ValueError, match="strict merge requires all 6 joints"):
        merge_sessions([session], output_path=out)


def test_merge_duplicate_joint_raises(tmp_path: Path):
    s1 = _make_session(tmp_path, "J1-a", 1, 110.0, 4.0)
    s2 = _make_session(tmp_path, "J1-b", 1, 120.0, 4.5)
    out = tmp_path / "dup.json"
    with pytest.raises(ValueError, match="covered by multiple sessions"):
        merge_sessions([s1, s2], output_path=out)


def test_merge_dry_run_does_not_write(tmp_path: Path):
    sessions = [
        _make_session(tmp_path, f"J{i}", i, float(10 * i), float(i) * 0.5)
        for i in range(1, 7)
    ]
    out = tmp_path / "dry.json"
    summary = merge_sessions(sessions, output_path=out, write=False)
    assert summary["output"] == str(out)
    assert not out.exists()


# --- coulomb / integral ------------------------------------------------------

def test_merge_coulomb_ff_per_joint(tmp_path: Path):
    sessions = []
    for i in range(1, 7):
        coulomb = np.zeros(6)
        coulomb[i - 1] = 0.1 * i
        sessions.append(_make_session(
            tmp_path, f"J{i}", i, float(10 * i), float(i) * 0.5,
            coulomb_ff=coulomb,
        ))
    out = tmp_path / "merged-coulomb.json"
    summary = merge_sessions(sessions, output_path=out)

    expected = np.array([0.1 * i for i in range(1, 7)])
    assert summary["coulomb_ff"] is not None
    assert np.allclose(summary["coulomb_ff"], expected)

    loaded = load_gains(out)
    assert np.allclose(loaded["coulomb_ff"], expected)


def test_merge_integral_consistent(tmp_path: Path):
    integral = {"t_wind_s": 1.5, "clamp_scale": 1.2, "t_leak_s": 0.8}
    sessions = [
        _make_session(tmp_path, f"J{i}", i, float(10 * i), float(i) * 0.5,
                      integral=integral)
        for i in range(1, 7)
    ]
    out = tmp_path / "merged-integral.json"
    summary = merge_sessions(sessions, output_path=out)
    assert summary["integral"] == integral


def test_merge_integral_conflict_raises(tmp_path: Path):
    s1 = _make_session(tmp_path, "J1-a", 1, 110.0, 4.0,
                       integral={"t_wind_s": 1.0})
    s2 = _make_session(tmp_path, "J2-a", 2, 120.0, 4.5,
                       integral={"t_wind_s": 2.0})
    out = tmp_path / "conflict.json"
    with pytest.raises(ValueError, match="integral config conflicts"):
        merge_sessions([s1, s2], output_path=out, strict=False)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
