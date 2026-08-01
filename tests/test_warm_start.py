"""Offline tests for OptStudy warm-start injection (devlog 2026-07-30 Q4-9-2).

No hardware: ``OptStudy.__init__`` only creates directories, and the
injection path works on an in-memory Optuna study.
Run with ``pytest tests/test_warm_start.py -v``.
"""

import csv

import optuna
import pytest

from a1z.analysis.optimize.search_space import build_optuna_space
from a1z.analysis.optimize.study import OptStudy

FIELDS = ["trial_id", "theta_kp", "theta_zeta", "cost_new", "watchdog_ok"]


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _fresh_study():
    return optuna.create_study(
        sampler=optuna.samplers.GPSampler(seed=42),
        direction="minimize",
    )


def test_inject_ok_violation_and_unusable_rows(tmp_path):
    csv_path = tmp_path / "relabel.csv"
    _write_csv(csv_path, [
        {"trial_id": 0, "theta_kp": 30.0, "theta_zeta": 0.8,
         "cost_new": 1.5, "watchdog_ok": "1"},
        {"trial_id": 1, "theta_kp": 40.0, "theta_zeta": 0.9,
         "cost_new": 1.2, "watchdog_ok": "1"},
        # watchdog violation -> constraint channel + surrogate objective value
        {"trial_id": 2, "theta_kp": 90.0, "theta_zeta": 0.4,
         "cost_new": "", "watchdog_ok": "0"},
        # unusable row (recompute failed) but theta known -> same treatment
        {"trial_id": 3, "theta_kp": 50.0, "theta_zeta": 0.7,
         "cost_new": "", "watchdog_ok": "1"},
    ])
    opt = OptStudy(session_dir=tmp_path / "sess", joint1=2,
                   warm_start_path=csv_path)
    study = _fresh_study()
    space = build_optuna_space(1, phase="A")  # J2: non-degenerate, zeta space

    n = opt._inject_warm_start(study, space)

    assert n == 4
    surrogate = 1.5  # max(feasible) — v7 violation surrogate (Q11)
    vals = sorted(t.value for t in study.trials)
    assert vals == pytest.approx([1.2, 1.5, surrogate, surrogate])
    # feasibility travels via the constraint channel
    by_kp = {t.params["kp"]: t for t in study.trials}
    assert by_kp[30.0].user_attrs["wd_ok"] == 1
    assert by_kp[30.0].system_attrs["constraints"] == (0.0,)
    assert by_kp[90.0].user_attrs["wd_ok"] == 0
    assert by_kp[90.0].system_attrs["constraints"] == (1.0,)
    # best_trial must see the injected historic optimum
    assert study.best_trial.value == pytest.approx(1.2)
    assert study.best_trial.params == {"kp": 40.0, "zeta_hat": 0.9}


def test_rows_without_theta_are_skipped(tmp_path):
    csv_path = tmp_path / "relabel.csv"
    _write_csv(csv_path, [
        {"trial_id": 0, "theta_kp": "", "theta_zeta": "",
         "cost_new": "", "watchdog_ok": "0"},
        {"trial_id": 1, "theta_kp": 30.0, "theta_zeta": 0.8,
         "cost_new": 2.0, "watchdog_ok": "1"},
    ])
    opt = OptStudy(session_dir=tmp_path / "sess", joint1=6,
                   warm_start_path=csv_path)
    study = _fresh_study()
    space = build_optuna_space(5, phase="A")

    assert opt._inject_warm_start(study, space) == 1
    assert len(study.trials) == 1


def test_injected_params_land_inside_search_space(tmp_path):
    """create_trial with out-of-space params would raise; real relabel CSVs
    come from the same space so this must pass for every row."""
    csv_path = tmp_path / "relabel.csv"
    _write_csv(csv_path, [
        {"trial_id": i, "theta_kp": kp, "theta_zeta": z,
         "cost_new": 1.0, "watchdog_ok": "1"}
        for i, (kp, z) in enumerate([(12.5, 0.4), (25.0, 0.8), (100.0, 1.2)])
    ])
    opt = OptStudy(session_dir=tmp_path / "sess", joint1=6,
                   warm_start_path=csv_path)
    study = _fresh_study()
    space = build_optuna_space(5, phase="A")
    assert opt._inject_warm_start(study, space) == 3


def test_injected_count_persists_for_resume_accounting(tmp_path):
    """Q15 regression: the injected count must land in study.json so a
    resumed run can subtract it from len(study.trials) when computing the
    remaining ONLINE trial budget."""
    import json

    csv_path = tmp_path / "relabel.csv"
    _write_csv(csv_path, [
        {"trial_id": i, "theta_kp": 30.0 + i, "theta_zeta": 0.8,
         "cost_new": 1.0 + 0.1 * i, "watchdog_ok": "1"}
        for i in range(5)
    ])
    sess = tmp_path / "sess"
    opt = OptStudy(session_dir=sess, joint1=6, warm_start_path=csv_path)
    study = _fresh_study()
    space = build_optuna_space(5, phase="A")
    opt._n_injected = opt._inject_warm_start(study, space)
    opt._write_study_json(study)

    doc = json.loads((sess / "study.json").read_text())
    assert doc["warm_start_injected"] == 5
    # resume-side accounting: online budget = total - injected
    n_online_done = len(study.trials) - doc["warm_start_injected"]
    assert n_online_done == 0


def test_inject_into_degenerate_joint_kd_space(tmp_path):
    """Q16: J5 (kd-degenerate) injection must emit (kp, kd) params matching
    the direct-kd space, with kd = the clamped value actually applied."""
    csv_path = tmp_path / "relabel.csv"
    _write_csv(csv_path, [
        {"trial_id": 0, "theta_kp": 30.0, "theta_zeta": 0.8,
         "cost_new": 1.2, "watchdog_ok": "1"},
        {"trial_id": 1, "theta_kp": 40.0, "theta_zeta": 1.2,
         "cost_new": "", "watchdog_ok": "0"},
    ])
    opt = OptStudy(session_dir=tmp_path / "sess", joint1=5,
                   warm_start_path=csv_path)
    study = _fresh_study()
    space = build_optuna_space(4, phase="A")   # J5 -> degenerate -> kd space
    assert "kd" in space and "zeta_hat" not in space

    n = opt._inject_warm_start(study, space)
    assert n == 2
    for t in study.trials:
        assert set(t.params.keys()) == {"kp", "kd"}
        # historic kd always clamped to KD_MIN(J5)=0.3 (Q16 measurement)
        assert t.params["kd"] == pytest.approx(0.3)
    by_kp = {t.params["kp"]: t for t in study.trials}
    assert by_kp[40.0].system_attrs["constraints"] == (1.0,)
    assert by_kp[40.0].value == pytest.approx(1.2)  # surrogate = max(feasible)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
