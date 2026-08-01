"""Optuna ask/tell thin wrapper + session lifecycle (SOP-11 §6.5, §10).

``OptStudy`` owns the Optuna ``Study``, anchor scheduling, trial saving, console
logging, and the final ``best_gains.json`` output.  The BO sampler config and
acquisition logic live entirely inside Optuna's ``GPSampler`` — this module
does NOT wrap GP/acquisition internals.
"""
from __future__ import annotations

import json
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from a1z.analysis.optimize.cost_spec import (
    DEFAULT_KD,
    DEFAULT_KP,
    I_HAT,
    KD_MIN,
    PENALTY_COST,
    cost_spec_snapshot,
    compute_joint_cost,
    violation_surrogate,
    COST_SPEC_VERSION,
)
from a1z.analysis.optimize.eval_runner import (
    OptimizeRunner,
    RobotHardFaultError,
    TrialResult,
)
from a1z.analysis.optimize.gains_io import save_gains
from a1z.analysis.optimize.search_space import (
    build_optuna_space,
    theta_to_gains_6,
    to_kd,
)
from a1z.analysis.optimize.watchdog import (
    AnchorMonitor,
    TickWatchdog,
    WatchdogViolation,
    make_tick_watchdog,
)
from a1z.analysis.report import write_json


DEG = 180.0 / np.pi


def _wd_constraints(trial) -> List[float]:
    """Feasibility constraint for GPSampler (devlog 2026-07-31 Q11).

    Convention (Optuna): constraint value <= 0 means feasible.  Watchdog
    verdicts travel via ``trial.user_attrs["wd_ok"]`` (set online before
    ``tell``; injected warm-start trials carry the equivalent
    ``system_attrs["constraints"]`` directly).  Keeping violations out of
    the objective values is what preserves the GP's landscape resolution.
    """
    return [0.0 if trial.user_attrs.get("wd_ok", 1) == 1 else 1.0]


def _git_rev() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


class OptStudy:
    """Bayesian optimisation session for one joint (Phase A) or joint group (Phase B)."""

    def __init__(
        self,
        session_dir: Path,
        joint1: int,
        *,
        n_trials: int = 40,
        phase: str = "A",
        vel_ff: bool = True,
        can_channel: str = "can0",
        warm_start_path: Optional[Path] = None,
        integral_level: str = "K0",
        integral_joints: Optional[list] = None,
        integral_overrides: Optional[dict] = None,
        watchdog_calib_path: Optional[Path] = None,
    ):
        self.session_dir = Path(session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.trials_dir = self.session_dir / "trials"
        self.trials_dir.mkdir(exist_ok=True)

        self.joint1 = joint1
        self.j = joint1 - 1
        self.n_trials = n_trials
        self.phase = phase
        self.vel_ff = vel_ff
        self.can_channel = can_channel
        self.warm_start_path = warm_start_path
        self.integral_level = integral_level
        self.integral_joints = integral_joints
        self.integral_overrides = integral_overrides
        self.watchdog_calib_path = watchdog_calib_path

        self._db_path = self.session_dir / "optuna.db"
        self._study_json_path = self.session_dir / "study.json"
        self._best_gains_path = self.session_dir / "best_gains.json"

        # Feasible (non-penalty) costs observed so far — injected + online.
        # Basis of the violation surrogate (devlog 2026-07-31 Q11).
        self._feasible_costs: List[float] = []
        # Number of warm-start-injected trials (persisted in study.json so
        # resume can subtract them from the online-trial budget, Q15).
        self._n_injected = 0

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------
    def run(self) -> Dict:
        """Run the full BO loop. Returns summary dict."""
        import optuna

        study_name = f"phaseA-J{self.joint1}"
        storage = f"sqlite:///{self._db_path}"

        space = build_optuna_space(self.j, phase=self.phase)

        # Create or load study
        try:
            study = optuna.load_study(study_name=study_name, storage=storage)
            n_done = len(study.trials)
            print(f"[opt] Resuming study '{study_name}' with {n_done} completed trials")
        except KeyError:
            sampler = optuna.samplers.GPSampler(seed=42,
                                                constraints_func=_wd_constraints)
            study = optuna.create_study(
                study_name=study_name,
                storage=storage,
                sampler=sampler,
                direction="minimize",
            )
            n_done = 0
            print(f"[opt] Created new study '{study_name}'")

        # Cost-spec version guard (SOP-11 §10.2): never resume a study built
        # under a different cost spec — old cost values would poison the new
        # GP surface (devlog 2026-07-30 Q4-9-3).  Use a fresh session with
        # --warm-start instead.
        old_spec = {}
        if self._study_json_path.exists():
            try:
                old_spec = json.loads(self._study_json_path.read_text())
            except Exception:
                old_spec = {}
        if n_done > 0 and old_spec:
            old_ver = (old_spec.get("cost_spec") or {}).get("version")
            if old_ver is not None and old_ver != COST_SPEC_VERSION:
                raise SystemExit(
                    f"[opt] cost_spec version mismatch: session study was built "
                    f"with {old_ver}, current code is {COST_SPEC_VERSION}. "
                    f"Resuming would mix cost surfaces (devlog 2026-07-30 "
                    f"Q4-9-3). Start a NEW session with --warm-start "
                    f"<relabel.csv> instead.")

        # Injected warm-start trials must NOT count against the online
        # n_trials budget — including on resume (devlog 2026-07-31 Q15:
        # previously len(study.trials) was used directly, so resuming a
        # warm-started session saw remaining = n_trials - (injected+online)
        # and ran zero online trials).
        self._n_injected = int(old_spec.get("warm_start_injected") or 0)

        # Warm-start: inject re-labelled historic trials into a FRESH study
        # (devlog 2026-07-30 Q4-9-2 Step 2).  Injected trials live only in
        # optuna.db; they do not count against n_trials (those are ONLINE
        # trials) and do not create trials/t*/ directories.
        if self.warm_start_path is not None:
            if n_done > 0:
                print("[opt] warm-start ignored: study already has trials "
                      "(injection only allowed on a fresh study)")
            else:
                n_inj = self._inject_warm_start(study, space)
                self._n_injected = n_inj
                print(f"[opt] warm-start: injected {n_inj} re-labelled trials "
                      f"from {self.warm_start_path}")
        if n_done > 0:
            # Resume / post-injection: rebuild the feasible-cost basis from
            # existing trials so the violation surrogate stays on scale.
            self._feasible_costs = [
                float(t.value) for t in study.trials
                if t.value is not None and np.isfinite(t.value)
                and t.value < PENALTY_COST
                and t.user_attrs.get("wd_ok", 1) == 1
            ]

        # Snapshot study.json
        self._write_study_json(study)

        # Anchor monitor
        anchor = AnchorMonitor(self.session_dir)

        # Tick watchdog for the active joint (load B1 calibration if provided)
        tick_wd = make_tick_watchdog(self.j, self.watchdog_calib_path)

        best_cost = float("inf")
        best_trial = -1
        results: List[Dict] = []
        aborted = False
        abort_reason = ""

        with OptimizeRunner(
            self.can_channel,
            vel_ff=self.vel_ff,
            integral_level=self.integral_level,
            integral_joints=self.integral_joints,
            integral_overrides=self.integral_overrides,
        ) as runner:
            n_online_done = len(study.trials) - self._n_injected
            remaining = self.n_trials - n_online_done
            for i in range(remaining):
                trial_idx = n_online_done + i

                # Anchor check
                if anchor.due(trial_idx):
                    self._run_anchor(runner, self.joint1, anchor)

                # Ask
                trial = study.ask(space)
                theta = dict(trial.params)

                # Eval
                try:
                    result = runner.eval_trial(theta, self.joint1, tick_watchdog=tick_wd)
                except RobotHardFaultError as e:
                    print(f"[opt] FATAL hardware fault at trial {trial_idx}: {e}")
                    trial.set_user_attr("wd_ok", 0)
                    study.tell(trial, self._violation_value())
                    self._save_trial(trial_idx, theta, TrialResult(
                        cost=PENALTY_COST, breakdown={},
                        watchdog_verdict=f"fatal:{e}"))
                    aborted = True
                    abort_reason = str(e)
                    break
                except Exception as e:
                    print(f"[opt] trial {trial_idx} exception: {e}")
                    trial.set_user_attr("wd_ok", 0)
                    study.tell(trial, self._violation_value())
                    self._save_trial(trial_idx, theta, TrialResult(
                        cost=PENALTY_COST, breakdown={},
                        watchdog_verdict=f"exception:{e}"))
                    continue

                # Tell — feasibility goes to the constraint channel
                # (user_attrs["wd_ok"] → constraints_func), the objective
                # value of a violated trial is only a scale-keeping
                # surrogate (devlog 2026-07-31 Q11).
                wd_ok = 0 if result.watchdog_verdict.startswith("violated") else 1
                trial.set_user_attr("wd_ok", wd_ok)
                if wd_ok == 0 or result.cost >= PENALTY_COST:
                    trial.set_user_attr("wd_ok", 0)
                    study.tell(trial, self._violation_value())
                else:
                    study.tell(trial, result.cost)
                    self._feasible_costs.append(result.cost)

                # Track best
                if result.cost < best_cost:
                    best_cost = result.cost
                    best_trial = trial_idx

                # Console line (SOP-11 §11.1)
                kp_j, kd_j = theta_to_gains_6(
                    self.j, theta["kp"], theta.get("zeta_hat"),
                    kd_j=theta.get("kd"))
                theta_desc = (f"zeta={theta['zeta_hat']:.2f}" if "zeta_hat" in theta
                              else f"kd={theta['kd']:.3f}")
                temp_str = (f"temp_mos={int(np.max(result.temp_mos))}C"
                            if result.temp_mos is not None else "temp=?")
                drift_str = (f"drift={'+' if anchor.drift_flag else ''}"
                             f"{'FLAGGED' if anchor.drift_flag else 'ok'}")
                bd = result.breakdown
                print(
                    f"[t{trial_idx:03d}/{self.n_trials:03d}] J{self.joint1}  "
                    f"kp={theta['kp']:.1f} {theta_desc} -> "
                    f"cost={result.cost:.3f} "
                    f"(lag={bd.get('lag', 0):.2f} ts={bd.get('ts', 0):.2f} "
                    f"resid={bd.get('resid', 0):.2f} ess={bd.get('ess', 0):.2f} "
                    f"os={bd.get('overshoot', 0):.2f})\n"
                    f"           best={best_cost:.3f}@t{best_trial:03d}  "
                    f"{temp_str}  wd={result.watchdog_verdict}  {drift_str}"
                )

                # Save trial data
                self._save_trial(trial_idx, theta, result)
                results.append({"trial": trial_idx, "cost": result.cost,
                                "theta": theta, "verdict": result.watchdog_verdict})

                # Temperature pause
                if "temp_pause" in result.watchdog_verdict:
                    print("[opt] Temperature pause — waiting 300s for cooldown")
                    time.sleep(300)

        # Finalize
        best = study.best_trial
        best_theta = dict(best.params)
        kp_best, kd_best = theta_to_gains_6(
            self.j, best_theta["kp"], best_theta.get("zeta_hat"),
            kd_j=best_theta.get("kd"))
        save_gains(self._best_gains_path, kp_best, kd_best,
                   source=f"opt-session-{datetime.now():%Y%m%d}-phaseA-J{self.joint1}")

        summary = {
            "joint": self.joint1,
            "n_trials": self.n_trials,
            "n_completed": len(study.trials),
            "n_warm_start_injected": self._n_injected,
            "aborted": aborted,
            "abort_reason": abort_reason,
            "best_cost": best.value,
            "best_theta": best_theta,
            "best_kp": kp_best.tolist(),
            "best_kd": kd_best.tolist(),
            "anchor_drift": anchor.drift_flag,
        }
        write_json(self.session_dir / "summary.json", summary)
        theta_desc = (f"zeta={best_theta['zeta_hat']:.3f}" if "zeta_hat" in best_theta
                      else f"kd={best_theta['kd']:.3f}")
        if aborted:
            print(f"\n[opt] ABORTED J{self.joint1} after fatal fault: {abort_reason}")
        else:
            print(f"\n[opt] Done J{self.joint1}: best cost={best.value:.4f} "
                  f"at kp={best_theta['kp']:.1f} {theta_desc}")
        return summary

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _violation_value(self) -> float:
        """Objective surrogate for a watchdog-violated trial (v7, Q11).

        Feasibility itself travels via the constraint channel; this value
        only keeps the objective GP on scale (worst observed feasible).
        """
        return violation_surrogate(self._feasible_costs)

    def _inject_warm_start(self, study, space) -> int:
        """Inject re-labelled historic trials via ``study.add_trial()``.

        Reads the output CSV of ``recompute_cost_v4.py`` (columns
        ``theta_kp, theta_zeta, cost_new, watchdog_ok``).  Feasibility is
        injected through the constraint channel: every row gets
        ``user_attrs["wd_ok"]`` plus the pre-computed
        ``system_attrs["constraints"]`` (``add_trial`` bypasses the
        post-tell hook that would compute them).  Violated/unusable rows
        get ``violation_surrogate`` as their objective value so the
        objective GP's fit stays on scale (devlog 2026-07-31 Q11).
        Returns the number injected.
        """
        import csv

        import optuna

        rows = []
        with open(self.warm_start_path, newline="") as f:
            for row in csv.DictReader(f):
                if not row.get("theta_kp") or not row.get("theta_zeta"):
                    continue
                rows.append(row)

        # First pass: feasible costs define the surrogate scale.
        for row in rows:
            if row.get("watchdog_ok", "1") == "1":
                try:
                    self._feasible_costs.append(float(row["cost_new"]))
                except (TypeError, ValueError):
                    pass
        surrogate = self._violation_value()

        n = 0
        for row in rows:
            watchdog_ok = row.get("watchdog_ok", "1") == "1"
            try:
                cost = float(row["cost_new"]) if watchdog_ok else surrogate
            except (TypeError, ValueError):
                cost = surrogate
            kp = float(row["theta_kp"])
            if "kd" in space:
                # Degenerate joint searches kd directly (Q16): inject the
                # kd that was actually applied at the time (zeta_hat -> kd
                # with the same KD_MIN clamp the online path used).
                params = {"kp": kp,
                          "kd": to_kd(kp, float(row["theta_zeta"]),
                                      I_HAT[self.j], kd_min=KD_MIN[self.j])}
            else:
                params = {"kp": kp, "zeta_hat": float(row["theta_zeta"])}
            study.add_trial(
                optuna.trial.create_trial(
                    params=params,
                    distributions=space,
                    value=float(cost),
                    state=optuna.trial.TrialState.COMPLETE,
                    user_attrs={"wd_ok": 1 if watchdog_ok else 0},
                    system_attrs={"constraints": (0.0 if watchdog_ok else 1.0,)},
                )
            )
            n += 1
        return n

    def _write_study_json(self, study) -> None:
        doc = {
            "study_name": study.study_name,
            "cost_spec": cost_spec_snapshot(),
            "sdk_commit": _git_rev(),
            "phase": self.phase,
            "joint": self.joint1,
            "n_trials_target": self.n_trials,
            "vel_ff": self.vel_ff,
            "integral_level": self.integral_level,
            "watchdog_calib_path": str(self.watchdog_calib_path)
            if self.watchdog_calib_path else None,
            "warm_start_path": str(self.warm_start_path)
            if self.warm_start_path else None,
            "warm_start_injected": self._n_injected,
            "created_at": datetime.now().isoformat(),
        }
        write_json(self._study_json_path, doc)

    def _save_trial(self, idx: int, theta: Dict, result: TrialResult) -> None:
        td = self.trials_dir / f"t{idx:03d}"
        td.mkdir(exist_ok=True)
        kp_applied, kd_applied = theta_to_gains_6(
            self.j, theta["kp"], theta.get("zeta_hat"), kd_j=theta.get("kd"))
        meta = {
            "trial_id": idx,
            "cost_spec_version": COST_SPEC_VERSION,
            "theta": theta,
            "gains_applied": {
                "kp": kp_applied.tolist(),
                "kd": kd_applied.tolist(),
            },
            "cost": result.cost,
            "cost_breakdown": result.breakdown,
            "watchdog_verdict": result.watchdog_verdict,
            "tick_wd_reason": result.tick_wd_reason,
            "duration_s": result.duration_s,
            "temp_mos": result.temp_mos.tolist() if result.temp_mos is not None else None,
            "temp_rotor": result.temp_rotor.tolist() if result.temp_rotor is not None else None,
        }
        write_json(td / "meta.json", meta)

        # Save raw CSV if available
        from a1z.analysis.report import write_unit_csv
        for wave_name in ("triangle", "square"):
            wave_data = result.raw.get(wave_name)
            if wave_data is not None and "t" in wave_data and len(wave_data["t"]) > 0:
                j = self.j
                write_unit_csv(
                    td / f"unit-J{self.joint1}-{wave_name}.csv",
                    wave_data["t"],
                    wave_data["ref"][:, j],
                    wave_data["resp"][:, j],
                    eff=wave_data["eff"][:, j],
                )

    def _run_anchor(self, runner: OptimizeRunner, joint1: int,
                    anchor: AnchorMonitor) -> None:
        """Run one anchor-config evaluation for drift detection."""
        print(f"[opt] anchor check (default gains)")
        # Pass kd directly — valid for both parameterisations (Q16).
        default_theta = {
            "kp": float(DEFAULT_KP[self.j]),
            "kd": float(DEFAULT_KD[self.j]),
        }
        result = runner.eval_trial(default_theta, joint1)
        temp_mos = result.temp_mos
        temp_rotor = result.temp_rotor
        anchor.record(result.cost, temp_mos, temp_rotor)
        print(f"[opt] anchor cost={result.cost:.4f}  drift={'FLAGGED' if anchor.drift_flag else 'ok'}")
