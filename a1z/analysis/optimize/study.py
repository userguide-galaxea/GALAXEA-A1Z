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
    build_refine_space,
    phase_b_default_theta,
    refine_default_theta,
    refine_theta_to_gains_6,
    theta_to_gains_6,
    to_kd,
)
from a1z.analysis.optimize.watchdog import (
    AnchorMonitor,
    MultiTickWatchdog,
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
    """Bayesian optimisation session for one joint (Phase A), a joint subset
    (E-segment refine, ``joints1`` given) or joint group (Phase B)."""

    def __init__(
        self,
        session_dir: Path,
        joint1: Optional[int] = None,
        *,
        joints1: Optional[List[int]] = None,
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
        if joint1 is None and not joints1:
            raise ValueError("joint1 or joints1 required")
        self.session_dir = Path(session_dir)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.trials_dir = self.session_dir / "trials"
        self.trials_dir.mkdir(exist_ok=True)

        # E-segment refine mode: a coupled multi-joint subset searched
        # jointly (SOP-11 §6.3).  ``joint1`` keeps the primary joint for
        # legacy logging/paths; ``joints1`` carries the full subset.
        self.joints1 = list(joints1) if joints1 else None
        self.joint1 = int(joint1) if joint1 is not None else self.joints1[0]
        self.j = self.joint1 - 1
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

        if self.joints1:
            study_name = f"phaseE-J{''.join(str(j) for j in self.joints1)}"
            space = build_refine_space(self.joints1)
        elif self.phase == "B":
            study_name = f"phaseB-J{self.joint1}"
            space = build_optuna_space(self.j, phase="B")
        else:
            study_name = f"phaseA-J{self.joint1}"
            space = build_optuna_space(self.j, phase=self.phase)
        storage = f"sqlite:///{self._db_path}"

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

        # Tick watchdogs (load B1 calibration if provided): one per searched
        # joint for the joint legs; sessions with an EE leg (refine, Phase B)
        # get a 6-joint fan-out backstop (SOP-11 §12.1 — vel_abs/pos/eff
        # channels, hf/acc per the v4 active table).
        tick_wd = None
        tick_wds: Dict[int, TickWatchdog] = {}
        ee_wd = None
        if self.joints1:
            tick_wds = {j1 - 1: make_tick_watchdog(j1 - 1, self.watchdog_calib_path)
                        for j1 in self.joints1}
        else:
            tick_wd = make_tick_watchdog(self.j, self.watchdog_calib_path)
        with_ee_leg = bool(self.joints1) or self.phase == "B"
        if with_ee_leg:
            ee_wd = MultiTickWatchdog(
                [make_tick_watchdog(j, self.watchdog_calib_path) for j in range(6)])

        best_cost = float("inf")
        best_tag = "—"
        results: List[Dict] = []
        aborted = False
        abort_reason = ""

        with OptimizeRunner(
            self.can_channel,
            vel_ff=self.vel_ff,
            integral_level=self.integral_level,
            integral_joints=self.integral_joints,
            integral_overrides=self.integral_overrides,
            ee_leg=with_ee_leg,
        ) as runner:
            # Seed (refine / Phase B, fresh studies only): evaluate the
            # default point live and inject as the warm-start trial.  Doubles
            # as the session's first anchor record; aborts if the default
            # config itself cannot run (devlog 2026-08-01「anchor
            # transit_fail 即中止」建议, E4/B2 种子同构).
            seeded = False
            if (self.joints1 or self.phase == "B") \
                    and len(study.trials) == 0 and self._n_injected == 0:
                if self.joints1:
                    theta_seed = refine_default_theta(self.joints1)
                    seed_result = runner.eval_trial_refine(theta_seed, self.joints1)
                else:
                    theta_seed = phase_b_default_theta(self.j)
                    seed_result = runner.eval_trial_phase_b(
                        theta_seed, self.joint1)
                print(f"[opt] seed eval at defaults: {theta_seed}")
                anchor.record(seed_result.cost, seed_result.temp_mos,
                              seed_result.temp_rotor)
                if seed_result.cost >= PENALTY_COST:
                    raise SystemExit(
                        f"[opt] ABORT: default seed eval failed "
                        f"({seed_result.watchdog_verdict}) — if the anchor "
                        f"config cannot run, every trial would fail too.")
                study.add_trial(
                    optuna.trial.create_trial(
                        params=theta_seed,
                        distributions=space,
                        value=float(seed_result.cost),
                        state=optuna.trial.TrialState.COMPLETE,
                        user_attrs={"wd_ok": 1},
                        system_attrs={"constraints": (0.0,)},
                    )
                )
                self._n_injected = 1
                self._feasible_costs.append(seed_result.cost)
                self._write_study_json(study)
                seeded = True
                print(f"[opt] seed cost={seed_result.cost:.4f} injected as "
                      f"warm-start trial")

            # Console best tracking: include injected/resumed feasible
            # trials so the display matches study.best_trial (v11) — the
            # refine seed (frozen defaults) is often the early incumbent.
            _feas_told = [
                t for t in study.trials
                if t.value is not None and np.isfinite(t.value)
                and t.value < PENALTY_COST
                and t.user_attrs.get("wd_ok", 1) == 1
            ]
            if _feas_told:
                best_cost = min(t.value for t in _feas_told)
                best_tag = "inj"

            n_online_done = len(study.trials) - self._n_injected
            remaining = self.n_trials - n_online_done
            for i in range(remaining):
                trial_idx = n_online_done + i

                # Anchor check
                if anchor.due(trial_idx) and not (seeded and trial_idx == 0):
                    self._run_anchor(runner, self.joint1, anchor)

                # Ask
                trial = study.ask(space)
                theta = dict(trial.params)

                # Eval
                try:
                    if self.joints1:
                        result = runner.eval_trial_refine(
                            theta, self.joints1,
                            tick_watchdogs=tick_wds, ee_watchdog=ee_wd)
                    elif self.phase == "B":
                        result = runner.eval_trial_phase_b(
                            theta, self.joint1,
                            tick_watchdog=tick_wd, ee_watchdog=ee_wd)
                    else:
                        result = runner.eval_trial(
                            theta, self.joint1, tick_watchdog=tick_wd)
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

                # Track best (feasible trials only — a PENALTY result must
                # never become the displayed incumbent, v11)
                if (wd_ok == 1 and result.cost < PENALTY_COST
                        and result.cost < best_cost):
                    best_cost = result.cost
                    best_tag = f"t{trial_idx:03d}"

                # Console line (SOP-11 §11.1)
                temp_str = (f"temp_mos={int(np.max(result.temp_mos))}C"
                            if result.temp_mos is not None else "temp=?")
                drift_str = (f"drift={'+' if anchor.drift_flag else ''}"
                             f"{'FLAGGED' if anchor.drift_flag else 'ok'}")
                bd = result.breakdown
                if self.joints1 or self.phase == "B":
                    if self.joints1:
                        theta_desc = " ".join(
                            f"J{j1}[{theta[f'kp{j1}']:.1f}/"
                            + (f"{theta[f'kd{j1}']:.2f}]" if f"kd{j1}" in theta
                               else f"ζ{theta[f'zeta_hat{j1}']:.2f}]")
                            for j1 in self.joints1)
                    else:
                        theta_desc = " ".join(f"{k}={v:.4g}" for k, v in theta.items())
                    bd_str = (f"ee={bd.get('ee', 0):.2f} jt={bd.get('joint', 0):.2f} "
                              f"j_ee={bd.get('j_ee', 0):.2f} j_jt={bd.get('j_joint', 0):.2f}")
                    print(
                        f"[t{trial_idx:03d}/{self.n_trials:03d}] {theta_desc} -> "
                        f"cost={result.cost:.3f} ({bd_str})\n"
                        f"           best={best_cost:.3f}@{best_tag}  "
                        f"{temp_str}  wd={result.watchdog_verdict}  {drift_str}"
                    )
                else:
                    theta_desc = (f"zeta={theta['zeta_hat']:.2f}" if "zeta_hat" in theta
                                  else f"kd={theta['kd']:.3f}")
                    print(
                        f"[t{trial_idx:03d}/{self.n_trials:03d}] J{self.joint1}  "
                        f"kp={theta['kp']:.1f} {theta_desc} -> "
                        f"cost={result.cost:.3f} "
                        f"(lag={bd.get('lag', 0):.2f} ts={bd.get('ts', 0):.2f} "
                        f"resid={bd.get('resid', 0):.2f} ess={bd.get('ess', 0):.2f} "
                        f"os={bd.get('overshoot', 0):.2f})\n"
                        f"           best={best_cost:.3f}@{best_tag}  "
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
        if self.joints1:
            kp_best, kd_best = refine_theta_to_gains_6(best_theta, self.joints1)
        elif self.phase == "B":
            # PD stays frozen; the Phase B result lives in best_theta
            # (coulomb_ff / integral dims) recorded in summary.json.
            kp_best, kd_best = DEFAULT_KP.copy(), DEFAULT_KD.copy()
            # Build coulomb_ff vector: only the optimized joint gets the value
            coulomb_ff_best = np.zeros(6)
            if "coulomb_ff" in best_theta:
                coulomb_ff_best[self.j] = best_theta["coulomb_ff"]
        else:
            kp_best, kd_best = theta_to_gains_6(
                self.j, best_theta["kp"], best_theta.get("zeta_hat"),
                kd_j=best_theta.get("kd"))
            coulomb_ff_best = None
        save_gains(self._best_gains_path, kp_best, kd_best,
                   source=f"opt-session-{datetime.now():%Y%m%d}-{study_name}",
                   coulomb_ff=coulomb_ff_best)

        summary = {
            "joint": self.joint1,
            "joints": self.joints1,
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
        if self.joints1:
            theta_desc = " ".join(
                f"J{j1}[{best_theta[f'kp{j1}']:.1f}/"
                + (f"{best_theta[f'kd{j1}']:.2f}]" if f"kd{j1}" in best_theta
                   else f"ζ{best_theta[f'zeta_hat{j1}']:.2f}]")
                for j1 in self.joints1)
        elif self.phase == "B":
            theta_desc = " ".join(f"{k}={v:.4g}" for k, v in best_theta.items())
        else:
            theta_desc = (f"kp={best_theta['kp']:.1f} "
                          + (f"zeta={best_theta['zeta_hat']:.3f}" if "zeta_hat" in best_theta
                             else f"kd={best_theta['kd']:.3f}"))
        if aborted:
            print(f"\n[opt] ABORTED {study_name} after fatal fault: {abort_reason}")
        else:
            print(f"\n[opt] Done {study_name}: best cost={best.value:.4f} "
                  f"at {theta_desc}")
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
        n_skipped = 0
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
            try:
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
            except ValueError:
                # v10 space change (DEFAULT_KP/KD synced to frozen gains):
                # historic rows can fall outside the new per-joint ranges —
                # skip them instead of aborting the whole injection.
                n_skipped += 1
                continue
            n += 1
        if n_skipped:
            print(f"[opt] warm-start: skipped {n_skipped} rows outside the "
                  f"current search space (v10 range change)")
        return n

    def _write_study_json(self, study) -> None:
        doc = {
            "study_name": study.study_name,
            "cost_spec": cost_spec_snapshot(),
            "sdk_commit": _git_rev(),
            "phase": self.phase,
            "joint": self.joint1,
            "joints": self.joints1,
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
        if self.joints1:
            kp_applied, kd_applied = refine_theta_to_gains_6(theta, self.joints1)
        elif self.phase == "B":
            kp_applied, kd_applied = DEFAULT_KP.copy(), DEFAULT_KD.copy()
        else:
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
        if self.joints1:
            # E-segment refine extras (SOP-11 §9.3 extension, v10).
            meta["joints"] = self.joints1
            meta["ee_metrics"] = result.raw.get("ee_metrics")
            meta["ee_breakdown"] = result.raw.get("ee_breakdown")
            meta["joint_costs"] = result.raw.get("joint_costs")
            meta["joint_metrics"] = result.raw.get("joint_metrics")
        elif self.phase == "B":
            # Phase B extras (v13): per-trial feedforward mechanisms.
            meta["coulomb_ff"] = theta.get("coulomb_ff")
            meta["integral"] = {k: theta[k] for k in
                                ("t_wind_s", "clamp_scale", "t_leak_s")
                                if k in theta}
            meta["ee_metrics"] = result.raw.get("ee_metrics")
            meta["ee_breakdown"] = result.raw.get("ee_breakdown")
            meta["joint_metrics"] = result.raw.get("joint_metrics")
        write_json(td / "meta.json", meta)

        # Save raw CSVs if available
        from a1z.analysis.report import write_ee_pose_csv, write_unit_csv
        joint_legs = result.raw.get("joint_legs")
        if joint_legs:
            for j1, leg in joint_legs.items():
                j = j1 - 1
                for wave_name in ("triangle", "square"):
                    wave_data = leg.get(wave_name)
                    if (wave_data is not None and "t" in wave_data
                            and len(wave_data["t"]) > 0):
                        write_unit_csv(
                            td / f"unit-J{j1}-{wave_name}.csv",
                            wave_data["t"],
                            wave_data["ref"][:, j],
                            wave_data["resp"][:, j],
                            eff=wave_data["eff"][:, j],
                        )
        else:
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
        ee = result.raw.get("ee")
        if ee is not None and len(ee.get("t", [])) > 0:
            write_ee_pose_csv(td / "ee-traj-ref.csv", ee["t"], ee["T_ref"])
            write_ee_pose_csv(td / "ee-traj-response.csv", ee["t"], ee["T_resp"])

    def _run_anchor(self, runner: OptimizeRunner, joint1: int,
                    anchor: AnchorMonitor) -> None:
        """Run one anchor-config evaluation for drift detection."""
        print(f"[opt] anchor check (default gains)")
        if self.joints1:
            result = runner.eval_trial_refine(
                refine_default_theta(self.joints1), self.joints1)
        else:
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
