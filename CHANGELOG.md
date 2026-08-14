# Changelog

All notable user-visible changes to the A1Z SDK are documented in this file.

This is the first changelog for this repository. It covers the safety and
motion-control port from the internal (GitLab) lineage, landed on the
`feat/gitlab-safety-port` branch (driver layer → CAN pacing → startup probe →
safety state machine → clip policy → default parameters → caller adaptation).
An error-integral / Coulomb friction feedforward stage was ported mid-stream
and later withdrawn before release (see V10): it involves internal algorithm
details and is not part of this version.

## [Unreleased] — Safety & Motion-Control Port

### Behavior Changes (V1–V11)

Old behavior → new behavior. Items marked [breaking] may require caller
updates; see "Compatibility Notes" below.

| # | Change | Before → After |
|---|--------|----------------|
| V1 | `start()` state precondition [breaking] | Callable at any time (a second call could leak/override the live control thread) → only allowed from `STOPPED` with no live control thread and no sticky lifecycle fault; otherwise raises `RuntimeError` |
| V2 | Object not restartable after a hard fault [breaking] | `start()` could be called again at any time → after `HARD_DISABLED` / `HARD_DISABLE_UNCONFIRMED` the object is permanently non-restartable; recreate the `ArmRobot` instance |
| V3 | `stop()` no longer shuts down the CAN bus [breaking] | `stop()` called `bus.shutdown()` → the bus is closed on object destruction (`__del__`) or by the owning caller; a stopped robot can be started again on the same bus |
| V4 | `release()` preconditions [breaking] | Only cleared the estop latch → requires the control loop alive, fresh and complete per-joint feedback, a matching state (`SOFT_ESTOP` / `COMMAND_HOLD` / `FAULT_HOLD`), and (optionally) a matching `expected_fault_code`; returns `False` and holds otherwise |
| V5 | `estop()` / `release()` return values | `None` → `bool` (whether the transition was actually applied) |
| V6 | Fault classification | Any control-loop exception triggered an emergency motor disable → recoverable faults (stale feedback, transient CAN TX errors, dynamics computation errors) hold the last command and resend the cached gravity-only MIT hold frame; after 0.2 s without a successful update they latch `FAULT_HOLD` (full-gain position hold, recoverable via `release()`); only confirmed hard faults (over-temperature, over-speed, motor error codes, non-finite feedback) disable the motors |
| V7 | Joint-limit handling for streaming commands [breaking for teleop-style callers] | Out-of-limits targets were rejected (frame dropped; repeated violations could estop) → targets are **always clipped** to the physical limits after an S¹ (circle-topology) unwrap toward the current command; large violations beyond the per-joint tolerance (0.05 rad default) are logged at ERROR, small overshoots at WARNING. `move_joints` still raises `ValueError` for targets beyond the tolerance, and gains a `max_jump_rad` argument that refuses targets too far from the measured pose |
| V8 | Default PD gains raised | KP `[30, 30, 30, 20, 5, 5]` / KD `[1, 1, 1, 0.5, 0.5, 0.5]` → KP `[146.8988, 62.9454, 89.2416, 120.0, 40.0, 100.0]` / KD `[5.0, 5.0, 5.0, 2.0776, 1.5059, 1.2553]` (hardware-tuned; `a1z.yaml` / `a1z_g1z.yaml` updated in sync; URDF dynamics parameters updated to match) |
| V9 | CAN command pacing on by default | Per-tick MIT frames were sent back-to-back → a 250 µs inter-frame gap is inserted by default (`get_a1z_robot(inter_cmd_gap_us=...)`; `0` disables, max 500 µs, `ValueError` beyond). Fixes feedback/target-latch starvation of the last-commanded (wrist) motor |
| V10 | ~~Optional integral / friction feedforward~~ — withdrawn before release | The error-integral (S2) and Coulomb friction (S1) feedforward were ported mid-stream and then removed: they involve internal algorithm details and are not part of the open-source version. The control law stays PD + gravity compensation |
| V11 | Runtime fault-state query | Not available → `get_fault_status()` returns a thread-safe snapshot: `state` / `code` / `reason` / fault age / per-joint feedback health / `restart_allowed` / disable-transmission confirmation |

### New Capabilities

- **Safety state machine** (`ControlState`): `STOPPED` / `RUNNING` / `SOFT_ESTOP` / `COMMAND_HOLD` / `FAULT_HOLD` / `HARD_DISABLED` / `HARD_DISABLE_UNCONFIRMED`, with a lifecycle-locked `start()` / `stop()` and a startup feedback probe (zero-gain probe, up to 3 attempts, plus motor-error/temperature/velocity admission checks before closed-loop gains are applied).
- **`get_fault_status()`** — programmatic access to the state machine snapshot (V11); the `a1zctl` server status output now includes `control_state` / `fault_code` / `fault_reason`.
- **`hold_last_command()`** — keep the last queued position target and clear motion feedforward without latching a fault (teleop input-loss behavior).
- **`latch_last_command()`** — latch the last transmitted target (`COMMAND_HOLD`) until `release()` succeeds (persistent upstream loss).
- **`hold_position()`** — latch a strict full-gain MIT position hold (`FAULT_HOLD`) while the control loop stays alive.
- **`inter_cmd_gap_us` constructor argument** — CAN command pacing control (V9); also paces the integrated gripper frame.

### Other API Adjustments

- `command_joint_pos()` / `command_joint_state()` now return `bool` (whether the frame was accepted) instead of `None`; they reject silently (logged) while estopped/holding.
- `estop()` / `release()` accept optional `reason` / `fault_code` / `expected_fault_code` arguments for upstream fault-source discrimination.
- Examples (`gravity_comp.py`, `position_hold.py`, `teach_and_play.py`, `dance.py`), `tools/`, and the `a1zctl` server (`a1z/robots/server.py`) were adapted to the new semantics; the server's estop/release handlers now surface `False` results as errors.

### Testing & Validation

- Unit suite: **148 passed + 1 xfailed** (149 collected) across 11 test modules in `tests/`. The single xfail documents an intentional deviation: `MixedMotorChain.disable_all` keeps this repository's bool-return contract instead of the internal lineage's raise-on-failure semantics.
- Suites introduced or reworked during the port: driver layer (`test_inter_cmd_gap.py`, `test_error_semantics.py`, `test_feedback_frame_filter.py`); safety state machine (`test_control_fault_policy.py`, `test_motion_safety.py`, `test_feedback_safety.py` rewritten); clip policy (`test_joint_limit_tolerance.py`, `test_command_clear_semantics.py`); tools (`test_motor_diag_semantics.py`); `test_safe_return.py` adapted to the state-machine fields.
- **Pre-release TODO (not yet done): on-hardware closed-loop regression is a release gate** — zero-gravity float, long position hold (no oscillation / no steady-state drift), `move_joints` round trips, soft estop/release behavior, gripper operations; run against both MotorA firmware protocols, on SocketCAN (required) and gs_usb (smoke), with both YAML configs. A release tag will only be cut after this regression passes.

### Compatibility Notes

- **Teleop / streaming-command users: read V7.** Out-of-limits joint targets are now clipped (arm keeps tracking) instead of rejected or estopped; confirm replayed trajectory shapes (e.g. `teach_and_play.py` sessions). The companion teleop repository (OpenA1Z-T) will be synchronized separately.
- **MotorA legacy firmware:** the dual enable-protocol paths are retained. The default remains the legacy per-motor `0xFC`/`0xFD` frames; the newer `0x7FF` broadcast config frame stays opt-in via `motor_a_use_new_enable_protocol=True`.
- **Cross-platform CAN backends retained:** the `can_backend.py` / `open_can_bus()` abstraction and the `gs_usb` / `pcan` extras are unchanged (Linux SocketCAN, macOS/Windows gs_usb userspace).
- **Default control law unchanged:** the torque sum is exactly `torque_ff + inverse-dynamics compensation` — PD + gravity compensation, as before.
- `is_running` / `is_estopped` remain available as compatibility properties over the state machine.
- Callers that relied on `stop()` closing the CAN bus (V3) must manage the bus lifecycle themselves (e.g. an explicit `bus.shutdown()` when done, as `examples/dance.py` does).
