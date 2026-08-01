"""P0 offline acceptance for the vel-ff channel (SOP-10 §1.4, all-six-joint).

Three checks, no hardware, gate the接口 before上机:

  1. 导数一致性 — for J1..J6 × triangle × period∈{2,4,8}, ``sample_vel`` equals
     the central difference of ``sample`` on the excited joint at every point
     OUTSIDE the apex blend windows (analytic ↔ numeric互证, <1e-9). This is the
     sign-correctness guard for R1: vel-ff is literally d/dt of the same
     canonical pos the arm already tracks, so the sign can't be inverted.
  2. 上限表 — the analytic peak |sample_vel| over 6 joints × 3 periods is
     ≤0.53 rad/s (§1.3 table), well under the 4.0 rad/s command cap; and
     ``sample_vel`` on a square wave hits the disabled/reject path.
  3. opt-in 不变性 — driving ``_Base.stream`` with vel_fn=None produces the same
     command dict (keys + a zero vel every tick) the pre-vel-ff code did; only
     passing a vel_fn changes the commanded velocity.

Run with ``pytest tests/test_vel_ff.py -v`` or directly
(``python tests/test_vel_ff.py`` — plain asserts, no pytest required).
"""
import math

import numpy as np

from a1z.analysis.signals import WaveTrajectory, _MAX_CMD_VEL_RAD_S

DEG = 180.0 / math.pi
PERIODS = (2.0, 4.0, 8.0)
UPPER_BOUND_RAD_S = 0.53          # §1.3: triangle p2 = 30°/s = 0.524 rad/s


def _transitions(tr: WaveTrajectory):
    """Wave-relative apex/hold-boundary times (k·half, k=0..2·cycles)."""
    half = tr.period / 2.0
    span = tr.cycles * tr.period
    return [k * half for k in range(0, 2 * tr.cycles + 1)], span, half


# ---------------------------------------------------------------------------
# Check 1 — 导数一致性 (analytic vel == central difference of pos)
# ---------------------------------------------------------------------------
def test_sample_vel_matches_central_difference():
    q0 = np.zeros(6)
    dt = 1e-3                                    # exact on linear segments
    worst = 0.0
    for j in range(6):
        for period in PERIODS:
            tr = WaveTrajectory(q0, j, "triangle", period=period)
            trans, span, _half = _transitions(tr)
            # Guard band around each transition: blend window + the stencil so
            # neither central-difference sample straddles an apex/hold edge.
            guard = tr.vel_blend_s + 2 * dt
            for t in np.arange(dt, tr.duration, 0.01):
                tt = t - tr.hold_pre
                if any(abs(tt - tj) < guard for tj in trans):
                    continue                     # inside a blend window — skip
                num = (tr.sample(t + dt)[j] - tr.sample(t - dt)[j]) / (2 * dt)
                ana = tr.sample_vel(t)[j]
                worst = max(worst, abs(num - ana))
                assert abs(num - ana) < 1e-9, (
                    f"J{j+1} p{period} t={t:.3f}: num={num:.9f} ana={ana:.9f}")
    print(f"[check1] max |num−ana| over 6×3 grid = {worst:.2e} rad/s  (<1e-9)")


def test_sample_vel_only_excited_joint_nonzero():
    """The other five joints (held at preset angles) get exactly 0 feedforward."""
    q0 = np.zeros(6)
    tr = WaveTrajectory(q0, 5, "triangle", period=4.0)      # excite J6
    v = tr.sample_vel(tr.hold_pre + 1.0)                    # mid up-swing
    assert v.shape == (6,)
    assert abs(v[5]) > 0.0
    assert np.all(v[:5] == 0.0)


def test_apex_blend_crosses_through_zero():
    """At a sign-flip apex the blended velocity passes through 0 (no kd·Δv slam)."""
    q0 = np.zeros(6)
    tr = WaveTrajectory(q0, 5, "triangle", period=4.0)
    # first apex (at hi) sits at wave-relative period/2 → absolute hold_pre+2.0
    t_apex = tr.hold_pre + tr.period / 2.0
    assert abs(tr.sample_vel(t_apex)[5]) < 1e-9              # exactly zero at apex
    # just before apex still +s, just after −s (blend is monotone through zero)
    before = tr.sample_vel(t_apex - tr.vel_blend_s * 0.5)[5]
    after = tr.sample_vel(t_apex + tr.vel_blend_s * 0.5)[5]
    assert before > 0.0 > after
    # magnitude never exceeds the plateau slope inside the blend
    assert abs(before) <= tr.max_abs_vel + 1e-12


# ---------------------------------------------------------------------------
# Check 2 — 上限表 + square disabled
# ---------------------------------------------------------------------------
def test_vel_upper_bound_table_all_joints():
    q0 = np.zeros(6)
    print("\n[check2] max|sample_vel| (rad/s) — 6 joints × 3 periods")
    print("period |   " + "     ".join(f"J{j+1}" for j in range(6)))
    gmax = 0.0
    for period in PERIODS:
        row = []
        for j in range(6):
            tr = WaveTrajectory(q0, j, "triangle", period=period)
            ts = np.arange(0.0, tr.duration, 0.005)
            mx = max(abs(tr.sample_vel(t)[j]) for t in ts)
            # analytic peak == observed peak (blend only lowers |v|)
            assert mx <= tr.max_abs_vel + 1e-9
            assert mx <= UPPER_BOUND_RAD_S, f"J{j+1} p{period}: {mx:.4f} > {UPPER_BOUND_RAD_S}"
            gmax = max(gmax, mx)
            row.append(f"{mx:.3f}")
        print(f"  {period:4.1f} | " + "  ".join(row))
    print(f"[check2] global max = {gmax:.4f} rad/s  (≤{UPPER_BOUND_RAD_S}; "
          f"{gmax/_MAX_CMD_VEL_RAD_S*100:.1f}% of {_MAX_CMD_VEL_RAD_S} cap)")
    assert gmax <= UPPER_BOUND_RAD_S


def test_square_vel_ff_disabled():
    """square never yields vel-ff — its ~240°/s slew would blow the 4.0 cap."""
    q0 = np.zeros(6)
    sq = WaveTrajectory(q0, 5, "square", period=4.0)
    raised = False
    try:
        sq.sample_vel(sq.hold_pre + 1.0)
    except ValueError:
        raised = True
    assert raised, "square.sample_vel must reject (§1.3 code-level disable)"


# ---------------------------------------------------------------------------
# Check 3 — opt-in 不变性 (vel_fn=None path unchanged)
# ---------------------------------------------------------------------------
class _FakeRobot:
    """Minimal stand-in: records every command dict, no CAN bus opened."""

    is_estopped = False

    def __init__(self):
        self.sent = []

    def command_joint_state(self, js):
        # snapshot: stream reuses one dict object across ticks (mutates in place)
        self.sent.append({k: (v.copy() if isinstance(v, np.ndarray) else v)
                          for k, v in js.items()})

    def get_joint_state(self):
        z = np.zeros(6)
        return {"pos": z.copy(), "eff": z.copy()}


def _make_base():
    # Import here so a missing `can` in a docs-only checkout doesn't break
    # collection of checks 1–2 (runner pulls get_robot → arm_robot → can).
    from a1z.analysis.runner import _Base
    b = _Base("can0", sample_hz=100)             # no start() → no hardware
    b.robot = _FakeRobot()
    b._started = True
    return b


def test_stream_vel_fn_none_is_byte_identical():
    b = _make_base()
    kp = np.full(6, 25.0)
    kd = np.full(6, 4.0)
    tr = WaveTrajectory(np.zeros(6), 5, "triangle", period=4.0)
    b.stream(tr.sample, 0.05, kp, kd)            # None path (default)
    assert b.robot.sent, "stream produced no commands"
    for js in b.robot.sent:
        assert set(js.keys()) == {"pos", "vel", "kp", "kd"}   # 逐键相同
        assert np.array_equal(js["vel"], np.zeros(6))         # vel stays zero
        assert np.array_equal(js["kp"], kp)
        assert np.array_equal(js["kd"], kd)
    print(f"[check3] None path: {len(b.robot.sent)} cmds, all vel==0, keys unchanged")


def test_stream_vel_fn_sets_velocity():
    """A vel_fn is called every tick and its result becomes the commanded vel.

    Uses a sentinel vel_fn (constant known vector) rather than sample_vel: a
    0.05 s window sits in the leading hold where sample_vel is legitimately 0,
    which wouldn't distinguish "wired" from "ignored". The sentinel proves the
    opt-in path actually forwards vel_fn(t) into js['vel']."""
    b = _make_base()
    kp = np.full(6, 25.0)
    kd = np.full(6, 4.0)
    tr = WaveTrajectory(np.zeros(6), 5, "triangle", period=4.0)
    sentinel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.123])

    b.stream(tr.sample, 0.05, kp, kd, vel_fn=lambda _t: sentinel)
    assert b.robot.sent
    for js in b.robot.sent:
        assert set(js.keys()) == {"pos", "vel", "kp", "kd"}   # same keys, opt-in
        assert np.array_equal(js["vel"], sentinel)            # vel_fn(t) forwarded
    print(f"[check3] vel-ff path: {len(b.robot.sent)} cmds, js['vel']=vel_fn(t)")


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {name}: {e}")
    print(f"{len(fns) - failed}/{len(fns)} passed")
    raise SystemExit(1 if failed else 0)
