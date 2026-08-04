"""Synthetic self-checks for the EE refine metrics (SOP-11 §2.2, devlog
2026-08-01 E1).

Same rig philosophy as test_metrics_v2.py: trajectories with KNOWN injected
structure (time shift / normal-direction jitter / terminal offset), so the
metric must recover the injected value — a bug cannot cancel out.

  * phase lag: a pure sample shift is recovered in ms (sign included);
  * jitter decoupling: a pure lag contributes ~nothing to the jitter std,
    while injected normal-direction jitter is recovered in mm;
  * terminal error: a constant terminal-window offset is recovered in mm/deg,
    insensitive to large errors earlier in the trajectory;
  * degenerate inputs: static traces return 0 without crashing.

Run with ``pytest tests/test_metrics_ee.py -v`` or directly with
``python tests/test_metrics_ee.py`` (pytest not required — plain asserts).
"""
import math

import numpy as np

from a1z.analysis.metrics import (
    ee_normal_jitter_std,
    ee_phase_lag,
    ee_refine_metrics,
    ee_terminal_error,
)

FS = 100.0
CENTER = (0.30, 0.0, 0.40)
RADIUS = 0.04
PERIOD = 8.0
CYCLES = 2


def _T(p):
    T = np.eye(4)
    T[:3, 3] = p
    return T


def _circle(fs=FS, period=PERIOD, cycles=CYCLES, radius=RADIUS, center=CENTER):
    """xz-plane circle, constant identity orientation. Returns (t, T (N,4,4))."""
    n = int(round(period * cycles * fs)) + 1
    t = np.arange(n) / fs
    ang = 2.0 * math.pi * t / period
    p = np.zeros((n, 3))
    p[:, 0] = center[0] + radius * np.cos(ang)
    p[:, 1] = center[1]
    p[:, 2] = center[2] + radius * np.sin(ang)
    T = np.tile(np.eye(4), (n, 1, 1))
    T[:, :3, 3] = p
    return t, T


def _shifted(T, k):
    """Response shifted right by k samples: resp(t) = ref(t−k) (edge-padded)."""
    out = np.empty_like(T)
    if k >= 0:
        out[:k] = T[0]
        out[k:] = T[: len(T) - k] if k else T
    else:
        out[: len(T) + k] = T[-k:]
        out[len(T) + k:] = T[-1]
    return out


# ---------------------------------------------------------------------------
# ee_phase_lag
# ---------------------------------------------------------------------------
def test_phase_lag_recovers_injected_shift():
    _, T_ref = _circle()
    for k0 in (15, 5):  # 150 ms / 50 ms at 100 Hz
        lag = ee_phase_lag(T_ref, _shifted(T_ref, k0), FS)
        assert abs(lag - k0 * 10.0) <= 10.0, f"k0={k0}: lag={lag:.1f} ms"


def test_phase_lag_negative_when_response_leads():
    _, T_ref = _circle()
    lag = ee_phase_lag(T_ref, _shifted(T_ref, -10), FS)
    assert abs(lag + 100.0) <= 10.0, f"lag={lag:.1f} ms"


def test_phase_lag_static_trace_is_zero():
    T = np.tile(_T(CENTER), (200, 1, 1))
    assert ee_phase_lag(T, T, FS) == 0.0


# ---------------------------------------------------------------------------
# ee_normal_jitter_std (normal = y for the xz-plane circle)
# ---------------------------------------------------------------------------
Y = np.array([0.0, 1.0, 0.0])


def test_jitter_ignores_pure_lag():
    """A pure phase lag must not leak into the jitter term (decoupling)."""
    _, T_ref = _circle()
    std = ee_normal_jitter_std(T_ref, _shifted(T_ref, 15), Y, FS)
    assert std < 1e-3, f"pure lag leaked into jitter: {std:.4f} mm"


def test_jitter_recovers_injected_normal_amplitude():
    """2 mm sine along the normal at 3 Hz → std = A/√2 ≈ 1.414 mm."""
    t, T_ref = _circle()
    a_inj, f_inj = 0.002, 3.0  # integer cycles over 16 s → exact A/√2
    T_resp = _shifted(T_ref, 10)  # 100 ms lag on top — must be aligned away
    T_resp[:, :3, 3] += (a_inj * np.sin(2.0 * math.pi * f_inj * t))[:, None] * Y
    std = ee_normal_jitter_std(T_ref, T_resp, Y, FS)
    expected = a_inj / math.sqrt(2.0) * 1000.0
    assert abs(std - expected) < 0.05, f"std={std:.3f} mm, expected {expected:.3f}"


def test_jitter_ignores_in_plane_error():
    """Jitter along the trajectory plane (x) does not pollute the normal (y)."""
    t, T_ref = _circle()
    T_resp = T_ref.copy()
    T_resp[:, :3, 3] += (0.003 * np.sin(2.0 * math.pi * 3.0 * t))[:, None] * np.array([1.0, 0.0, 0.0])
    std = ee_normal_jitter_std(T_ref, T_resp, Y, FS)
    assert std < 1e-3, f"in-plane error leaked: {std:.4f} mm"


# ---------------------------------------------------------------------------
# ee_terminal_error
# ---------------------------------------------------------------------------
def test_terminal_error_recovers_tail_offset():
    """Last 10 % of the trace carries a constant 5 mm / 2° offset."""
    _, T_ref = _circle()
    T_resp = T_ref.copy()
    n_tail = int(round(len(T_ref) * 0.1))
    # large early error that must NOT pollute the terminal metric
    T_resp[:-n_tail, 0, 3] += 0.020
    T_resp[-n_tail:, 0, 3] += 0.005
    th = math.radians(2.0)
    Rz = np.array([[math.cos(th), -math.sin(th), 0.0],
                   [math.sin(th), math.cos(th), 0.0],
                   [0.0, 0.0, 1.0]])
    T_resp[-n_tail:, :3, :3] = Rz
    out = ee_terminal_error(T_ref, T_resp, tail_frac=0.1)
    assert abs(out["pos_mm"] - 5.0) < 0.1, f"pos_mm={out['pos_mm']:.3f}"
    assert abs(out["ang_deg"] - 2.0) < 0.05, f"ang_deg={out['ang_deg']:.3f}"


def test_terminal_error_identical_trace_is_zero():
    _, T = _circle()
    out = ee_terminal_error(T, T)
    assert out["pos_mm"] == 0.0 and out["ang_deg"] == 0.0


# ---------------------------------------------------------------------------
# wrapper
# ---------------------------------------------------------------------------
def test_refine_metrics_wrapper_keys_and_consistency():
    t, T_ref = _circle()
    T_resp = _shifted(T_ref, 10)
    T_resp[:, :3, 3] += (0.002 * np.sin(2.0 * math.pi * 3.0 * t))[:, None] * Y
    m = ee_refine_metrics(T_ref, T_resp, FS, Y)
    assert set(m) == {"terminal_pos_mm", "terminal_ang_deg",
                      "normal_jitter_std_mm", "phase_lag_ms"}
    assert abs(m["phase_lag_ms"] - 100.0) <= 10.0
    assert abs(m["normal_jitter_std_mm"] - 0.002 / math.sqrt(2.0) * 1000.0) < 0.05


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"{len(fns)} tests passed")
