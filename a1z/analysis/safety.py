"""Safety windows, preconditions, and limit checks (pure numpy, no hardware).

Single source of truth for:

* per-joint absolute excitation windows and required pre-postures
  (``SAFE_RANGES_DEG`` / ``PRECONDITIONS_DEG``, measured on the DK1 leader
  2026-07-16, reused here for the follower unit tests);
* waveform-window clamping (inset by margin, amplitude auto-shrunk to fit);
* the effective joint-limit box the analysis module treats as hard —
  the INTERSECTION of the URDF limits (what IK clips to) and the SDK
  ``get_robot._JOINT_LIMITS`` (what the running arm enforces), minus a buffer.

Keeping the intersection here means both the waveform gate and the IK dry-run
gate (SOP-03 §3.1, §4.4) reject the same out-of-range configs the hardware
would, instead of trusting either limit set alone.
"""
from __future__ import annotations

import math

import numpy as np

DEG = 180.0 / math.pi

# Safe excitation windows per joint, ABSOLUTE canonical angles in deg
# (measured on the DK1 leader, 2026-07-16). Waveforms must stay inside.
SAFE_RANGES_DEG = {
    1: (-40.0, 0.0),
    2: (15.0, 55.0),    # requires J4 pre-positioned at -40
    3: (-40.0, -10.0),
    4: (-40.0, -10.0),
    5: (-40.0, 40.0),   # requires J2=20, J3=-20, J4=0
    6: (-40.0, 40.0),
}

# Posture other joints must take BEFORE exciting a given joint (1-based, deg).
PRECONDITIONS_DEG = {
    2: {4: -40.0},
    5: {2: 20.0, 3: -20.0, 4: 0.0},
}

# SDK-enforced joint limits (rad), mirrored from a1z.robots.get_robot._JOINT_LIMITS.
# Kept as a literal so safety.py stays import-light (no CAN/hardware pulled in),
# with a runtime cross-check in assert_matches_sdk_limits().
_SDK_JOINT_LIMITS = [
    (-2.094, 2.094),
    (0.0, 3.142),
    (-3.142, 0.0),
    (-1.484, 1.484),
    (-1.484, 1.484),
    (-2.007, 2.007),
]

# Default clearance kept away from every hard limit (rad). Matches the
# 0.05 rad margin used by the PD-tuning step tests (SOP-02 §5.1).
LIMIT_BUFFER_RAD = 0.05


def clamp_wave_window(j1: int, *, amp_deg: float, margin_deg: float) -> tuple[float, float]:
    """Return the (lo, hi) waveform bounds in RAD for 1-based joint ``j1``.

    Window is inset by ``margin_deg`` on each side, then the waveform swings
    +/-``amp_deg`` around the inset-window centre, capped so it never exceeds
    the inset window. Raises if the margin eats the whole window.
    """
    lo_w, hi_w = (math.radians(v) for v in sorted(SAFE_RANGES_DEG[j1]))
    m = math.radians(margin_deg)
    lo_i, hi_i = lo_w + m, hi_w - m
    if lo_i >= hi_i:
        raise ValueError(f"margin {margin_deg} deg eats the whole J{j1} window")
    half = min(math.radians(amp_deg), (hi_i - lo_i) / 2.0)
    centre = (lo_i + hi_i) / 2.0
    return centre - half, centre + half


def effective_limits(buffer_rad: float = LIMIT_BUFFER_RAD,
                     urdf_limits: np.ndarray | None = None) -> np.ndarray:
    """Hard (lo, hi) box per joint = intersection(URDF, SDK) shrunk by buffer.

    Args:
        buffer_rad: clearance removed from each side after intersecting.
        urdf_limits: optional (6,2) array of the model's own limits (from
            ``Kinematics``); when given, intersected with the SDK limits so the
            gate matches whichever set is tighter per joint. When None only the
            SDK limits are used.

    Returns:
        (6, 2) array of [lo, hi] in rad.
    """
    sdk = np.asarray(_SDK_JOINT_LIMITS, dtype=float)
    if urdf_limits is not None:
        urdf = np.asarray(urdf_limits, dtype=float)
        lo = np.maximum(sdk[:, 0], urdf[:, 0])
        hi = np.minimum(sdk[:, 1], urdf[:, 1])
    else:
        lo, hi = sdk[:, 0], sdk[:, 1]
    out = np.stack([lo + buffer_rad, hi - buffer_rad], axis=1)
    if np.any(out[:, 0] >= out[:, 1]):
        bad = np.flatnonzero(out[:, 0] >= out[:, 1])
        raise ValueError(f"buffer {buffer_rad} rad collapses limits for joints {bad + 1}")
    return out


def within_limits(q: np.ndarray, limits: np.ndarray) -> np.ndarray:
    """Boolean (6,) mask of joints inside [lo, hi]."""
    q = np.asarray(q, dtype=float)[:6]
    return (q >= limits[:, 0]) & (q <= limits[:, 1])


def assert_wave_windows_within_limits(buffer_rad: float = LIMIT_BUFFER_RAD,
                                      urdf_limits: np.ndarray | None = None) -> None:
    """Fail fast if any safe excitation window pokes past the effective limits.

    Includes the preconditioned joints (they get driven to fixed absolute
    angles too), so a bad precondition can't sneak past the check.
    """
    lim = effective_limits(buffer_rad, urdf_limits)
    problems = []
    for j1, (lo_d, hi_d) in SAFE_RANGES_DEG.items():
        lo, hi = math.radians(min(lo_d, hi_d)), math.radians(max(lo_d, hi_d))
        jlo, jhi = lim[j1 - 1]
        if lo < jlo or hi > jhi:
            problems.append(
                f"J{j1} window [{lo_d:.0f},{hi_d:.0f}]deg exceeds effective "
                f"limit [{jlo * DEG:.1f},{jhi * DEG:.1f}]deg")
    for j1, pre in PRECONDITIONS_DEG.items():
        for pj1, deg in pre.items():
            v = math.radians(deg)
            jlo, jhi = lim[pj1 - 1]
            if v < jlo or v > jhi:
                problems.append(
                    f"precondition J{pj1}={deg:.0f}deg (for J{j1}) exceeds "
                    f"effective limit [{jlo * DEG:.1f},{jhi * DEG:.1f}]deg")
    if problems:
        raise ValueError("safe-window/limit conflict:\n  " + "\n  ".join(problems))


def assert_matches_sdk_limits(sdk_joint_limits) -> None:
    """Cross-check the mirrored SDK limits against the live SDK constant.

    Called by the runner after importing get_robot so a future edit to the SDK
    limits that isn't reflected here fails loudly instead of silently gating on
    stale numbers.
    """
    live = np.asarray(sdk_joint_limits, dtype=float)
    mirror = np.asarray(_SDK_JOINT_LIMITS, dtype=float)
    if live.shape != mirror.shape or not np.allclose(live, mirror, atol=1e-3):
        raise ValueError(
            "safety._SDK_JOINT_LIMITS out of sync with get_robot._JOINT_LIMITS; "
            f"live={live.tolist()} mirror={mirror.tolist()}")


if __name__ == "__main__":
    lim = effective_limits()
    print("effective limits (deg):")
    for i, (lo, hi) in enumerate(lim):
        print(f"  J{i + 1}: [{lo * DEG:+7.2f}, {hi * DEG:+7.2f}]")
    assert_wave_windows_within_limits()
    print("all safe windows + preconditions within effective limits OK")
