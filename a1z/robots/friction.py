"""CoulombConfig — config object for tanh-smoothed Coulomb friction feedforward.

Mirrors ``integrator.py::IntegralConfig`` structure (SOP-11 §7.2):
per-joint ``tau_c`` + smooth sign bandwidth ``qd_eps`` + enable mask.
The existing bare-array ``coulomb_ff`` path in ``arm_robot.py`` (hard sign)
remains as-is; ``CoulombConfig`` activates the ``tanh(vel/qd_eps)`` path.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Any, Iterable, Optional

import numpy as np


@dataclass
class CoulombConfig:
    """Per-joint Coulomb friction feedforward configuration.

    ``tau_c``
        (n,) Nm; 0 = disabled for that joint.
    ``qd_eps``
        Velocity bandwidth for tanh smoothing (rad/s).
        ``tau_coulomb = tau_c * tanh(v_des / qd_eps)``.
    ``enable_mask``
        (n,) bool; joints with False have tau_c forced to 0.
    """

    tau_c: np.ndarray
    qd_eps: float = 0.05
    enable_mask: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.tau_c = np.asarray(self.tau_c, dtype=float)
        n = self.tau_c.shape[0]
        if self.enable_mask is not None:
            self.enable_mask = np.asarray(self.enable_mask, dtype=bool)
            if self.enable_mask.shape[0] != n:
                raise ValueError("tau_c / enable_mask length mismatch")
            self.tau_c = np.where(self.enable_mask, self.tau_c, 0.0)
        if self.qd_eps <= 0:
            raise ValueError(f"qd_eps must be positive, got {self.qd_eps}")

    @classmethod
    def from_tau_c_hat(
        cls,
        tau_c_hat: np.ndarray,
        *,
        scale: float = 1.0,
        qd_eps: float = 0.05,
        joints: Optional[Iterable[int]] = None,
    ) -> "CoulombConfig":
        """Construct from static Coulomb torque estimates (``_TAU_C_HAT``).

        ``scale``: multiplier (default 1.0 = use the estimate as-is).
        ``joints``: 1-based joint indices to enable (None = all finite).
        """
        tau_c_hat = np.asarray(tau_c_hat, dtype=float)
        n = tau_c_hat.shape[0]
        mask = np.isfinite(tau_c_hat)
        if joints is not None:
            jmask = np.zeros(n, dtype=bool)
            for j1 in joints:
                jmask[j1 - 1] = True
            mask &= jmask
        tau_c = np.where(mask, tau_c_hat * scale, 0.0)
        return cls(tau_c=tau_c, qd_eps=qd_eps, enable_mask=mask)

    def compute_tau(self, v_des: np.ndarray) -> np.ndarray:
        """Return (n,) Coulomb feedforward torque for commanded velocity ``v_des``.

        Uses ``tanh(v_des / qd_eps)`` instead of hard ``sign(e)``, so the
        torque is continuous through zero velocity (SOP-11 §7.2).
        """
        return self.tau_c * np.tanh(np.asarray(v_des, dtype=float) / self.qd_eps)

    def as_info(self) -> Dict[str, Any]:
        """Serialisable dict for meta/get_robot_info recording."""
        return {
            "tau_c": self.tau_c.tolist(),
            "qd_eps": self.qd_eps,
            "enable_mask": (self.tau_c != 0).tolist(),
        }
