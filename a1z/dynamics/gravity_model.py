"""Pinocchio RNEA-based gravity compensation model."""

import os

import numpy as np

try:
    import pinocchio
except ImportError:
    raise ImportError("Pinocchio is required. Install with: pip install pin")


class GravityModel:
    """Compute gravity compensation torques from a URDF using Pinocchio RNEA."""

    def __init__(self, urdf_path: str, mesh_dir: str = ""):
        """Load URDF and build Pinocchio model.

        Args:
            urdf_path: Path to the URDF file.
            mesh_dir: Root directory for mesh resources (optional, not needed
                      for pure dynamics computation).
        """
        urdf_path = os.path.expanduser(urdf_path)
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.nq = self.model.nq
        self.nv = self.model.nv

        assert np.allclose(self.model.gravity.linear, [0, 0, -9.81]), (
            f"Unexpected gravity vector: {self.model.gravity.linear}. "
            "The base_link Z axis should point upward."
        )

    def compute_gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torque tau_g(q).

        Uses RNEA with zero velocity and acceleration, yielding the static
        gravity term g(q).

        Args:
            q: Joint angles (rad), shape (nq,).

        Returns:
            tau_g: Gravity torques (Nm), shape (nv,).
        """
        assert q.shape == (self.nq,), f"Expected q shape ({self.nq},), got {q.shape}"
        zero_v = np.zeros(self.nv)
        tau_g = pinocchio.rnea(self.model, self.data, q, zero_v, zero_v)
        return tau_g.copy()

    def compute_inverse_dynamics(
        self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray
    ) -> np.ndarray:
        """Compute full inverse dynamics torque via RNEA.

        tau = M(q)*qdd + C(q,qd)*qd + g(q)

        When qd=0 and qdd=0 this degenerates to compute_gravity_torque.

        Args:
            q:   Joint positions (rad), shape (nq,).
            qd:  Joint velocities (rad/s), shape (nv,).
            qdd: Joint accelerations (rad/s^2), shape (nv,).

        Returns:
            tau: Inverse dynamics torques (Nm), shape (nv,).
        """
        assert q.shape == (self.nq,), f"Expected q shape ({self.nq},), got {q.shape}"
        assert qd.shape == (self.nv,), f"Expected qd shape ({self.nv},), got {qd.shape}"
        assert qdd.shape == (self.nv,), f"Expected qdd shape ({self.nv},), got {qdd.shape}"
        tau = pinocchio.rnea(self.model, self.data, q, qd, qdd)
        return tau.copy()

    def get_joint_limits(self) -> np.ndarray:
        """Return joint limits array, shape (nq, 2): [[lower, upper], ...]."""
        lower = self.model.lowerPositionLimit
        upper = self.model.upperPositionLimit
        return np.column_stack([lower, upper])

    def set_gravity(self, gravity: np.ndarray) -> None:
        """Set gravity vector (for non-standard mounting orientations)."""
        assert gravity.shape == (3,)
        self.model.gravity = pinocchio.Motion(gravity, np.zeros(3))
