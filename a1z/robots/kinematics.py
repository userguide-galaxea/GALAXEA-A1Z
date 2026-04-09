"""Forward/inverse kinematics using Pinocchio."""

from typing import Optional, Tuple

import numpy as np

try:
    import pinocchio
except ImportError:
    raise ImportError("Pinocchio is required. Install with: pip install pin")


class Kinematics:
    """FK/IK using Pinocchio.

    Provides forward kinematics and a simple iterative inverse kinematics
    solver based on the Pinocchio framework.
    """

    def __init__(self, urdf_path: str, end_effector_frame: Optional[str] = None):
        """Initialize kinematics from URDF.

        Args:
            urdf_path: Path to the URDF file.
            end_effector_frame: Name of the end-effector frame for FK/IK.
                                If None, the last frame in the model is used.
        """
        self._model = pinocchio.buildModelFromUrdf(urdf_path)
        self._data = self._model.createData()
        self._end_effector_frame = end_effector_frame

        if end_effector_frame is not None:
            self._frame_id = self._model.getFrameId(end_effector_frame)
        else:
            self._frame_id = self._model.nframes - 1

    def fk(self, q: np.ndarray, frame_name: Optional[str] = None) -> np.ndarray:
        """Compute forward kinematics.

        Args:
            q: Joint configuration, shape (nq,).
            frame_name: Override frame name. Uses default if None.

        Returns:
            4x4 homogeneous transform of the frame in world coordinates.
        """
        pinocchio.forwardKinematics(self._model, self._data, q)
        pinocchio.updateFramePlacements(self._model, self._data)

        if frame_name is not None:
            fid = self._model.getFrameId(frame_name)
        else:
            fid = self._frame_id

        oMf = self._data.oMf[fid]
        T = np.eye(4)
        T[:3, :3] = oMf.rotation
        T[:3, 3] = oMf.translation
        return T

    def ik(
        self,
        target_pose: np.ndarray,
        init_q: Optional[np.ndarray] = None,
        frame_name: Optional[str] = None,
        dt: float = 0.01,
        pos_threshold: float = 1e-4,
        ori_threshold: float = 1e-4,
        damping: float = 1e-6,
        max_iters: int = 200,
    ) -> Tuple[bool, np.ndarray]:
        """Iterative inverse kinematics using damped least-squares.

        Args:
            target_pose: 4x4 target homogeneous transform.
            init_q: Initial joint configuration. If None, uses zero configuration.
            frame_name: Override frame name.
            dt: Integration timestep.
            pos_threshold: Position convergence threshold (m).
            ori_threshold: Orientation convergence threshold (rad).
            damping: Damped least-squares damping factor.
            max_iters: Maximum iterations.

        Returns:
            (converged, q): Whether IK converged and the resulting configuration.
        """
        if frame_name is not None:
            fid = self._model.getFrameId(frame_name)
        else:
            fid = self._frame_id

        q = init_q.copy() if init_q is not None else np.zeros(self._model.nq)

        target_se3 = pinocchio.SE3(target_pose[:3, :3], target_pose[:3, 3])

        for _ in range(max_iters):
            pinocchio.forwardKinematics(self._model, self._data, q)
            pinocchio.updateFramePlacements(self._model, self._data)

            oMf = self._data.oMf[fid]
            err_se3 = pinocchio.log6(oMf.actInv(target_se3))
            err = err_se3.vector

            pos_err = np.linalg.norm(err[:3])
            ori_err = np.linalg.norm(err[3:])

            if pos_err <= pos_threshold and ori_err <= ori_threshold:
                return True, q

            J = pinocchio.computeFrameJacobian(
                self._model, self._data, q, fid, pinocchio.LOCAL
            )
            # Damped least-squares
            JtJ = J.T @ J + damping * np.eye(self._model.nv)
            dq = np.linalg.solve(JtJ, J.T @ err)
            q = pinocchio.integrate(self._model, q, dq * dt)

        return False, q
