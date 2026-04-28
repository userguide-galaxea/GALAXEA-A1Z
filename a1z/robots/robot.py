"""Robot protocol interface."""

from abc import abstractmethod
from typing import Any, Dict, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class Robot(Protocol):
    """A generic Robot protocol for the a1z SDK."""

    @abstractmethod
    def num_dofs(self) -> int:
        """Get the number of controllable degrees of freedom."""
        raise NotImplementedError

    def get_joint_pos(self) -> np.ndarray:
        """Get current joint positions (rad)."""
        ...

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        """Get current joint positions and velocities.

        Returns:
            Dict with keys 'pos', 'vel', 'eff' (all np.ndarray).
        """
        ...

    def command_joint_pos(self, joint_pos: np.ndarray) -> None:
        """Command target joint positions (rad) with default PD gains."""
        ...

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Command target joint state (pos, vel, kp, kd)."""
        ...

    @abstractmethod
    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get all available observations (pos, vel, eff, etc.)."""
        raise NotImplementedError

    def get_robot_info(self) -> Dict[str, Any]:
        """Get robot configuration info (kp, kd, joint limits, etc.)."""
        return {}
