"""Configuration-file support for the A1Z SDK.

A central YAML config lets you set default robot parameters (CAN channel,
URDF, gripper, gains, ...) once and share them across examples and tools.

Example ``a1z_config.yaml``::

    can_channel: can0
    control_freq_hz: 250
    gravity_comp_factor: 1.0
    zero_gravity_mode: true
    with_gripper: false
    gripper_max_torque: 2.0
    # urdf_path: a1z/robot_models/a1z/A1Z_G1Z.urdf
    # motor_a_use_new_enable_protocol: false

Usage in a script::

    from a1z.config import load_config, config_to_robot_kwargs

    config = load_config(args.config) if args.config else {}
    kwargs = config_to_robot_kwargs(config)
    kwargs.setdefault("can_channel", args.can)
    robot = get_a1z_robot(**kwargs)

Command-line arguments override config-file values.
"""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml


# Mapping from config-file key -> get_a1z_robot keyword argument.
# Lists under ``default_kp`` / ``default_kd`` are converted to numpy arrays.
_ROBOT_KWARGS = {
    "can_channel",
    "gravity_comp_factor",
    "zero_gravity_mode",
    "control_freq_hz",
    "min_freq_hz",
    "urdf_path",
    "default_kp",
    "default_kd",
    "with_gripper",
    "gripper_max_torque",
    "motor_a_use_new_enable_protocol",
}


def load_config(path: str) -> Dict[str, Any]:
    """Load a YAML config file.

    Args:
        path: Path to the YAML file.

    Returns:
        Dict of config values. Returns an empty dict if the file is empty.

    Raises:
        FileNotFoundError: If the file does not exist.
        yaml.YAMLError: If the file cannot be parsed.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def config_to_robot_kwargs(config: Dict[str, Any]) -> Dict[str, Any]:
    """Convert a loaded config dict into kwargs for :func:`get_a1z_robot`.

    Only recognized keys are forwarded. ``default_kp`` / ``default_kd`` are
    converted from lists to ``np.ndarray``.
    """
    kwargs: Dict[str, Any] = {}
    for key in _ROBOT_KWARGS:
        if key not in config:
            continue
        value = config[key]
        if key in ("default_kp", "default_kd") and isinstance(value, list):
            value = np.array(value, dtype=np.float64)
        kwargs[key] = value
    return kwargs


def add_config_argument(parser, default: Optional[str] = None) -> None:
    """Add a ``--config`` argument to an argparse parser.

    Args:
        parser: ``argparse.ArgumentParser`` or sub-parser.
        default: Optional default config file path.
    """
    parser.add_argument(
        "--config",
        default=default,
        help="Path to a YAML config file (values can be overridden by CLI flags).",
    )
