"""A1Z robot arm control server.

Binds a Unix socket at /tmp/a1z.sock and dispatches JSON commands to a live
ArmRobot instance.  Start via tools/a1zctl; communicate via the same script.

Protocol
--------
Each connection sends one newline-terminated JSON request and receives one
newline-terminated JSON response:

  Request:  {"cmd": "<name>", "args": {...}}
  Response: {"ok": true,  "data": {...}}
         or {"ok": false, "error": "<message>"}

Commands: status | move | gripper | dance | stop | info
"""

import json
import os
import signal
import socket
import threading
import time

import numpy as np

from a1z.robots.get_robot import get_a1z_robot

SOCKET_PATH = "/tmp/a1z.sock"


def _deg(*angles: float) -> np.ndarray:
    return np.deg2rad(np.array(angles, dtype=np.float64))


PRESETS: dict[str, np.ndarray] = {
    "home":    _deg(  0,  60,  -60,   0,   0,   0),
    "ready":   _deg(  0,  30,  -30,   0,  45,   0),
    "salute":  _deg( 30,  35,  -80,   0,  80,  90),
    "wave_l":  _deg(-80,  60,  -60,   0,  60,  90),
    "wave_r":  _deg( 80,  60,  -60,   0, -60, -90),
    "nod_a":   _deg(  0,  70,  -60,  50,   0,   0),
    "nod_b":   _deg(  0,  70,  -60,   0,   0,   0),
    "shake_a": _deg(  0,  70,  -60,   0,  40,   0),
    "shake_b": _deg(  0,  70,  -60,   0, -40,   0),
    "reach":   _deg(  0,  20,  -30,   0,  60,   0),
    "bow":     _deg(  0, 110, -130,   0,   0,   0),
}

# Each move: list of (pose_key, speed_multiplier, pause_s)
DANCE_MOVES: dict[str, list] = {
    "salute": [
        ("salute", 1.0, 0.8), ("home", 0.8, 0.0),
    ],
    "wave": [
        ("ready",  1.0, 0.0), ("wave_l", 1.5, 0.1),
        ("wave_r", 1.5, 0.1), ("wave_l", 1.5, 0.1),
        ("wave_r", 1.5, 0.1), ("home",   1.0, 0.0),
    ],
    "nod": [
        ("nod_a", 1.2, 0.0), ("nod_b", 1.2, 0.0), ("home", 1.0, 0.0),
    ],
    "shake": [
        ("shake_a", 1.2, 0.0), ("shake_b", 1.2, 0.0),
        ("shake_a", 1.2, 0.0), ("shake_b", 1.2, 0.0),
        ("home",    1.0, 0.0),
    ],
    "reach": [("reach", 0.9, 0.5), ("home", 0.9, 0.0)],
    "bow":   [("home", 0.7, 0.0), ("bow", 0.5, 0.8), ("home", 0.5, 0.0)],
}

DEFAULT_DANCE_ORDER = ["salute", "wave", "nod", "reach", "bow"]


class RobotServer:
    def __init__(self, robot, with_gripper: bool) -> None:
        self._robot = robot
        self._with_gripper = with_gripper
        self._lock = threading.Lock()
        self._shutdown = threading.Event()

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    def _cmd_status(self, _args: dict) -> dict:
        state = self._robot.get_joint_state()
        pos_deg = np.rad2deg(state["pos"]).tolist()
        data: dict = {
            "pos_deg":    [round(v, 2) for v in pos_deg],
            "vel_rad_s":  [round(v, 3) for v in state["vel"].tolist()],
            "torque_nm":  [round(v, 3) for v in state["eff"].tolist()],
        }
        if self._with_gripper:
            gpos = self._robot.get_gripper_pos()
            data["gripper"] = round(gpos, 3) if gpos is not None else None
        return {"ok": True, "data": data}

    def _cmd_move(self, args: dict) -> dict:
        speed = float(args.get("speed", 0.5))
        if "preset" in args:
            name = args["preset"]
            if name not in PRESETS:
                avail = ", ".join(sorted(PRESETS))
                return {"ok": False, "error": f"Unknown preset '{name}'. Available: {avail}"}
            target = PRESETS[name]
        elif "joints" in args:
            joints = args["joints"]
            if len(joints) != 6:
                return {"ok": False, "error": "joints must be a list of 6 values (degrees)"}
            target = np.deg2rad(np.array(joints, dtype=np.float64))
        else:
            return {"ok": False, "error": "move requires 'preset' or 'joints'"}

        self._robot.move_joints(target, speed=speed)
        pos_deg = np.rad2deg(self._robot.get_joint_pos()[:6]).tolist()
        return {"ok": True, "data": {"pos_deg": [round(v, 2) for v in pos_deg]}}

    def _cmd_gripper(self, args: dict) -> dict:
        if not self._with_gripper:
            return {"ok": False, "error": "Server was started without --with-gripper"}
        value = float(args.get("value", 1.0))
        if not 0.0 <= value <= 1.0:
            return {"ok": False, "error": "value must be in [0.0, 1.0]"}
        self._robot.command_gripper(value)
        return {"ok": True, "data": {"gripper": value}}

    def _cmd_dance(self, args: dict) -> dict:
        moves_list = args.get("moves", DEFAULT_DANCE_ORDER)
        speed = float(args.get("speed", 0.6))
        unknown = [m for m in moves_list if m not in DANCE_MOVES]
        if unknown:
            avail = ", ".join(DANCE_MOVES)
            return {"ok": False, "error": f"Unknown moves: {unknown}. Available: {avail}"}

        self._robot.move_joints(PRESETS["home"], speed=speed * 0.7)
        time.sleep(0.4)
        for move_name in moves_list:
            print(f"[a1z] dance: {move_name}")
            for pose_key, spd_mul, pause in DANCE_MOVES[move_name]:
                self._robot.move_joints(PRESETS[pose_key], speed=speed * spd_mul)
                if pause > 0:
                    time.sleep(pause)
            time.sleep(0.2)
        self._robot.move_joints(PRESETS["home"], speed=speed * 0.6)
        return {"ok": True, "data": {"moves": moves_list}}

    def _cmd_stop(self, _args: dict) -> dict:
        self._shutdown.set()
        return {"ok": True, "data": {"message": "Stopping server"}}

    def _cmd_info(self, _args: dict) -> dict:
        return {
            "ok": True,
            "data": {
                "presets": sorted(PRESETS),
                "dance_moves": list(DANCE_MOVES),
                "joint_limits_deg": {
                    "J1": [-120, 120],
                    "J2": [0, 180],
                    "J3": [-180, 0],
                    "J4": [-85, 85],
                    "J5": [-85, 85],
                    "J6": [-115, 115],
                },
            },
        }

    # ------------------------------------------------------------------
    # Connection handling
    # ------------------------------------------------------------------

    _HANDLERS = {
        "status":  _cmd_status,
        "move":    _cmd_move,
        "gripper": _cmd_gripper,
        "dance":   _cmd_dance,
        "stop":    _cmd_stop,
        "info":    _cmd_info,
    }

    def _handle_connection(self, conn: socket.socket) -> None:
        try:
            data = b""
            while b"\n" not in data:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                data += chunk
            req = json.loads(data.split(b"\n", 1)[0].decode())
            cmd = req.get("cmd", "")
            args = req.get("args", {})
            handler = self._HANDLERS.get(cmd)
            if handler is None:
                result = {"ok": False, "error": f"Unknown command '{cmd}'"}
            else:
                with self._lock:
                    result = handler(self, args)
        except Exception as exc:
            result = {"ok": False, "error": str(exc)}
        try:
            conn.sendall((json.dumps(result) + "\n").encode())
        except Exception:
            pass
        finally:
            conn.close()

    def run(self, socket_path: str = SOCKET_PATH) -> None:
        if os.path.exists(socket_path):
            os.unlink(socket_path)

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(socket_path)
        srv.listen(8)
        srv.settimeout(1.0)
        print(f"[a1z] Listening on {socket_path}")

        try:
            while not self._shutdown.is_set():
                try:
                    conn, _ = srv.accept()
                except socket.timeout:
                    continue
                t = threading.Thread(
                    target=self._handle_connection, args=(conn,), daemon=True
                )
                t.start()
        finally:
            srv.close()
            if os.path.exists(socket_path):
                os.unlink(socket_path)


# ------------------------------------------------------------------
# Entry point (called from tools/a1zctl)
# ------------------------------------------------------------------

def serve(
    can_channel: str = "can0",
    with_gripper: bool = False,
    gravity_mode: bool = False,
) -> None:
    """Start the robot server in the foreground."""
    print(f"[a1z] Initialising arm  can={can_channel}  gripper={'yes' if with_gripper else 'no'}")
    robot = get_a1z_robot(
        can_channel=can_channel,
        zero_gravity_mode=gravity_mode,
        with_gripper=with_gripper,
        gravity_comp_factor=1.0,
    )

    server = RobotServer(robot, with_gripper=with_gripper)

    def _sigint(sig, frame):
        print("\n[a1z] Interrupted — stopping...")
        server._shutdown.set()

    signal.signal(signal.SIGINT, _sigint)

    robot.start()
    print("[a1z] Arm ready.  Press Ctrl+C to stop.")

    try:
        server.run()
    finally:
        robot.stop()
        print("[a1z] Arm stopped.")
