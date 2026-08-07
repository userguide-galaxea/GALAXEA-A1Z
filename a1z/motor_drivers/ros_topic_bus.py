"""ROS2 topic transport bus for the lemo main board.

Bridges the SDK to the embedded ``g4spi_node`` (lemo_main_board repo, ``jsc``
branch) over its ROS2 topics; the node owns the SPI polling of the G4 MCU,
so the SDK can control the arm while the node keeps publishing leader-arm /
key / servo-state topics.

The MCU firmware forwards CAN frames one at a time (no 56-byte command
image): every ``send()`` publishes one message and every feedback frame
arrives as its own message. Message layout (``lemo_main_board/msg``)::

    std_msgs/Header header
    uint8 arm_id          # 1 = left arm, 2 = right arm
    uint16 id
    uint8[] data          # variable length, 0..8 (classic CAN payload)

Topic wiring::

    SDK -> node:  <side>_motor_send   (one CAN frame per message)
    node -> SDK:  <side>_motor_data   (one CAN feedback frame per message)

``<side>`` is ``left`` (G4 CMD 0x11) or ``right`` (0x12). The ``arm_id``
field replaces the old can0/can1 bus distinction: the firmware forwards
each frame to the arm bus selected by ``arm_id`` and stamps feedback with
the arm it came from. All CAN IDs — arm motors 1..6, gripper 7 / 0x307,
and the 0x7FF management broadcasts (MotorA enable/disable/set-zero,
gripper mode-4 register write) — travel on the same pair of topics; the
firmware forwards each frame by its ``id``.

``data`` carries exactly the CAN payload, unpadded: a 4-byte MotorA enable
goes out as 4 bytes, so the firmware can forward it with the true DLC.

Requires rclpy and the built ``lemo_main_board`` message package on the
PYTHONPATH (colcon build the jsc branch, source the workspace). Both are
imported lazily so the SDK stays importable without ROS.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Deque, Optional

import can

logger = logging.getLogger(__name__)

MAX_DATA_LEN = 8

# CanFrame.arm_id values, replacing the old can0/can1 bus distinction.
ARM_ID_BY_SIDE = {"left": 1, "right": 2}


def check_payload(data: bytes) -> bytes:
    """Return the CAN payload as-is after the classic-CAN length check."""
    payload = bytes(data)
    if len(payload) > MAX_DATA_LEN:
        raise can.CanOperationError(
            f"g4ros frame payload is {len(payload)} bytes; classic CAN max is 8"
        )
    return payload


def decode_motor_frame(msg) -> can.Message:
    """Single-frame message (header/arm_id/id/data) -> feedback ``can.Message``.

    ``arm_id`` is not checked here; ``RosTopicBus._on_motor_frame`` filters
    frames for the other arm before decoding.
    """
    return can.Message(
        timestamp=time.time(),
        arbitration_id=int(msg.id),
        data=bytes(msg.data),
        is_extended_id=False,
        is_rx=True,
    )


class RosTopicBus:
    """python-can BusABC-compatible facade over the g4spi_node ROS topics.

    ``send()`` publishes each CAN frame immediately as one message on
    ``<side>_motor_send`` with ``arm_id`` set (1 = left, 2 = right).
    ``recv()`` returns feedback decoded from ``<side>_motor_data`` as
    ``can.Message`` objects with ``is_rx=True`` and Linux-side timestamps;
    frames stamped with the other arm's ``arm_id`` are dropped.

    A background executor thread spins the rclpy node, so callbacks run
    independently of the caller's recv() pattern.

    Args:
        arm_side: ``"left"`` or ``"right"`` — selects the topic prefix and
            the ``arm_id`` stamped on outgoing frames.
        ros_node: optional existing rclpy node to attach to (caller owns its
            spinning and lifecycle); when omitted the bus creates and spins
            its own node, calling ``rclpy.init()`` if needed.
    """

    def __init__(self, arm_side: str = "left", ros_node=None) -> None:
        if arm_side not in ("left", "right"):
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side!r}")
        self._side = arm_side
        self._arm_id = ARM_ID_BY_SIDE[arm_side]

        try:
            import rclpy
            from lemo_main_board.msg import CanFrame
        except ImportError as exc:
            raise ImportError(
                "transport='g4ros' requires rclpy and the lemo_main_board message "
                "package (CanFrame: std_msgs/Header header, uint8 arm_id, "
                "uint16 id, uint8[] data): colcon-build the lemo_main_board "
                "repo (jsc branch) and source the workspace before starting "
                "the SDK"
            ) from exc
        self._rclpy = rclpy
        self._frame_cls = CanFrame

        self._owns_node = ros_node is None
        self._owns_context = False
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
                self._owns_context = True
            self._node = rclpy.create_node(f"a1z_topic_bridge_{arm_side}")
        else:
            self._node = ros_node

        self._send_pub = self._node.create_publisher(
            CanFrame, f"{arm_side}_motor_send", 10
        )
        self._node.create_subscription(
            CanFrame, f"{arm_side}_motor_data", self._on_motor_frame, 10
        )

        self._rx_cv = threading.Condition()
        self._rx_queue: Deque[can.Message] = deque()

        self._executor = None
        self._spin_thread = None
        if self._owns_node:
            self._executor = rclpy.executors.SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._executor.spin, name=f"g4ros-spin-{arm_side}", daemon=True
            )
            self._spin_thread.start()

    # -- python-can compatible surface -----------------------------------

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        """Publish one CAN frame immediately as a single-frame message."""
        frame = self._frame_cls()
        frame.header.stamp = self._node.get_clock().now().to_msg()
        frame.arm_id = self._arm_id
        frame.id = int(msg.arbitration_id)
        frame.data = list(check_payload(msg.data))
        try:
            self._send_pub.publish(frame)
        except Exception as exc:  # noqa: BLE001 - surface as CAN-flavour error
            raise can.CanOperationError(f"g4ros frame publish failed: {exc}") from exc

    def recv(self, timeout: Optional[float] = 0.0) -> Optional[can.Message]:
        """Return the next decoded feedback message, or None on timeout."""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._rx_cv:
            while not self._rx_queue:
                if timeout is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._rx_cv.wait(remaining)
                else:
                    self._rx_cv.wait()
            return self._rx_queue.popleft()

    def shutdown(self) -> None:
        """Stop spinning and release ROS resources owned by this bus."""
        with self._rx_cv:
            self._rx_cv.notify_all()
        if self._owns_node:
            if self._executor is not None:
                self._executor.shutdown()
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=1.0)
            self._node.destroy_node()
            if self._owns_context and self._rclpy.ok():
                self._rclpy.shutdown()

    # -- internals ---------------------------------------------------------

    def _on_motor_frame(self, msg) -> None:
        # Frames stamped for the other arm (or by old firmware without the
        # arm_id field) are not this bus's feedback.
        if getattr(msg, "arm_id", self._arm_id) != self._arm_id:
            return
        with self._rx_cv:
            self._rx_queue.append(decode_motor_frame(msg))
            self._rx_cv.notify_all()

    def __enter__(self) -> "RosTopicBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.shutdown()
