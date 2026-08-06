"""ROS2 topic transport bus for the lemo main board (plan B).

Bridges the SDK to the embedded ``g4spi_node`` (lemo_main_board repo, ``jsc``
branch) over its ROS2 topics instead of driving spidev directly. Use this
when the ROS node must keep running — e.g. it also publishes leader-arm /
key / servo-state topics — since two processes must not poll the same spidev
concurrently (the direct-SPI transport in ``spi_bus.py`` conflicts with it).

Topic wiring (all in the ``lemo_main_board/msg`` package):

    SDK -> node:  <side>_a1z_send   (A1zFrame: 6x MotorCanFrame, ids 1..6)
                  <side>_claw_send  (Claw: 8-byte claw payload)
    node -> SDK:  <side>_a1z_data   (A1zFrame, raw CAN feedback payloads)
                  <side>_claw       (Claw feedback, CAN ID 7)

``<side>`` is ``left`` (G4 CMD 0x11) or ``right`` (0x12). The node forwards a
command frame to the MCU only after it has BOTH a pending A1zFrame and a
pending Claw (``send_pending`` in main.cpp), so every flush publishes the
A1zFrame plus the most recent claw value; until the first claw command has
been written (with_gripper=True recommended) nothing reaches the arm.

Requires rclpy and the built ``lemo_main_board`` message package on the
PYTHONPATH (colcon build the jsc branch, source the workspace). Both are
imported lazily so the SDK stays importable without ROS.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Deque, List, Optional

import can

from a1z.motor_drivers.command_image import CommandImage

logger = logging.getLogger(__name__)


def decode_a1z_frame(msg) -> List[can.Message]:
    """A1zFrame (6x MotorCanFrame) -> feedback messages on CAN IDs 1..6."""
    stamp = time.time()
    return [
        can.Message(
            timestamp=stamp,
            arbitration_id=int(motor.id),
            data=bytes(motor.data),
            is_extended_id=False,
            is_rx=True,
        )
        for motor in msg.motors
    ]


def decode_claw(msg) -> can.Message:
    """Claw -> feedback message on the gripper's CAN ID 7."""
    return can.Message(
        timestamp=time.time(),
        arbitration_id=7,
        data=bytes(msg.data),
        is_extended_id=False,
        is_rx=True,
    )


class RosTopicBus:
    """python-can BusABC-compatible facade over the g4spi_node ROS topics.

    ``send()`` buffers each CAN payload into the 56-byte command image (see
    ``command_image.CommandImage``); once all six arm-motor slots of a burst
    are written, the image is published as one ``A1zFrame`` plus the cached
    ``Claw``. ``recv()`` returns feedback decoded from the node's
    ``<side>_a1z_data`` / ``<side>_claw`` topics as ``can.Message`` objects
    with ``is_rx=True`` and Linux-side timestamps.

    A background executor thread spins the rclpy node, so callbacks run
    independently of the caller's recv() pattern.

    Args:
        arm_side: ``"left"`` or ``"right"`` — selects the topic prefix.
        ros_node: optional existing rclpy node to attach to (caller owns its
            spinning and lifecycle); when omitted the bus creates and spins
            its own node, calling ``rclpy.init()`` if needed.
    """

    def __init__(self, arm_side: str = "left", ros_node=None) -> None:
        if arm_side not in ("left", "right"):
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side!r}")
        self._side = arm_side

        try:
            import rclpy
            from lemo_main_board.msg import A1zFrame, Claw
        except ImportError as exc:
            raise ImportError(
                "transport='g4ros' requires rclpy and the lemo_main_board message "
                "package: colcon-build the lemo_main_board repo (jsc branch) and "
                "source the workspace before starting the SDK"
            ) from exc
        self._rclpy = rclpy
        self._a1z_frame_cls = A1zFrame
        self._claw_cls = Claw

        self._owns_node = ros_node is None
        self._owns_context = False
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
                self._owns_context = True
            self._node = rclpy.create_node(f"a1z_topic_bridge_{arm_side}")
        else:
            self._node = ros_node

        self._a1z_pub = self._node.create_publisher(A1zFrame, f"{arm_side}_a1z_send", 10)
        self._claw_pub = self._node.create_publisher(Claw, f"{arm_side}_claw_send", 10)
        self._node.create_subscription(A1zFrame, f"{arm_side}_a1z_data", self._on_a1z_frame, 10)
        self._node.create_subscription(Claw, f"{arm_side}_claw", self._on_claw, 10)

        self._lock = threading.Lock()  # guards the command image
        self._rx_cv = threading.Condition()
        self._rx_queue: Deque[can.Message] = deque()
        self._tx = CommandImage()

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
        """Buffer one CAN payload; publish A1zFrame+Claw when the burst is full."""
        with self._lock:
            if self._tx.write(msg.arbitration_id, bytes(msg.data)):
                self._flush_locked()

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

    def _flush_locked(self) -> None:
        """Publish the command image. Caller holds ``self._lock``."""
        stamp = self._node.get_clock().now().to_msg()
        a1z = self._a1z_frame_cls()
        a1z.header.stamp = stamp
        for slot in range(6):
            a1z.motors[slot].id = slot + 1
            a1z.motors[slot].data = list(self._tx.payload[slot * 8 : slot * 8 + 8])
        try:
            self._a1z_pub.publish(a1z)
            if self._tx.claw_seen:
                claw = self._claw_cls()
                claw.header.stamp = stamp
                claw.data = list(self._tx.payload[48:56])
                self._claw_pub.publish(claw)
            else:
                logger.warning(
                    "[g4ros] no claw command written yet; g4spi_node forwards "
                    "only with both A1zFrame and Claw pending — nothing sent. "
                    "Run with with_gripper=True."
                )
        except Exception as exc:  # noqa: BLE001 - surface as CAN-flavour error
            raise can.CanOperationError(f"g4ros command publish failed: {exc}") from exc
        finally:
            self._tx.reset()

    def _on_a1z_frame(self, msg) -> None:
        self._enqueue(decode_a1z_frame(msg))

    def _on_claw(self, msg) -> None:
        self._enqueue([decode_claw(msg)])

    def _enqueue(self, messages: List[can.Message]) -> None:
        with self._rx_cv:
            self._rx_queue.extend(messages)
            self._rx_cv.notify_all()

    def __enter__(self) -> "RosTopicBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.shutdown()
