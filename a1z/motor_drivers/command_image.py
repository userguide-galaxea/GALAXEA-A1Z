"""56-byte A1Z command image buffering for the direct G4 SPI transport.

The direct-SPI transport (``spi_bus.py``) ships one 56-byte payload per arm:
seven 8-byte slots holding the raw CAN data of motors 1..6 and the claw.
This module owns the CAN-ID → slot mapping and the per-tick flush trigger.

The ROS2 topic transport (``ros_topic_bus.py``) does NOT use this: the MCU
firmware speaks a frame-wise protocol there (one CAN frame per message), so
``RosTopicBus`` publishes each frame immediately without slot buffering.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

NUM_ARM_MOTORS = 6
GRIPPER_CAN_ID = 7
HYBRID_CMD_ID_BASE = 0x300
CLAW_SLOT = 6
PAYLOAD_SIZE = 7 * 8

_ARM_SLOTS_MASK = (1 << NUM_ARM_MOTORS) - 1


class CommandImage:
    """Latest 7×8-byte command payload for one arm, with dirty-slot tracking.

    ``write()`` buffers one CAN frame's 8-byte payload into its slot and
    reports when all six arm-motor slots have been written since the last
    ``reset()`` — the flush trigger, matching the SDK's per-tick burst of
    six motor frames followed by an optional claw frame (the claw slot just
    carries its most recent value). Frames on CAN IDs without a slot (the
    0x7FF management broadcasts: MotorA enable/disable/set-zero, gripper
    mode-4 register write) are dropped with a one-time warning; the G4
    firmware must own motor enable and gripper mode setup.
    """

    def __init__(self) -> None:
        self.payload = bytearray(PAYLOAD_SIZE)
        self.claw_seen = False
        self._dirty = 0
        self._warned_ids: set = set()

    def write(self, arbitration_id: int, data: bytes) -> bool:
        """Buffer one payload; return True when the image should be flushed."""
        arb = int(arbitration_id)
        if 1 <= arb <= NUM_ARM_MOTORS:
            slot = arb - 1
        elif arb in (GRIPPER_CAN_ID, HYBRID_CMD_ID_BASE + GRIPPER_CAN_ID):
            slot = CLAW_SLOT
            self.claw_seen = True
        else:
            self._warn_once(
                arb,
                "not transportable over the lemo board channel; dropping "
                "(G4 firmware must handle it)",
            )
            return False
        if len(data) != 8:
            self._warn_once(
                arb,
                f"dropping {len(data)}-byte frame; channel is fixed 8-byte slots",
            )
            return False
        self.payload[slot * 8 : slot * 8 + 8] = data
        self._dirty |= 1 << slot
        return (self._dirty & _ARM_SLOTS_MASK) == _ARM_SLOTS_MASK

    def reset(self) -> None:
        """Clear dirty tracking after a flush; payload values persist."""
        self._dirty = 0

    def _warn_once(self, arb: int, what: str) -> None:
        if arb not in self._warned_ids:
            self._warned_ids.add(arb)
            logger.warning("[lemo-bus] CAN ID 0x%03x: %s", arb, what)
