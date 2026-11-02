"""Recoil motor controller CAN protocol (Berkeley Humanoid Lite actuators).

Frame layout and parameter ids follow berkeley_humanoid_lite_lowlevel/recoil/core.py
at HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel@652777c:

    CAN id = (function << 7) | device_id          (standard 11-bit id)
    NMT             set_mode:  [mode, device_id]
    RECEIVE_SDO     write:     [0x20, param_lo, param_hi, 0, value(4)]
                    read:      [0x40, param_lo, param_hi]  -> TRANSMIT_SDO [value(4), ...]
    RECEIVE_PDO_2   [position_target f32, velocity_target f32]
                    -> TRANSMIT_PDO_2 [position_measured f32, velocity_measured f32]
    HEARTBEAT       empty frame, feeds the firmware watchdog (default 1 s)

Two things are deliberately different from the upstream Bus, both found on
this lab's arms:

  * the PDO reply is filtered by function code TRANSMIT_PDO_2. Upstream takes
    the first frame from the device, so a late SDO reply was read as a position.
  * replies wait on an absolute deadline (default 4 ms) instead of 1 ms, and a
    run of error frames does not restart the wait.

python-can (socketcan) is the only dependency.
"""

from __future__ import annotations

import struct
import time

import can


class Function:
    NMT = 0b0000
    TRANSMIT_PDO_1 = 0b0011
    RECEIVE_PDO_1 = 0b0100
    TRANSMIT_PDO_2 = 0b0101
    RECEIVE_PDO_2 = 0b0110
    TRANSMIT_SDO = 0b1011
    RECEIVE_SDO = 0b1100
    FLASH = 0b1101
    HEARTBEAT = 0b1110


class Mode:
    DISABLED = 0x00
    IDLE = 0x01
    DAMPING = 0x02
    CALIBRATION = 0x05
    CURRENT = 0x10
    TORQUE = 0x11
    VELOCITY = 0x12
    POSITION = 0x13


class Param:
    DEVICE_ID = 0x000
    FIRMWARE_VERSION = 0x004
    WATCHDOG_TIMEOUT = 0x008
    MODE = 0x010
    ERROR = 0x014
    GEAR_RATIO = 0x01C
    POSITION_KP = 0x020
    VELOCITY_KP = 0x028      # the "kd" of the position loop (upstream write_position_kd)
    TORQUE_LIMIT = 0x030
    POSITION_MEASURED = 0x060
    BUS_VOLTAGE = 0x100


DEVICE_ID_MASK = 0x7F
FUNC_SHIFT = 7


class RecoilBus:
    """One socketcan channel shared by every actuator on it."""

    def __init__(self, channel: str, bitrate: int = 1_000_000, reply_timeout: float = 0.004) -> None:
        self.channel = channel
        self.reply_timeout = reply_timeout
        self._bus = can.interface.Bus(interface="socketcan", channel=channel, bitrate=bitrate)

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - shutting down a dead adapter must not raise
            pass

    # -- raw frames -------------------------------------------------------------
    def send(self, device_id: int, function: int, data: bytes = b"") -> None:
        if not 0 <= device_id <= DEVICE_ID_MASK:
            raise ValueError(f"device_id {device_id} out of range")
        self._bus.send(can.Message(arbitration_id=(function << FUNC_SHIFT) | device_id,
                                   is_extended_id=False, data=data))

    def receive(self, device_id: int, function: int, timeout: float | None = None) -> bytes | None:
        """First frame from `device_id` with `function`, or None at the deadline."""
        deadline = time.monotonic() + (self.reply_timeout if timeout is None else timeout)
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            message = self._bus.recv(timeout=left)
            if message is None:
                return None
            if message.is_error_frame:
                continue
            if (message.arbitration_id & DEVICE_ID_MASK) == device_id and \
                    (message.arbitration_id >> FUNC_SHIFT) == function:
                return bytes(message.data)

    def flush(self) -> None:
        while self._bus.recv(timeout=0) is not None:
            pass

    # -- commands ---------------------------------------------------------------
    def set_mode(self, device_id: int, mode: int) -> None:
        self.send(device_id, Function.NMT, struct.pack("<BB", mode, device_id))

    def feed(self, device_id: int) -> None:
        self.send(device_id, Function.HEARTBEAT)

    def ping(self, device_id: int, timeout: float = 0.05) -> bool:
        self.send(device_id, Function.RECEIVE_PDO_1, b"\xCA")
        reply = self.receive(device_id, Function.TRANSMIT_PDO_1, timeout)
        return bool(reply) and reply[0] == 0xCA

    def write_f32(self, device_id: int, param: int, value: float) -> None:
        self.send(device_id, Function.RECEIVE_SDO, struct.pack("<BHB", 0x01 << 5, param, 0) + struct.pack("<f", value))

    def read_raw(self, device_id: int, param: int, timeout: float = 0.05) -> bytes | None:
        self.send(device_id, Function.RECEIVE_SDO, struct.pack("<BH", 0x02 << 5, param))
        reply = self.receive(device_id, Function.TRANSMIT_SDO, timeout)
        return reply[:4] if reply and len(reply) >= 4 else None

    def read_f32(self, device_id: int, param: int, timeout: float = 0.05) -> float | None:
        raw = self.read_raw(device_id, param, timeout)
        return struct.unpack("<f", raw)[0] if raw else None

    def read_u32(self, device_id: int, param: int, timeout: float = 0.05) -> int | None:
        raw = self.read_raw(device_id, param, timeout)
        return struct.unpack("<L", raw)[0] if raw else None

    def write_gains(self, device_id: int, kp: float, kd: float, torque_limit: float) -> None:
        self.write_f32(device_id, Param.POSITION_KP, kp)
        self.write_f32(device_id, Param.VELOCITY_KP, kd)
        self.write_f32(device_id, Param.TORQUE_LIMIT, torque_limit)

    def exchange_pdo_2(self, device_id: int, position: float, velocity: float = 0.0
                       ) -> tuple[float, float] | None:
        """Send a position target, return (measured position, velocity) or None."""
        self.send(device_id, Function.RECEIVE_PDO_2, struct.pack("<ff", position, velocity))
        reply = self.receive(device_id, Function.TRANSMIT_PDO_2)
        if not reply or len(reply) < 8:
            return None
        return struct.unpack("<ff", reply[:8])
