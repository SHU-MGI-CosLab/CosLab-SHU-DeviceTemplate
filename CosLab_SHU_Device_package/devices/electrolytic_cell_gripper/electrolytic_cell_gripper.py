"""
Electrolytic Cell Gripper Workstation Driver (电解池夹爪工作站)
Combines 2x QYL stepper motors + 1x DH PGE gripper into a single device
with two high-level actions: pick_sample and place_sample.

All three physical devices share COM29 via RS-485 Modbus RTU.
Motor 1 (slave_id=1), Motor 2 (slave_id=2), Gripper (slave_id=5).

Communication: 115200, 8N1
Motor: FC 03/06/10, speed/accel = register raw value (5000 in debug software = reg 5000)
Gripper: FC 03/06

v7: Self-contained driver with lazy serial initialization.
    Serial port is opened on first use (not only in initialize()),
    so it works regardless of whether the framework calls initialize().
"""

import logging
import struct
import time as time_module
from typing import Dict, Any, Optional

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

try:
    import serial
    from serial import Serial
except ImportError:
    serial = None
    Serial = None


# ═══════════════════════════════════════════════════════════════════
# CRC16 Modbus
# ═══════════════════════════════════════════════════════════════════

def _crc16_modbus(data: bytes) -> bytes:
    """Calculate Modbus RTU CRC16, returns 2 bytes (low, high)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)  # low byte first


# ═══════════════════════════════════════════════════════════════════
# Internal Modbus helpers
# ═══════════════════════════════════════════════════════════════════

class _ModbusRTU:
    """Low-level Modbus RTU helper bound to a Serial port."""

    def __init__(self, ser: Serial, logger: logging.Logger):
        self._ser = ser
        self.logger = logger

    # ── Frame builders ──────────────────────────────────────────

    @staticmethod
    def _build_read_frame(slave_id: int, register: int, count: int = 1) -> bytes:
        frame = struct.pack(">B B H H", slave_id, 0x03, register, count)
        return frame + _crc16_modbus(frame)

    @staticmethod
    def _build_write_single_frame(slave_id: int, register: int, value: int) -> bytes:
        frame = struct.pack(">B B H H", slave_id, 0x06, register, value & 0xFFFF)
        return frame + _crc16_modbus(frame)

    @staticmethod
    def _build_write_multiple_frame(slave_id: int, start_register: int, values: list) -> bytes:
        count = len(values)
        byte_count = count * 2
        frame = struct.pack(">B B H H B", slave_id, 0x10, start_register, count, byte_count)
        for v in values:
            frame += struct.pack(">H", v & 0xFFFF)
        return frame + _crc16_modbus(frame)

    # ── Send / Receive ──────────────────────────────────────────

    def send_and_receive(self, frame: bytes, expect_len: int) -> Optional[bytes]:
        if self._ser is None or not self._ser.is_open:
            self.logger.error("Serial not open")
            return None

        self.logger.debug(f"TX: {frame.hex(' ')}")
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        time_module.sleep(0.05)

        raw = self._ser.read(expect_len + 10)
        self.logger.debug(f"RX: {raw.hex(' ') if raw else '(empty)'}")

        if len(raw) < expect_len:
            self.logger.warning(f"Short response: expected >={expect_len}, got {len(raw)}")
            if len(raw) == 0:
                return None

        fc_sent = frame[1]
        slave_id = frame[0]
        for i in range(len(raw) - 1):
            if raw[i] == slave_id and raw[i + 1] == fc_sent:
                resp = raw[i:]
                if len(resp) >= expect_len:
                    payload = resp[:expect_len - 2]
                    crc_recv = resp[expect_len - 2:expect_len]
                    if _crc16_modbus(payload) == crc_recv:
                        return resp[:expect_len]
                    else:
                        self.logger.warning("CRC mismatch")
                        return resp[:expect_len]
                break
            if raw[i] == slave_id and raw[i + 1] == (fc_sent | 0x80):
                error_code = raw[i + 2] if i + 2 < len(raw) else 0xFF
                self.logger.error(f"Modbus error: FC=0x{raw[i+1]:02X}, err=0x{error_code:02X}")
                return None

        self.logger.warning("Could not locate valid response frame")
        return raw[:expect_len] if len(raw) >= expect_len else None

    # ── High-level register operations ──────────────────────────

    def read_registers(self, slave_id: int, start: int, count: int = 1) -> Optional[list]:
        frame = self._build_read_frame(slave_id, start, count)
        expect = 3 + count * 2 + 2
        resp = self.send_and_receive(frame, expect)
        if resp is None or len(resp) < expect:
            return None
        values = []
        for i in range(count):
            offset = 3 + i * 2
            values.append(struct.unpack(">H", resp[offset:offset + 2])[0])
        return values

    def write_single(self, slave_id: int, register: int, value: int) -> bool:
        frame = self._build_write_single_frame(slave_id, register, value)
        resp = self.send_and_receive(frame, 8)
        return resp is not None

    def write_multiple(self, slave_id: int, start: int, values: list) -> bool:
        frame = self._build_write_multiple_frame(slave_id, start, values)
        resp = self.send_and_receive(frame, 8)
        return resp is not None


# ═══════════════════════════════════════════════════════════════════
# Helper: signed 32-bit conversion
# ═══════════════════════════════════════════════════════════════════

def _from_signed32(val: int) -> tuple:
    if val < 0:
        val += 0x100000000
    return ((val >> 16) & 0xFFFF, val & 0xFFFF)

def _to_signed32(high: int, low: int) -> int:
    val = (high << 16) | low
    if val >= 0x80000000:
        val -= 0x100000000
    return val


# ═══════════════════════════════════════════════════════════════════
# Motor register addresses
# ═══════════════════════════════════════════════════════════════════

_M_STATUS       = 0x0000
_M_POS_H        = 0x0001
_M_POS_L        = 0x0002
_M_SPEED        = 0x0003
_M_ESTOP        = 0x0004
_M_ENABLE       = 0x0006
_M_PP_TARGET_H  = 0x0010   # Point-to-point (absolute) mode
_M_PP_TARGET_L  = 0x0011
_M_PP_INIT_SPD  = 0x0012
_M_PP_RUN_SPD   = 0x0013
_M_PP_ACCEL     = 0x0014
_M_PP_TOL       = 0x0015
_M_HOME         = 0x001F
_M_FW_STEPS_H   = 0x0040   # Forward (relative) mode
_M_FW_STEPS_L   = 0x0041
_M_FW_INIT_SPD  = 0x0042
_M_FW_RUN_SPD   = 0x0043
_M_FW_ACCEL     = 0x0044
_M_FW_TOL       = 0x0045

# Gripper register addresses
_G_INIT         = 0x0100
_G_FORCE        = 0x0101
_G_TARGET_POS   = 0x0103
_G_SPEED        = 0x0104
_G_INIT_STATE   = 0x0200
_G_GRIP_STATE   = 0x0201
_G_ACTUAL_POS   = 0x0202

# Motor status map
_MOTOR_STATUS = {0: "Idle", 1: "Busy", 2: "Stopped", 3: "LimitPos", 4: "LimitNeg"}


# ═══════════════════════════════════════════════════════════════════
# Main workstation class
# ═══════════════════════════════════════════════════════════════════

class ElectrolyticCellGripper:
    """
    Electrolytic Cell Gripper Workstation (电解池夹爪).

    Combines:
      - Motor 1 (slave_id=1): Horizontal slide
      - Motor 2 (slave_id=2): Vertical slide
      - Gripper  (slave_id=5): DH PGE parallel gripper

    Exposes two high-level actions:
      - pick_sample(): Grab a sample from the cell
      - place_sample(): Put the sample back

    All motion parameters are hardcoded per the validated workflow.
    """

    _ros_node: "BaseROS2DeviceNode"

    # Motor speed/accel: user confirmed 5000 = register raw value in debug software
    MOTOR_SPEED     = 5000   # register raw value
    MOTOR_ACCEL     = 5000   # register raw value (acceleration time)
    MOTOR_INIT_SPD  = 50     # initial speed register raw value
    MOTOR_TOL       = 100    # tolerance in steps

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")

        self.device_id = device_id or "electrolytic_cell_gripper"
        self.config = config or {}
        self.logger = logging.getLogger(f"ECG.{self.device_id}")

        # ── Config ──────────────────────────────────────────────
        self._port_name: str = self.config.get("port", "COM29")
        self._baudrate: int = int(self.config.get("baudrate", 115200))
        self._timeout: float = float(self.config.get("timeout", 0.5))

        self._motor1_id: int = int(self.config.get("motor1_slave_id", 1))
        self._motor2_id: int = int(self.config.get("motor2_slave_id", 2))
        self._gripper_id: int = int(self.config.get("gripper_slave_id", 5))

        self._ser: Optional[Serial] = None
        self._bus: Optional[_ModbusRTU] = None

        # ── Data store ──────────────────────────────────────────
        self.data: Dict[str, Any] = {
            "status": "Idle",
        }

    # ── Framework hooks ─────────────────────────────────────────

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    def _ensure_serial(self) -> bool:
        """
        Lazy serial initialization.
        If serial is already open, return True.
        If not, try to open it now. This ensures the serial port is available
        regardless of whether the framework called initialize() or not.
        """
        if self._bus is not None and self._ser is not None and self._ser.is_open:
            return True

        self.logger.info(f"Opening serial port {self._port_name} @ {self._baudrate}...")
        try:
            if Serial is None:
                self.logger.error("pyserial not installed")
                return False

            self._ser = Serial(
                port=self._port_name,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout,
            )

            if not self._ser.is_open:
                self.logger.error(f"Failed to open {self._port_name}")
                return False

            self._bus = _ModbusRTU(self._ser, self.logger)
            self.logger.info(f"Serial {self._port_name} opened successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to open serial: {e}")
            return False

    async def initialize(self) -> bool:
        """Open serial port and verify communication."""
        self.logger.info("initialize() called")
        ok = self._ensure_serial()
        if ok:
            self.data["status"] = "Idle"
            self.logger.info("initialize() SUCCESS — serial is open")
        else:
            self.data["status"] = "Error"
            self.logger.error("initialize() FAILED — could not open serial")
        return ok

    async def cleanup(self) -> bool:
        """Close serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self._bus = None
        self.data["status"] = "Offline"
        return True

    # ── Properties ──────────────────────────────────────────────

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    # ── Internal motor helpers ──────────────────────────────────

    def _motor_set_position_zero(self, slave_id: int) -> bool:
        """Write 0 to actual position registers (set current position as zero)."""
        return self._bus.write_multiple(slave_id, _M_POS_H, [0, 0])

    def _motor_move_absolute(self, slave_id: int, position: int,
                              speed: int = None, accel: int = None) -> bool:
        """
        Move motor to absolute position using point-to-point mode.
        Args:
            position: target in steps (signed 32-bit)
            speed: register raw value (default: MOTOR_SPEED=5000)
            accel: register raw value (default: MOTOR_ACCEL=5000)
        """
        spd = speed if speed is not None else self.MOTOR_SPEED
        acc = accel if accel is not None else self.MOTOR_ACCEL
        pos_h, pos_l = _from_signed32(int(position))
        values = [pos_h, pos_l, self.MOTOR_INIT_SPD, spd, acc, self.MOTOR_TOL]
        return self._bus.write_multiple(slave_id, _M_PP_TARGET_H, values)

    def _motor_read_status(self, slave_id: int) -> Optional[Dict]:
        """Read motor status, position, speed."""
        regs = self._bus.read_registers(slave_id, _M_STATUS, 4)
        if regs is None or len(regs) < 4:
            return None
        return {
            "status_code": regs[0],
            "status": _MOTOR_STATUS.get(regs[0], f"Unknown({regs[0]})"),
            "position": _to_signed32(regs[1], regs[2]),
            "speed_reg": regs[3],
        }

    def _motor_emergency_stop(self, slave_id: int) -> bool:
        """Send emergency stop to motor."""
        return self._bus.write_single(slave_id, _M_ESTOP, 0x0001)

    async def _motor_wait_idle(self, slave_id: int, timeout: float = 120.0,
                                poll_interval: float = 0.3) -> bool:
        """Wait until motor status returns to Idle (0) or non-running state."""
        elapsed = 0.0
        motor_name = f"Motor{slave_id}"
        while elapsed < timeout:
            info = self._motor_read_status(slave_id)
            if info is not None:
                code = info["status_code"]
                if code != 1:  # Not "Busy"
                    self.logger.info(f"{motor_name} idle: pos={info['position']}, status={info['status']}")
                    return True
            await self._ros_node.sleep(poll_interval)
            elapsed += poll_interval
        self.logger.warning(f"{motor_name} wait_idle timed out after {timeout}s")
        return False

    # ── Internal gripper helpers ────────────────────────────────

    def _gripper_init(self) -> bool:
        """Send homing command (0x01) to gripper."""
        return self._bus.write_single(self._gripper_id, _G_INIT, 0x01)

    async def _gripper_wait_init(self, timeout: float = 30.0) -> bool:
        """Wait for gripper initialization to complete."""
        elapsed = 0.0
        while elapsed < timeout:
            regs = self._bus.read_registers(self._gripper_id, _G_INIT_STATE, 1)
            if regs is not None and regs[0] == 1:
                self.logger.info("Gripper init complete")
                return True
            await self._ros_node.sleep(0.5)
            elapsed += 0.5
        self.logger.warning("Gripper init timed out")
        return False

    def _gripper_set_force(self, force: int) -> bool:
        force = max(20, min(100, force))
        return self._bus.write_single(self._gripper_id, _G_FORCE, force)

    def _gripper_set_speed(self, speed: int) -> bool:
        speed = max(1, min(100, speed))
        return self._bus.write_single(self._gripper_id, _G_SPEED, speed)

    def _gripper_set_position(self, position: int) -> bool:
        """Set gripper target position. 0=fully closed, 1000=fully open."""
        position = max(0, min(1000, position))
        return self._bus.write_single(self._gripper_id, _G_TARGET_POS, position)

    async def _gripper_wait_done(self, timeout: float = 15.0) -> bool:
        """Wait for gripper to finish moving (grip_state != 0)."""
        elapsed = 0.0
        while elapsed < timeout:
            regs = self._bus.read_registers(self._gripper_id, _G_GRIP_STATE, 1)
            if regs is not None and regs[0] in (1, 2, 3):
                state_names = {1: "Reached", 2: "Gripped", 3: "Dropped"}
                self.logger.info(f"Gripper done: {state_names.get(regs[0], regs[0])}")
                return True
            await self._ros_node.sleep(0.2)
            elapsed += 0.2
        self.logger.warning("Gripper wait timed out")
        return False

    # ═══════════════════════════════════════════════════════════════
    #  ACTION 1: 夹取样品 (Pick Sample)
    # ═══════════════════════════════════════════════════════════════

    async def pick_sample(self):
        """
        夹取样品 — 完整序列：
          1. 夹爪初始化
          2. 设置夹爪力 50%
          3. 设置夹爪速度 100%
          4. 两个电机设置当前位置为零点
          5. 1号电机移动到 838000 步 (speed=5000, accel=5000)
          6. 2号电机移动到 -800000 步
          7. 夹爪闭合
          8. 2号电机移动到 0 步
          9. 1号电机移动到 0 步
         10. 2号电机移动到 -630000 步
        """
        # ── Lazy serial open ────────────────────────────────────
        if not self._ensure_serial():
            self.logger.error("pick_sample ABORTED: cannot open serial port")
            self.data["status"] = "Error"
            return

        self.data["status"] = "Busy"
        self.logger.info("=" * 60)
        self.logger.info("pick_sample START")
        self.logger.info("=" * 60)

        try:
            # Step 1: Gripper init (homing)
            self.logger.info("[1/11] Gripper init (homing)...")
            self._gripper_init()
            await self._gripper_wait_init(timeout=30.0)

            # Step 2: Set gripper force 50%
            self.logger.info("[2/11] Set gripper force = 50%")
            self._gripper_set_force(50)
            time_module.sleep(0.05)

            # Step 3: Set gripper speed 100%
            self.logger.info("[3/11] Set gripper speed = 100%")
            self._gripper_set_speed(100)
            time_module.sleep(0.05)

            # Step 4: Both motors set current position as zero
            self.logger.info("[4/11] Motor1 + Motor2 set position zero")
            self._motor_set_position_zero(self._motor1_id)
            time_module.sleep(0.05)
            self._motor_set_position_zero(self._motor2_id)
            time_module.sleep(0.05)

            # Step 5: Motor 1 move to 838000
            self.logger.info("[5/11] Motor1 move to 838000 steps (speed=5000, accel=5000)")
            self._motor_move_absolute(self._motor1_id, 838000, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor1_id, timeout=120.0)

            # Step 6: Motor 2 move to -800000
            self.logger.info("[6/11] Motor2 move to -800000 steps")
            self._motor_move_absolute(self._motor2_id, -800000, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            # Step 7: Gripper close (position=0)
            self.logger.info("[7/11] Gripper close")
            self._gripper_set_position(0)
            await self._gripper_wait_done(timeout=15.0)

            # Step 8: Motor 2 move to 0
            self.logger.info("[8/11] Motor2 move to 0 steps")
            self._motor_move_absolute(self._motor2_id, 0, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            # Step 9: Motor 1 move to 0
            self.logger.info("[9/11] Motor1 move to 0 steps")
            self._motor_move_absolute(self._motor1_id, 0, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor1_id, timeout=120.0)

            # Step 10: Motor 2 move to -850000
            # self.logger.info("[10/11] Motor2 move to -850000 steps")
            # self._motor_move_absolute(self._motor2_id, -850000, speed=5000, accel=5000)
            self.logger.info("[10/11] Motor2 move to -630000 steps")
            self._motor_move_absolute(self._motor2_id, -630000, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            self.data["status"] = "Idle"
            self.logger.info("=" * 60)
            self.logger.info("pick_sample COMPLETE")
            self.logger.info("=" * 60)

        except Exception as e:
            self.logger.error(f"pick_sample failed: {e}")
            self.data["status"] = "Error"

    # ═══════════════════════════════════════════════════════════════
    #  ACTION 2: 放下样品 (Place Sample)
    # ═══════════════════════════════════════════════════════════════

    async def place_sample(self):
        """
        放下样品 — 完整序列：
          1. 2号电机移动到 0 步
          2. 1号电机移动到 838000 步
          3. 2号电机移动到 -790000 步
          4. 夹爪张开
          5. 2号电机移动到 0 步
          6. 1号电机移动到 0 步
        """
        # ── Lazy serial open ────────────────────────────────────
        if not self._ensure_serial():
            self.logger.error("place_sample ABORTED: cannot open serial port")
            self.data["status"] = "Error"
            return

        self.data["status"] = "Busy"
        self.logger.info("=" * 60)
        self.logger.info("place_sample START")
        self.logger.info("=" * 60)

        try:
            # Step 1: Motor 2 move to 0
            self.logger.info("[1/6] Motor2 move to 0 steps")
            self._motor_move_absolute(self._motor2_id, 0, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            # Step 2: Motor 1 move to 838000
            self.logger.info("[2/6] Motor1 move to 838000 steps")
            self._motor_move_absolute(self._motor1_id, 838000, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor1_id, timeout=120.0)

            # Step 3: Motor 2 move to -790000
            self.logger.info("[3/6] Motor2 move to -790000 steps")
            self._motor_move_absolute(self._motor2_id, -790000, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            # Step 4: Gripper open (position=1000)
            self.logger.info("[4/6] Gripper open")
            self._gripper_set_position(1000)
            await self._gripper_wait_done(timeout=15.0)

            # Step 5: Motor 2 move to 0
            self.logger.info("[5/6] Motor2 move to 0 steps")
            self._motor_move_absolute(self._motor2_id, 0, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor2_id, timeout=120.0)

            # Step 6: Motor 1 move to 0
            self.logger.info("[6/6] Motor1 move to 0 steps")
            self._motor_move_absolute(self._motor1_id, 0, speed=5000, accel=5000)
            await self._motor_wait_idle(self._motor1_id, timeout=120.0)

            self.data["status"] = "Idle"
            self.logger.info("=" * 60)
            self.logger.info("place_sample COMPLETE")
            self.logger.info("=" * 60)

        except Exception as e:
            self.logger.error(f"place_sample failed: {e}")
            self.data["status"] = "Error"

    # ═══════════════════════════════════════════════════════════════
    #  Emergency stop
    # ═══════════════════════════════════════════════════════════════

    async def emergency_stop(self):
        """Emergency stop all motors immediately."""
        self.logger.warning("EMERGENCY STOP")
        if self._bus is not None:
            self._motor_emergency_stop(self._motor1_id)
            time_module.sleep(0.02)
            self._motor_emergency_stop(self._motor2_id)
        self.data["status"] = "Stopped"
