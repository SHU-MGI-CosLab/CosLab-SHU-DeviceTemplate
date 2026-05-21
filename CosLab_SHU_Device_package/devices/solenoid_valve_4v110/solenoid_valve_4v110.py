"""
亚德客 4V110-06 DC24V 电磁阀驱动
通过 Arduino Uno GPIO + 继电器模块控制（复用 CNI 激光器 Arduino）
通信方式：串口 ASCII 指令

接线：
  Arduino D7 → 继电器模块 IN
  Arduino 5V → 继电器模块 VCC
  Arduino GND → 继电器模块 GND
  继电器 COM → 24V DC (+)
  继电器 NO  → 电磁阀线圈+ (红)
  电磁阀线圈- (黑) → 24V DC GND (-)
"""

import logging
import time as time_module
from typing import Dict, Any

try:
    import serial
except ImportError:
    serial = None

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None


class SolenoidValve4V110:
    """亚德客 4V110-06 DC24V 二位五通电磁阀
    
    通过 Arduino Uno 的 GPIO D7 驱动继电器模块，控制 24V 线圈通断。
    串口 ASCII 指令协议：
      VALVE ON\\n  → 打开电磁阀（D7 HIGH → 继电器吸合 → 线圈通电）
      VALVE OFF\\n → 关闭电磁阀（D7 LOW → 继电器释放 → 线圈断电 → 弹簧复位）
      VALVE?\\n    → 查询状态，返回 "VALVE:ON" 或 "VALVE:OFF"
    """

    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        # --- 标准 init 模式 ---
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')

        self.device_id = device_id or "solenoid_valve_4v110"
        self.config = config or {}
        self.logger = logging.getLogger(f"SolenoidValve4V110.{self.device_id}")

        # --- self.data 必须预填充所有 @property 字段 ---
        self.data = {
            "status": "Idle",
            "valve_position": "Closed",
        }

        # --- 串口配置（从 config 或 kwargs 双重读取）---
        self._port = self.config.get("port") or kwargs.get("port", "COM3")
        self._baudrate = int(self.config.get("baudrate") or kwargs.get("baudrate", 9600))
        self._timeout = float(self.config.get("timeout") or kwargs.get("timeout", 1))

        # --- 串口连接 ---
        self.ser = None
        try:
            if serial is not None:
                self.ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baudrate,
                    timeout=self._timeout,
                )
                self.logger.info(f"Serial connected: {self._port} @ {self._baudrate}")
            else:
                self.logger.warning("pyserial not installed, running in dry-run mode")
        except Exception as e:
            self.logger.error(f"Failed to open serial port {self._port}: {e}")
            self.ser = None

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    async def initialize(self) -> bool:
        """初始化：确保电磁阀处于关闭状态"""
        self._send_command("VALVE OFF")
        self.data["status"] = "Idle"
        self.data["valve_position"] = "Closed"
        self.logger.info("Initialized: valve closed")
        return True

    async def cleanup(self) -> bool:
        """清理：关闭电磁阀并释放串口"""
        try:
            self._send_command("VALVE OFF")
            self.data["valve_position"] = "Closed"
        except Exception:
            pass
        self.data["status"] = "Offline"
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.logger.info("Cleanup complete")
        return True

    # ========== 通信辅助 ==========

    def _send_command(self, cmd: str) -> str:
        """发送 ASCII 指令到 Arduino，返回响应"""
        if self.ser is None or not self.ser.is_open:
            self.logger.warning(f"Serial not available, dry-run: {cmd}")
            return ""
        try:
            self.ser.reset_input_buffer()
            self.ser.write(f"{cmd}\n".encode("ascii"))
            time_module.sleep(0.05)  # 等待 Arduino 处理
            response = self.ser.readline().decode("ascii", errors="ignore").strip()
            self.logger.debug(f"TX: {cmd} → RX: {response}")
            return response
        except Exception as e:
            self.logger.error(f"Serial communication error: {e}")
            return ""

    # ========== 电磁阀标准动作（对齐已有接口）==========

    async def open(self, **kwargs) -> bool:
        """打开电磁阀（线圈通电，阀芯换向）"""
        self.data["status"] = "Busy"
        resp = self._send_command("VALVE ON")
        self.data["valve_position"] = "Open"
        self.data["status"] = "Idle"
        self.logger.info("Valve opened")
        return True

    async def close(self, **kwargs) -> bool:
        """关闭电磁阀（线圈断电，弹簧复位）"""
        self.data["status"] = "Busy"
        resp = self._send_command("VALVE OFF")
        self.data["valve_position"] = "Closed"
        self.data["status"] = "Idle"
        self.logger.info("Valve closed")
        return True

    async def set_valve_position(self, position, **kwargs) -> bool:
        """设置阀门位置。参数名必须是 position（接口契约）
        
        Args:
            position: "Open" 或 "Closed"
        """
        pos_str = str(position).strip().lower()
        if pos_str in ("open", "1", "on", "true"):
            return await self.open()
        elif pos_str in ("closed", "close", "0", "off", "false"):
            return await self.close()
        else:
            self.logger.warning(f"Unknown valve position: {position}, closing for safety")
            return await self.close()

    async def is_open(self, **kwargs) -> bool:
        """检查电磁阀是否打开"""
        resp = self._send_command("VALVE?")
        if "ON" in resp.upper():
            self.data["valve_position"] = "Open"
            return True
        elif "OFF" in resp.upper():
            self.data["valve_position"] = "Closed"
            return False
        # 若无法通信，依据本地状态
        return self.data.get("valve_position", "Closed") == "Open"

    async def is_closed(self, **kwargs) -> bool:
        """检查电磁阀是否关闭"""
        return not await self.is_open()

    async def send_command(self, command: str, **kwargs) -> str:
        """发送自定义指令（对齐已有接口）"""
        return self._send_command(str(command))

    # ========== 属性（@property）==========

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def valve_position(self) -> str:
        return self.data.get("valve_position", "Closed")
