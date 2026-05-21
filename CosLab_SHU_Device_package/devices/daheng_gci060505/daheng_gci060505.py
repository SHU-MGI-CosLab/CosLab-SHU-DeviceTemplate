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


class DahengGCI060505:
    """大恒光电 GCI-060505 LED光源驱动（通过 Arduino + MCP4725 DAC 控制）

    通信协议（Arduino 串口 115200）：
        PING           → PONG
        ON             → OK:ON
        OFF            → OK:OFF
        BRIGHT:xxx     → OK:BRIGHT:xxx    (xxx = 0~100)
        STATUS         → OK:STATUS:ON:xxx 或 OK:STATUS:OFF:xxx
    """

    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        self.device_id = device_id or "unknown_device"
        self.config = config or {}
        self.logger = logging.getLogger(f"DahengGCI060505.{self.device_id}.daheng_gci060505")

        # 串口配置
        self._port = self.config.get("port", "COM14")
        self._baudrate = self.config.get("baudrate", 115200)
        self._timeout = self.config.get("timeout", 2)
        self._ser = None

        # self.data 必须预填充所有 @property 对应的字段
        self.data = {
            "status": "Idle",
            "brightness": 0.0,
            "light_on": False,
            "max_brightness": 100.0,
        }

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    # ─── 串口通信层 ───

    def _open_serial(self):
        """打开串口连接"""
        if self._ser is not None and self._ser.is_open:
            return True
        if serial is None:
            self.logger.error("pyserial 未安装，请运行: pip install pyserial")
            return False
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            # 等待 Arduino 重启（DTR 复位）
            time_module.sleep(2)
            # 清空缓冲区（包括 Arduino 启动时发的 READY）
            self._ser.reset_input_buffer()
            self.logger.info(f"串口 {self._port} 已打开")
            return True
        except Exception as e:
            self.logger.error(f"串口打开失败: {e}")
            self._ser = None
            return False

    def _send_command(self, cmd: str) -> str:
        """发送指令并读取响应

        Args:
            cmd: 指令字符串（不含换行符）

        Returns:
            响应字符串（已去除换行），失败返回空字符串
        """
        if self._ser is None or not self._ser.is_open:
            if not self._open_serial():
                return ""
        try:
            self._ser.reset_input_buffer()
            self._ser.write(f"{cmd}\n".encode("ascii"))
            self._ser.flush()
            # 读取响应，超时由 serial.timeout 控制
            response = self._ser.readline().decode("ascii", errors="ignore").strip()
            self.logger.debug(f"TX: {cmd} → RX: {response}")
            return response
        except Exception as e:
            self.logger.error(f"通信失败: {e}")
            return ""

    def _close_serial(self):
        """关闭串口"""
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    # ─── 生命周期 ───

    async def initialize(self) -> bool:
        """初始化设备：打开串口并验证通信"""
        if not self._open_serial():
            self.data["status"] = "Error"
            return False

        # PING 测试
        response = self._send_command("PING")
        # 兼容响应：可能带前缀噪声，只要包含 PONG 即可
        if "PONG" in response:
            self.logger.info("Arduino 通信正常")
            self.data["status"] = "Idle"
            # 刷新一次状态
            self._refresh_from_arduino()
            return True
        else:
            self.logger.error(f"PING 测试失败，响应: {response}")
            self.data["status"] = "Error"
            return False

    async def cleanup(self) -> bool:
        """关灯并关闭串口"""
        try:
            self._send_command("OFF")
        except Exception:
            pass
        self._close_serial()
        self.data["status"] = "Offline"
        self.data["light_on"] = False
        return True

    # ─── 属性（@property 返回类型只能是 str / float / bool）───

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def brightness(self) -> float:
        """当前亮度 0~100%"""
        return self.data.get("brightness", 0.0)

    @property
    def light_on(self) -> bool:
        """光源是否开启"""
        return self.data.get("light_on", False)

    @property
    def max_brightness(self) -> float:
        """最大亮度（固定 100）"""
        return self.data.get("max_brightness", 100.0)

    # ─── 内部辅助方法 ───

    def _refresh_from_arduino(self):
        """从 Arduino 读取当前状态并更新 self.data"""
        response = self._send_command("STATUS")
        # 期望格式: OK:STATUS:ON:75 或 OK:STATUS:OFF:0
        if "OK:STATUS:" in response:
            try:
                parts = response.split(":")
                # 找到 STATUS 关键字的位置
                idx = parts.index("STATUS")
                on_off = parts[idx + 1]  # ON 或 OFF
                bright = int(parts[idx + 2])  # 0~100
                self.data["light_on"] = (on_off == "ON")
                self.data["brightness"] = float(bright)
                if self.data["light_on"]:
                    self.data["status"] = "On"
                else:
                    self.data["status"] = "Idle"
            except (ValueError, IndexError) as e:
                self.logger.warning(f"STATUS 解析失败: {response}, 错误: {e}")

    # ─── 动作方法 ───

    async def turn_on(self):
        """开灯（如果亮度为0则自动设为100%）"""
        self.data["status"] = "Busy"
        response = self._send_command("ON")
        # Arduino 返回 OK:ON
        if "OK" in response:
            self.data["light_on"] = True
            if self.data["brightness"] == 0.0:
                self.data["brightness"] = 100.0
            self.data["status"] = "On"
            self.logger.info("开灯成功")
        else:
            self.data["status"] = "Error"
            self.logger.error(f"开灯失败，响应: {response}")

    async def turn_off(self):
        """关灯"""
        self.data["status"] = "Busy"
        response = self._send_command("OFF")
        # Arduino 返回 OK:OFF
        if "OK" in response:
            self.data["light_on"] = False
            self.data["status"] = "Idle"
            self.logger.info("关灯成功")
        else:
            self.data["status"] = "Error"
            self.logger.error(f"关灯失败，响应: {response}")

    async def set_brightness(self, brightness: float):
        """设置亮度

        Args:
            brightness: 亮度百分比，0~100
        """
        # 参数范围限制
        brightness = max(0.0, min(100.0, float(brightness)))
        self.data["status"] = "Busy"

        response = self._send_command(f"BRIGHT:{int(brightness)}")
        # Arduino 返回 OK:BRIGHT:xx
        if "OK" in response:
            self.data["brightness"] = brightness
            if brightness > 0:
                self.data["light_on"] = True
                self.data["status"] = "On"
            else:
                self.data["light_on"] = False
                self.data["status"] = "Idle"
            self.logger.info(f"亮度设置为 {brightness}%")
        else:
            self.data["status"] = "Error"
            self.logger.error(f"设置亮度失败，响应: {response}")

    async def refresh_status(self):
        """从 Arduino 重新读取状态"""
        self._refresh_from_arduino()
        self.logger.info(f"状态刷新: on={self.data['light_on']}, brightness={self.data['brightness']}%")
