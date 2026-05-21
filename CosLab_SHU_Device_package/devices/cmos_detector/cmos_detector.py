import logging
import time as time_module
import os
import json
import csv
from datetime import datetime
from typing import Dict, Any, Optional, List

try:
    import serial
except ImportError:
    serial = None

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None


class CMOSDetector:
    """LCAMV8 CMOS 线阵检测器驱动 (S11639-01, 2048 pixels)

    通讯协议: USB 虚拟串口, 115200 baud, 8N1
    帧格式: [0x81][CMD][DATA1][DATA2][CRC], CRC = sum(byte0~3) & 0xFF
    特殊: 版本查询返回 ASCII 字符串, 采集数据返回 4103 字节大帧
    注意: 单次采集(0x01)无响应, 必须用连续采集(0x02)+停止(0x06)实现单帧采集
    """

    _ros_node: "BaseROS2DeviceNode"

    # ── 命令码 ──
    CMD_SINGLE_ACQ      = 0x01  # 单次采集 (实测无响应, 不使用)
    CMD_CONTINUOUS_ACQ   = 0x02  # 连续采集
    CMD_INTEGRATION_TIME = 0x03  # 设置积分时间
    CMD_GAIN             = 0x04  # 设置增益
    CMD_OFFSET           = 0x05  # 设置偏移
    CMD_STOP             = 0x06  # 暂停采集
    CMD_TRIGGER_MODE     = 0x07  # 触发模式
    CMD_TRIGGER_INTERVAL = 0x08  # 设置触发间隔
    CMD_GET_VERSION      = 0x09  # 获取版本 (返回ASCII)
    CMD_GET_INTEG_TIME   = 0x0A  # 获取积分时间
    CMD_GET_INTERVAL     = 0x0B  # 获取触发间隔
    CMD_AVG_COUNT        = 0x0C  # 设置平均次数
    CMD_ANALOG_OUT       = 0x0D  # 模拟电压输出
    CMD_GET_AVG_COUNT    = 0x0E  # 获取平均次数
    CMD_SYNC_OUTPUT      = 0x0F  # 同步信号输出
    CMD_TRIGGER_OUT2     = 0x10  # Trigger Out2 输出
    CMD_INTEG_UNIT       = 0x11  # 积分时间单位设置
    CMD_GET_INTEG_UNIT   = 0x12  # 获取积分时间单位
    CMD_TTL_BAUDRATE     = 0x13  # TTL 串口波特率设置
    CMD_READ_X_COORD     = 0x15  # 读取 X 坐标值
    CMD_GET_TTL_BAUD     = 0x16  # 获取 TTL 串口波特率
    CMD_SAVE_PARAMS      = 0x22  # 保存参数
    CMD_GET_GAIN         = 0x23  # 获取增益
    CMD_GET_OFFSET       = 0x24  # 获取偏移
    CMD_SET_SMOOTH       = 0x25  # 设置平滑等级
    CMD_GET_SMOOTH       = 0x26  # 获取平滑等级
    CMD_ERASE_FLASH      = 0x27  # 擦除 Flash
    CMD_WRITE_CORRECTION = 0x28  # 写入矫正系数
    CMD_READ_CORRECTION  = 0x29  # 获取矫正系数

    # ── 数据帧常量 ──
    HEAD = 0x81
    PIXEL_COUNT = 2048
    DATA_FRAME_SIZE = 5 + PIXEL_COUNT * 2 + 2  # 4103 bytes

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        self.device_id = device_id or "cmos_detector"
        self.config = config or {}
        self.logger = logging.getLogger(f"CMOSDetector.{self.device_id}")

        self._serial: Optional[serial.Serial] = None
        self._is_acquiring = False
        self._correction_coeffs = [None, None, None, None]  # f[0]~f[3] 用于像素→波长转换
        self._wavelengths: List[float] = []  # 2048 个波长值
        self._last_pixel_data: List[int] = []  # 最近一帧原始像素值

        # 注意: UniLab 框架只支持 float 和 str 类型的属性
        # 所有数值属性使用 float, 布尔使用 str ("true"/"false")
        self.data = {
            "status": "Idle",
            "level": "false",
            "value": "[]",
            "integration_time": 10000.0,
            "integration_time_unit": "ms",
            "gain": 0.0,
            "offset": 0.0,
            "smooth_level": 1.0,
            "average_count": 1.0,
            "trigger_mode": 0.0,
            "trigger_interval": 0.0,
            "version_info": "",
        }

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    # ══════════════════════════════════════════
    #  底层通信
    # ══════════════════════════════════════════

    def _calc_crc(self, data: bytes) -> int:
        """CRC = sum(byte0~byte3) & 0xFF"""
        return sum(data) & 0xFF

    def _build_cmd(self, cmd: int, data1: int = 0x00, data2: int = 0x00) -> bytes:
        """构建 5 字节命令帧"""
        frame = bytes([self.HEAD, cmd, data1, data2])
        crc = self._calc_crc(frame)
        return frame + bytes([crc])

    def _send_cmd(self, cmd: int, data1: int = 0x00, data2: int = 0x00) -> bool:
        """发送命令帧, 返回是否成功"""
        if not self._serial or not self._serial.is_open:
            self.logger.error("串口未打开")
            return False
        frame = self._build_cmd(cmd, data1, data2)
        self.logger.debug(f"发送: {frame.hex()}")
        self._serial.write(frame)
        return True

    def _read_response_5byte(self, timeout: float = 1.0) -> Optional[bytes]:
        """读取 5 字节标准响应帧, 先定位帧头 0x81"""
        if not self._serial or not self._serial.is_open:
            return None
        old_timeout = self._serial.timeout
        try:
            self._serial.timeout = timeout
            # 逐字节找帧头
            deadline = time_module.time() + timeout
            while time_module.time() < deadline:
                b = self._serial.read(1)
                if not b:
                    return None
                if b[0] == self.HEAD:
                    rest = self._serial.read(4)
                    if len(rest) == 4:
                        frame = b + rest
                        self.logger.debug(f"收到5字节帧: {frame.hex()}")
                        return frame
                    return None
            return None
        finally:
            self._serial.timeout = old_timeout

    def _read_version_response(self, timeout: float = 2.0) -> Optional[str]:
        """读取版本信息响应 (ASCII 字符串, 非二进制帧)"""
        if not self._serial or not self._serial.is_open:
            return None
        old_timeout = self._serial.timeout
        try:
            self._serial.timeout = timeout
            time_module.sleep(0.5)
            n = self._serial.in_waiting
            if n > 0:
                data = self._serial.read(n)
                try:
                    text = data.decode('ascii', errors='ignore')
                    self.logger.debug(f"版本信息: {text}")
                    return text
                except Exception:
                    return data.hex()
            return None
        finally:
            self._serial.timeout = old_timeout

    def _read_image_frame(self, timeout: float = 5.0) -> Optional[List[int]]:
        """读取一帧图像数据 (4103 字节), 返回 2048 个像素值"""
        if not self._serial or not self._serial.is_open:
            return None
        old_timeout = self._serial.timeout
        try:
            self._serial.timeout = timeout
            # 找帧头 0x81
            deadline = time_module.time() + timeout
            while time_module.time() < deadline:
                b = self._serial.read(1)
                if not b:
                    return None
                if b[0] == self.HEAD:
                    # 读取剩余 4102 字节
                    buf = bytearray(b)
                    while len(buf) < self.DATA_FRAME_SIZE and time_module.time() < deadline:
                        chunk = self._serial.read(self.DATA_FRAME_SIZE - len(buf))
                        if not chunk:
                            break
                        buf.extend(chunk)
                    if len(buf) < self.DATA_FRAME_SIZE:
                        self.logger.warning(f"图像帧不完整: {len(buf)}/{self.DATA_FRAME_SIZE}")
                        return None
                    # 验证帧头: 81 01 ...
                    if buf[1] != 0x01:
                        self.logger.debug(f"非图像帧, CMD=0x{buf[1]:02X}, 继续查找")
                        continue
                    # 解析 2048 个像素值 (高位在前)
                    pixels = []
                    for i in range(self.PIXEL_COUNT):
                        idx = 5 + i * 2
                        val = (buf[idx] << 8) | buf[idx + 1]
                        pixels.append(val)
                    # 校验 CRC (数据部分累加和, 高低位)
                    data_sum = sum(buf[5:5 + self.PIXEL_COUNT * 2])
                    crc_h = buf[-2]
                    crc_l = buf[-1]
                    expected_crc = (crc_h << 8) | crc_l
                    actual_crc = data_sum & 0xFFFF
                    if expected_crc != actual_crc:
                        self.logger.warning(f"CRC 校验失败: 期望 {expected_crc:#06x}, 实际 {actual_crc:#06x}")
                    return pixels
            return None
        finally:
            self._serial.timeout = old_timeout

    async def _async_sleep(self, seconds: float):
        """安全的异步休眠"""
        try:
            if hasattr(self, '_ros_node') and self._ros_node is not None:
                await self._ros_node.sleep(seconds)
            else:
                time_module.sleep(seconds)
        except Exception:
            time_module.sleep(seconds)

    # ══════════════════════════════════════════
    #  生命周期
    # ══════════════════════════════════════════

    async def initialize(self) -> bool:
        """初始化: 打开串口, 读取版本信息和当前参数"""
        port = self.config.get("port", "COM10")
        baudrate = self.config.get("baudrate", 115200)

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2
            )
            self.logger.info(f"串口 {port} 已打开")
        except Exception as e:
            self.logger.error(f"串口打开失败: {e}")
            self.data["status"] = "Offline"
            return False

        await self._async_sleep(0.3)

        # 读取版本信息
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_VERSION)
        version = self._read_version_response(timeout=2.0)
        if version:
            self.data["version_info"] = version
            self.logger.info(f"设备版本: {version}")

        await self._async_sleep(0.1)

        # 读取当前积分时间
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_INTEG_TIME)
        resp = self._read_response_5byte(timeout=1.0)
        if resp and len(resp) == 5:
            self.data["integration_time"] = float((resp[2] << 8) | resp[3])

        await self._async_sleep(0.1)

        # 读取增益
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_GAIN)
        resp = self._read_response_5byte(timeout=1.0)
        if resp and len(resp) == 5:
            self.data["gain"] = float(resp[2])

        await self._async_sleep(0.1)

        # 读取平滑等级
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_SMOOTH)
        resp = self._read_response_5byte(timeout=1.0)
        if resp and len(resp) == 5:
            self.data["smooth_level"] = float(resp[2])

        await self._async_sleep(0.1)

        # 读取平均次数
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_AVG_COUNT)
        resp = self._read_response_5byte(timeout=1.0)
        if resp and len(resp) == 5:
            self.data["average_count"] = float(resp[2])

        await self._async_sleep(0.1)

        # 读取积分时间单位
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GET_INTEG_UNIT)
        resp = self._read_response_5byte(timeout=1.0)
        if resp and len(resp) == 5:
            self.data["integration_time_unit"] = "us" if resp[2] == 0x01 else "ms"

        self.data["status"] = "Idle"
        self.data["level"] = "true"
        self.logger.info("初始化完成")
        return True

    async def cleanup(self) -> bool:
        """清理: 停止采集, 关闭串口"""
        if self._is_acquiring:
            self._send_cmd(self.CMD_STOP)
            self._is_acquiring = False
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.logger.info("串口已关闭")
        self.data["status"] = "Offline"
        self.data["level"] = "false"
        return True

    # ══════════════════════════════════════════
    #  属性 (@property) — 框架只支持 float 和 str
    # ══════════════════════════════════════════

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def level(self) -> str:
        return self.data.get("level", "false")

    @property
    def value(self) -> str:
        return self.data.get("value", "[]")

    @property
    def integration_time(self) -> float:
        return self.data.get("integration_time", 10000.0)

    @property
    def integration_time_unit(self) -> str:
        return self.data.get("integration_time_unit", "ms")

    @property
    def gain(self) -> float:
        return self.data.get("gain", 0.0)

    @property
    def offset(self) -> float:
        return self.data.get("offset", 0.0)

    @property
    def smooth_level(self) -> float:
        return self.data.get("smooth_level", 1.0)

    @property
    def average_count(self) -> float:
        return self.data.get("average_count", 1.0)

    @property
    def trigger_mode(self) -> float:
        return self.data.get("trigger_mode", 0.0)

    @property
    def trigger_interval(self) -> float:
        return self.data.get("trigger_interval", 0.0)

    @property
    def version_info(self) -> str:
        return self.data.get("version_info", "")

    # ══════════════════════════════════════════
    #  动作方法 — 参数类型也用 float
    # ══════════════════════════════════════════

    async def start_single_acquisition(self) -> str:
        """单帧采集: 启动连续采集→读取一帧→停止采集

        Returns:
            str: JSON 格式的像素数据数组
        """
        self.data["status"] = "Acquiring"
        try:
            self._serial.reset_input_buffer()
            # 启动连续采集
            self._send_cmd(self.CMD_CONTINUOUS_ACQ)
            await self._async_sleep(0.5)
            # 读取一帧
            pixels = self._read_image_frame(timeout=5.0)
            # 立即停止采集
            self._send_cmd(self.CMD_STOP)
            await self._async_sleep(0.1)
            # 清空残留数据
            self._serial.reset_input_buffer()

            if pixels:
                self._last_pixel_data = pixels
                self.data["value"] = json.dumps(pixels)
                self.logger.info(f"采集成功, {len(pixels)} 像素, 范围 [{min(pixels)}-{max(pixels)}]")
                return self.data["value"]
            else:
                self.logger.error("采集失败: 未收到数据")
                return "[]"
        except Exception as e:
            self.logger.error(f"采集异常: {e}")
            return "[]"
        finally:
            self.data["status"] = "Idle"

    async def start_continuous_acquisition(self) -> str:
        """启动连续采集模式

        Returns:
            str: 状态信息
        """
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_CONTINUOUS_ACQ)
        self._is_acquiring = True
        self.data["status"] = "Acquiring"
        return "continuous acquisition started"

    async def stop_acquisition(self) -> str:
        """停止采集

        Returns:
            str: 状态信息
        """
        self._send_cmd(self.CMD_STOP)
        self._is_acquiring = False
        await self._async_sleep(0.1)
        self._serial.reset_input_buffer()
        self.data["status"] = "Idle"
        return "acquisition stopped"

    async def read_frame(self) -> str:
        """在连续采集模式下读取一帧数据

        Returns:
            str: JSON 格式的像素数据数组
        """
        pixels = self._read_image_frame(timeout=5.0)
        if pixels:
            self._last_pixel_data = pixels
            self.data["value"] = json.dumps(pixels)
            return self.data["value"]
        return "[]"

    async def set_integration_time(self, time: float, unit: str = "ms") -> str:
        """设置积分时间

        Args:
            time: 积分时间值 (0-65535)
            unit: 时间单位, 'ms' 或 'us'

        Returns:
            str: 设置结果
        """
        time_int = int(time)
        # 先设置单位
        unit_val = 0x01 if unit == "us" else 0x00
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_INTEG_UNIT, unit_val)
        await self._async_sleep(0.1)

        # 设置积分时间
        data1 = (time_int >> 8) & 0xFF
        data2 = time_int & 0xFF
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_INTEGRATION_TIME, data1, data2)
        await self._async_sleep(0.1)

        self.data["integration_time"] = float(time_int)
        self.data["integration_time_unit"] = unit
        self.logger.info(f"积分时间已设置: {time_int} {unit}")
        return f"integration time set to {time_int} {unit}"

    async def set_gain(self, gain: float) -> str:
        """设置增益

        Args:
            gain: 增益值 (0-255)

        Returns:
            str: 设置结果
        """
        gain_int = max(0, min(255, int(gain)))
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_GAIN, gain_int)
        await self._async_sleep(0.1)
        self.data["gain"] = float(gain_int)
        return f"gain set to {gain_int}"

    async def set_offset(self, offset: float) -> str:
        """设置偏移

        Args:
            offset: 偏移值 (-255 到 255)

        Returns:
            str: 设置结果
        """
        offset_int = int(offset)
        data1 = abs(offset_int) & 0xFF
        data2 = 0x01 if offset_int >= 0 else 0x00
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_OFFSET, data1, data2)
        await self._async_sleep(0.1)
        self.data["offset"] = float(offset_int)
        return f"offset set to {offset_int}"

    async def set_smooth_level(self, level: float) -> str:
        """设置平滑等级

        Args:
            level: 平滑等级 (1-10)

        Returns:
            str: 设置结果
        """
        level_int = max(1, min(10, int(level)))
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_SET_SMOOTH, level_int)
        await self._async_sleep(0.1)
        self.data["smooth_level"] = float(level_int)
        return f"smooth level set to {level_int}"

    async def set_average_count(self, count: float) -> str:
        """设置平均次数

        Args:
            count: 平均次数 (1-255)

        Returns:
            str: 设置结果
        """
        count_int = max(1, min(255, int(count)))
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_AVG_COUNT, count_int)
        await self._async_sleep(0.1)
        self.data["average_count"] = float(count_int)
        return f"average count set to {count_int}"

    async def set_trigger_mode(self, mode: float) -> str:
        """设置触发模式

        Args:
            mode: 0=软触发, 1=外部连续脉冲(Trigger In2), 2=外部单脉冲(Trigger In1)

        Returns:
            str: 设置结果
        """
        mode_int = max(0, min(2, int(mode)))
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_TRIGGER_MODE, mode_int)
        await self._async_sleep(0.1)
        self.data["trigger_mode"] = float(mode_int)
        modes = {0: "software", 1: "ext_continuous", 2: "ext_single"}
        return f"trigger mode set to {modes.get(mode_int, str(mode_int))}"

    async def set_trigger_interval(self, interval: float) -> str:
        """设置触发间隔

        Args:
            interval: 触发间隔值 (0-65535)

        Returns:
            str: 设置结果
        """
        interval_int = int(interval)
        data1 = (interval_int >> 8) & 0xFF
        data2 = interval_int & 0xFF
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_TRIGGER_INTERVAL, data1, data2)
        await self._async_sleep(0.1)
        self.data["trigger_interval"] = float(interval_int)
        return f"trigger interval set to {interval_int}"

    async def set_analog_output(self, voltage: float) -> str:
        """设置模拟电压输出

        Args:
            voltage: 电压值 (0-5000 mV)

        Returns:
            str: 设置结果
        """
        voltage_int = max(0, min(5000, int(voltage)))
        data1 = (voltage_int >> 8) & 0xFF
        data2 = voltage_int & 0xFF
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_ANALOG_OUT, data1, data2)
        await self._async_sleep(0.1)
        return f"analog output set to {voltage_int} mV"

    async def set_trigger_out2(self, level: float) -> str:
        """设置 Trigger Out2 输出电平

        Args:
            level: 0=输出0V, 1=输出5V

        Returns:
            str: 设置结果
        """
        level_int = 1 if int(level) else 0
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_TRIGGER_OUT2, level_int)
        await self._async_sleep(0.1)
        return f"trigger out2 set to {'5V' if level_int else '0V'}"

    async def set_sync_output(self, enable: float) -> str:
        """设置同步信号输出

        Args:
            enable: 0=禁止, 1=使能

        Returns:
            str: 设置结果
        """
        enable_int = 1 if int(enable) else 0
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_SYNC_OUTPUT, enable_int)
        await self._async_sleep(0.1)
        return f"sync output {'enabled' if enable_int else 'disabled'}"

    async def save_parameters(self) -> str:
        """保存当前参数到 Flash

        Returns:
            str: 保存结果
        """
        self._serial.reset_input_buffer()
        self._send_cmd(self.CMD_SAVE_PARAMS, 0x01)
        await self._async_sleep(0.5)
        return "parameters saved to flash"

    async def read_correction_coefficients(self) -> str:
        """读取矫正系数 (3 组, 用于像素→波长转换)

        Returns:
            str: 矫正系数信息
        """
        coeffs = []
        for group in [0x01, 0x02, 0x03]:
            self._serial.reset_input_buffer()
            self._send_cmd(self.CMD_READ_CORRECTION, group)
            await self._async_sleep(0.3)

            # 读取 69 字节响应: [0x81][0x29][DATA1][0x40][DATA2(64bytes)][CRC]
            old_timeout = self._serial.timeout
            try:
                self._serial.timeout = 2.0
                # 找帧头
                header = b''
                deadline = time_module.time() + 2.0
                while time_module.time() < deadline:
                    b = self._serial.read(1)
                    if not b:
                        break
                    if b[0] == self.HEAD:
                        rest = self._serial.read(68)
                        if len(rest) == 68:
                            header = b + rest
                        break
                if len(header) == 69 and header[1] == 0x29:
                    coeff_data = header[4:68]
                    coeffs.append(coeff_data)
                    self.logger.info(f"矫正系数组 {group} 读取成功, {len(coeff_data)} 字节")
                else:
                    self.logger.warning(f"矫正系数组 {group} 读取失败")
                    coeffs.append(None)
            finally:
                self._serial.timeout = old_timeout

        # 解析系数: 每组 64 字节分成 4 段, 每段 16 字节作为一个字符串
        if all(c is not None for c in coeffs):
            f_coeffs = ["", "", "", ""]
            for c in coeffs:
                for i in range(4):
                    segment = c[i * 16:(i + 1) * 16]
                    f_coeffs[i] += segment.decode('ascii', errors='ignore')
            self._correction_coeffs = f_coeffs

            # 计算波长映射: wavelength = f[0]*i^3 + f[1]*i^2 + f[2]*i + f[3]
            try:
                f0 = float(self._correction_coeffs[0].strip('\x00').strip())
                f1 = float(self._correction_coeffs[1].strip('\x00').strip())
                f2 = float(self._correction_coeffs[2].strip('\x00').strip())
                f3 = float(self._correction_coeffs[3].strip('\x00').strip())
                self._wavelengths = [
                    f0 * i * i * i + f1 * i * i + f2 * i + f3
                    for i in range(self.PIXEL_COUNT)
                ]
                self.logger.info(f"波长映射已计算, 范围 [{self._wavelengths[0]:.2f} - {self._wavelengths[-1]:.2f}]")
                return f"correction coefficients loaded, wavelength range: {self._wavelengths[0]:.2f} - {self._wavelengths[-1]:.2f}"
            except (ValueError, IndexError) as e:
                self.logger.warning(f"波长计算失败: {e}")
                return f"correction coefficients loaded but wavelength calculation failed: {e}"
        else:
            return "failed to read correction coefficients"

    async def save_data_to_file(self, filename: str = "") -> str:
        """保存最近一次采集数据到本地 CSV 文件

        Args:
            filename: 文件名 (为空则自动生成带时间戳的文件名)

        Returns:
            str: 保存文件的完整路径
        """
        if not self._last_pixel_data:
            return "no data to save, please acquire first"

        save_dir = self.config.get("save_dir", "./cmos_data")
        os.makedirs(save_dir, exist_ok=True)

        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"cmos_{timestamp}.csv"

        filepath = os.path.join(save_dir, filename)

        try:
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                if self._wavelengths and len(self._wavelengths) == self.PIXEL_COUNT:
                    writer.writerow(["pixel_index", "raw_value", "wavelength"])
                    for i, val in enumerate(self._last_pixel_data):
                        writer.writerow([i, val, f"{self._wavelengths[i]:.4f}"])
                else:
                    writer.writerow(["pixel_index", "raw_value"])
                    for i, val in enumerate(self._last_pixel_data):
                        writer.writerow([i, val])

            self.logger.info(f"数据已保存: {filepath}")
            return f"saved to {os.path.abspath(filepath)}"
        except Exception as e:
            self.logger.error(f"保存失败: {e}")
            return f"save failed: {e}"
