"""
JYHSM 一体化温度变送器驱动
厂家：安徽久跃仪表有限公司
通信协议：Modbus RTU (RS485)
默认参数：9600, 8N1, 从站地址 1
寄存器映射：
  - 0x0000: 实时值*100 (有符号整型)
  - 0x0001: 实时值*10 (有符号整型)
  - 0x0002-0x0003: 浮点值 (ABCD格式, 32bit IEEE754)
  - 0x0108-0x0109: 偏移值 (浮点, ABCD格式)
  - 0x012C: 从站地址
  - 0x012D: 波特率 (0~7 对应 1200~115200)
  - 0x012E: 校验位 (0=None, 1=Odd, 2=Even)
  - 0x012F: 小数位数 (0~3)
  - 0x0130: 单位 (11=℃, 12=℉)
  - 0x0131: ADC速率 (10 或 40 Hz)

新增功能：
  - 温度阈值监控
  - 达到目标温度后触发提醒
"""

import logging
import struct
import time as time_module
import asyncio
from typing import Dict, Any, List

try:
    import serial
except ImportError:
    serial = None

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

try:
    from unilabos.registry.decorators import device, action, topic_config, not_action
except ImportError:
    def device(**kwargs):
        def wrapper(cls):
            return cls
        return wrapper
    def action(**kwargs):
        def wrapper(func):
            return func
        return wrapper
    def topic_config(**kwargs):
        def wrapper(func):
            return func
        return wrapper
    def not_action(func):
        return func


# ==================== Modbus RTU 工具函数 ====================

def _crc16_modbus(data: bytes) -> int:
    """计算 Modbus RTU CRC16 校验码"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _build_read_request(slave_addr: int, func_code: int, start_reg: int, reg_count: int) -> bytes:
    """构建 Modbus RTU 读请求帧"""
    frame = struct.pack(">BBH H", slave_addr, func_code, start_reg, reg_count)
    crc = _crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    return frame


def _build_write_single_request(slave_addr: int, reg_addr: int, value: int) -> bytes:
    """构建 Modbus RTU 写单个寄存器请求帧 (06H)"""
    frame = struct.pack(">BBH H", slave_addr, 0x06, reg_addr, value)
    crc = _crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    return frame


def _build_write_multiple_request(slave_addr: int, start_reg: int, values: List[int]) -> bytes:
    """构建 Modbus RTU 写多个寄存器请求帧 (10H)"""
    reg_count = len(values)
    byte_count = reg_count * 2
    frame = struct.pack(">BBH HB", slave_addr, 0x10, start_reg, reg_count, byte_count)
    for v in values:
        frame += struct.pack(">H", v)
    crc = _crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    return frame


def _validate_response(resp: bytes, slave_addr: int, func_code: int) -> bytes:
    """验证 Modbus RTU 响应帧"""
    if resp is None or len(resp) < 5:
        raise ValueError(f"响应帧过短或为空: {resp}")

    header = bytes([slave_addr, func_code])
    idx = resp.find(header)
    if idx < 0:
        err_header = bytes([slave_addr, func_code | 0x80])
        err_idx = resp.find(err_header)
        if err_idx >= 0:
            err_code = resp[err_idx + 2] if len(resp) > err_idx + 2 else 0xFF
            raise ValueError(f"Modbus 异常响应, 功能码 {hex(func_code)}, 异常码 {hex(err_code)}")
        raise ValueError(f"响应中未找到帧头 [{hex(slave_addr)}, {hex(func_code)}]: {resp.hex()}")

    frame = resp[idx:]
    if len(frame) < 5:
        raise ValueError(f"响应帧数据不完整: {frame.hex()}")

    if func_code in (0x03, 0x04):
        byte_count = frame[2]
        expected_len = 3 + byte_count + 2
        if len(frame) < expected_len:
            raise ValueError(f"响应帧长度不足: 期望{expected_len}, 实际{len(frame)}")
        payload = frame[:expected_len]
        crc_received = struct.unpack("<H", payload[-2:])[0]
        crc_calculated = _crc16_modbus(payload[:-2])
        if crc_received != crc_calculated:
            raise ValueError(f"CRC校验失败: 接收={hex(crc_received)}, 计算={hex(crc_calculated)}")
        return payload[3:-2]

    elif func_code in (0x06, 0x10):
        expected_len = 8
        if len(frame) < expected_len:
            raise ValueError(f"写响应帧长度不足: 期望{expected_len}, 实际{len(frame)}")
        payload = frame[:expected_len]
        crc_received = struct.unpack("<H", payload[-2:])[0]
        crc_calculated = _crc16_modbus(payload[:-2])
        if crc_received != crc_calculated:
            raise ValueError(f"CRC校验失败: 接收={hex(crc_received)}, 计算={hex(crc_calculated)}")
        return payload[2:-2]

    else:
        raise ValueError(f"不支持的功能码: {hex(func_code)}")


def _decode_float_abcd(high_word: int, low_word: int) -> float:
    """将两个 16bit 寄存器值解码为 IEEE754 浮点数 (ABCD 大端格式)"""
    raw = struct.pack(">HH", high_word, low_word)
    return struct.unpack(">f", raw)[0]


def _encode_float_abcd(value: float) -> tuple:
    """将浮点数编码为两个 16bit 寄存器值 (ABCD 大端格式)"""
    raw = struct.pack(">f", value)
    high_word, low_word = struct.unpack(">HH", raw)
    return high_word, low_word


# ==================== 波特率映射 ====================

BAUDRATE_MAP = {
    0: 1200.0,
    1: 2400.0,
    2: 4800.0,
    3: 9600.0,
    4: 19200.0,
    5: 38400.0,
    6: 57600.0,
    7: 115200.0,
}
BAUDRATE_REVERSE_MAP = {v: k for k, v in BAUDRATE_MAP.items()}

UNIT_MAP = {
    11.0: "℃",
    12.0: "℉",
}


# ==================== 设备驱动类 ====================

@device(
    id="jyhsm_temperature_transmitter",
    category=["temperature"],
    description="JYHSM 一体化温度变送器，Modbus RTU",
    display_name="JYHSM 温度变送器"
)
class JyhsmTemperatureTransmitter:
    """
    JYHSM 一体化温度变送器 Modbus RTU 驱动
    (所有数值类型均对齐为 float 以支持 Uni-Lab-OS 框架)
    
    新增功能：
    - 温度阈值监控
    - 达到目标温度后触发提醒
    """

    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")

        self.device_id = device_id or "jyhsm_temp_1"
        self.config = config or {}
        self.logger = logging.getLogger(f"JyhsmTemp.{self.device_id}")

        # 串口配置
        self._port = self.config.get("port", "COM4")
        self._baudrate = float(self.config.get("baudrate", 9600.0))
        self._slave_address = int(self.config.get("slave_address", 1))
        self._timeout = float(self.config.get("timeout", 1.0))

        # 串口对象
        self._serial = None

        # 温度监控相关（内部状态）
        self._monitoring_task = None
        self._target_temperature_internal = -999.0  # 内部使用，表示未设置
        self._tolerance_internal = 0.5
        self._monitoring_internal = False
        self._alarm_triggered_internal = False
        self._alarm_status_internal = "Idle"
        self._monitor_interval = 1.0  # 监控间隔（秒）

        # 硬约束：所有数值属性必须对齐为 float，不能用 None
        # target_temperature 用 -999.0 表示未设置
        self.data = {
            "status": "Idle",
            "temperature": 0.0,
            "target_temperature": -999.0,  # 用特殊值表示未设置
            "tolerance": 0.5,
            "monitoring": False,
            "alarm_triggered": False,
            "alarm_status": "Idle",
            "offset": 0.0,
            "unit": "℃",
            "slave_address": float(self._slave_address),
            "baudrate": float(self._baudrate),
            "level": False,
            "rssi": 0.0,
        }

    @not_action
    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    # ==================== 属性（全部使用简单类型，不用 Optional）====================

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def temperature(self) -> float:
        return float(self.data.get("temperature", 0.0))

    @property
    def target_temperature(self) -> float:
        """目标温度阈值，-999.0 表示未设置"""
        return float(self.data.get("target_temperature", -999.0))

    @property
    def tolerance(self) -> float:
        """允许误差范围"""
        return float(self.data.get("tolerance", 0.5))

    @property
    def monitoring(self) -> bool:
        """是否正在监控"""
        return bool(self.data.get("monitoring", False))

    @property
    def alarm_triggered(self) -> bool:
        """是否已触发提醒"""
        return bool(self.data.get("alarm_triggered", False))

    @property
    def alarm_status(self) -> str:
        """告警状态: Idle/Monitoring/Reached/Timeout/Error"""
        return str(self.data.get("alarm_status", "Idle"))

    @property
    def offset(self) -> float:
        return float(self.data.get("offset", 0.0))

    @property
    def unit(self) -> str:
        return str(self.data.get("unit", "℃"))

    @property
    def slave_address(self) -> float:
        return float(self.data.get("slave_address", 1.0))

    @property
    def baudrate(self) -> float:
        return float(self.data.get("baudrate", 9600.0))

    @property
    def level(self) -> bool:
        return bool(self.data.get("level", False))

    @property
    def rssi(self) -> float:
        return float(self.data.get("rssi", 0.0))

    # ==================== 串口底层通信 ====================

    def _open_serial(self):
        if serial is None:
            raise ImportError("pyserial 未安装")
        if self._serial is not None and self._serial.is_open:
            return
        self._serial = serial.Serial(
            port=self._port,
            baudrate=int(self._baudrate),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
        )

    def _close_serial(self):
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def _send_and_receive(self, request: bytes, expected_resp_len: int = 64) -> bytes:
        if self._serial is None or not self._serial.is_open:
            raise ConnectionError("串口未打开")
        self._serial.reset_input_buffer()
        self._serial.write(request)
        time_module.sleep(0.05)
        resp = self._serial.read(expected_resp_len)
        return resp

    def _read_registers(self, start_reg: int, reg_count: int, func_code: int = 0x03) -> bytes:
        request = _build_read_request(self._slave_address, func_code, start_reg, reg_count)
        resp = self._send_and_receive(request, 5 + reg_count * 2 + 10)
        return _validate_response(resp, self._slave_address, func_code)

    def _write_single_register(self, reg_addr: int, value: int):
        request = _build_write_single_request(self._slave_address, reg_addr, value)
        resp = self._send_and_receive(request, 20)
        _validate_response(resp, self._slave_address, 0x06)

    def _write_multiple_registers(self, start_reg: int, values: List[int]):
        request = _build_write_multiple_request(self._slave_address, start_reg, values)
        resp = self._send_and_receive(request, 20)
        _validate_response(resp, self._slave_address, 0x10)

    # ==================== 异步动作方法 ====================

    @action(description="初始化设备")
    async def initialize(self) -> bool:
        try:
            self.data["status"] = "Busy"
            self._open_serial()
            # 简单验证连接，读取实时温度
            temp = _decode_float_abcd(*struct.unpack(">HH", self._read_registers(0x0002, 2)))
            self.data["temperature"] = float(temp)
            self.data["status"] = "Idle"
            return True
        except Exception as e:
            self.data["status"] = "Error"
            self.logger.error(f"初始化失败: {e}")
            return False

    @action(description="清理资源")
    async def cleanup(self) -> bool:
        # 停止监控任务
        if self._monitoring_task is not None:
            self._monitoring_task.cancel()
            try:
                await self._monitoring_task
            except asyncio.CancelledError:
                pass
            self._monitoring_task = None
        
        self._close_serial()
        self.data["status"] = "Offline"
        self.data["monitoring"] = False
        return True

    async def read_temperature(self) -> float:
        """读取当前温度"""
        try:
            self.data["status"] = "Busy"
            data = self._read_registers(0x0002, 2)
            temp = _decode_float_abcd(struct.unpack(">H", data[0:2])[0], struct.unpack(">H", data[2:4])[0])
            self.data["temperature"] = float(temp)
            self.data["status"] = "Idle"
            return float(temp)
        except Exception as e:
            self.data["status"] = "Error"
            raise

    # ==================== 温度监控功能（参数全部使用 float，不用 Optional）====================

    async def set_target_temperature(self, target: float) -> bool:
        """设置目标温度阈值"""
        try:
            self.data["target_temperature"] = float(target)
            self._target_temperature_internal = float(target)
            self.logger.info(f"目标温度已设置为: {target}°C")
            return True
        except Exception as e:
            self.logger.error(f"设置目标温度失败: {e}")
            return False

    async def set_tolerance(self, tolerance: float) -> bool:
        """设置允许误差范围"""
        try:
            self.data["tolerance"] = float(tolerance)
            self._tolerance_internal = float(tolerance)
            self.logger.info(f"误差范围已设置为: {tolerance}°C")
            return True
        except Exception as e:
            self.logger.error(f"设置误差范围失败: {e}")
            return False

    async def _monitor_temperature_loop(self, target: float, tolerance: float, timeout: float):
        """温度监控循环"""
        start_time = time_module.time()
        self.data["alarm_status"] = "Monitoring"
        
        while self._monitoring_internal:
            try:
                # 读取当前温度
                current_temp = await self.read_temperature()
                self.logger.debug(f"当前温度: {current_temp}°C, 目标: {target}°C, 误差: {tolerance}°C")
                
                # 检查是否达到目标温度
                if abs(current_temp - target) <= tolerance:
                    self.data["alarm_triggered"] = True
                    self.data["alarm_status"] = "Reached"
                    self._alarm_triggered_internal = True
                    self._monitoring_internal = False
                    self.logger.info(f"温度已达到目标! 当前: {current_temp}°C, 目标: {target}°C")
                    
                    # 触发回调（如果有 ROS 节点）
                    if self._ros_node is not None:
                        self._ros_node.get_logger().info(f"Temperature reached target: {current_temp}°C")
                    return
                
                # 检查超时（timeout > 0 表示有限制）
                if timeout > 0.0:
                    elapsed = time_module.time() - start_time
                    if elapsed > timeout:
                        self.data["alarm_status"] = "Timeout"
                        self._monitoring_internal = False
                        self.logger.warning(f"温度监控超时: {timeout}秒")
                        return
                
                # 等待下一次检查
                if self._ros_node is not None:
                    await self._ros_node.sleep(self._monitor_interval)
                else:
                    await asyncio.sleep(self._monitor_interval)
                    
            except asyncio.CancelledError:
                self.data["alarm_status"] = "Cancelled"
                self.logger.info("温度监控已取消")
                raise
            except Exception as e:
                self.data["alarm_status"] = "Error"
                self.logger.error(f"温度监控错误: {e}")
                self._monitoring_internal = False
                raise

    async def start_temperature_monitoring(self, target: float, tolerance: float = 0.5, timeout: float = 0.0) -> bool:
        """
        开始后台监控温度，达到目标后触发提醒
        
        Args:
            target: 目标温度 (°C)
            tolerance: 允许误差范围 (°C)，默认 0.5
            timeout: 超时时间（秒），0.0 表示不限制
        
        Returns:
            bool: 是否成功启动监控
        """
        try:
            # 如果已有监控任务在运行，先停止
            if self._monitoring_task is not None and not self._monitoring_task.done():
                self._monitoring_task.cancel()
                try:
                    await self._monitoring_task
                except asyncio.CancelledError:
                    pass
            
            # 重置状态
            self._target_temperature_internal = float(target)
            self._tolerance_internal = float(tolerance)
            self._monitoring_internal = True
            self._alarm_triggered_internal = False
            
            # 更新 data 字典
            self.data["target_temperature"] = float(target)
            self.data["tolerance"] = float(tolerance)
            self.data["monitoring"] = True
            self.data["alarm_triggered"] = False
            self.data["alarm_status"] = "Idle"
            
            # 启动后台监控任务
            self._monitoring_task = asyncio.create_task(
                self._monitor_temperature_loop(target, tolerance, timeout)
            )
            
            self.logger.info(f"开始监控温度: 目标 {target}°C, 误差 ±{tolerance}°C")
            return True
            
        except Exception as e:
            self.data["alarm_status"] = "Error"
            self.logger.error(f"启动温度监控失败: {e}")
            return False

    async def stop_temperature_monitoring(self) -> bool:
        """停止温度监控"""
        try:
            self._monitoring_internal = False
            
            if self._monitoring_task is not None and not self._monitoring_task.done():
                self._monitoring_task.cancel()
                try:
                    await self._monitoring_task
                except asyncio.CancelledError:
                    pass
                self._monitoring_task = None
            
            self.data["monitoring"] = False
            if self.data["alarm_status"] == "Monitoring":
                self.data["alarm_status"] = "Cancelled"
            
            self.logger.info("温度监控已停止")
            return True
            
        except Exception as e:
            self.logger.error(f"停止温度监控失败: {e}")
            return False

    async def wait_for_temperature(self, target: float, tolerance: float = 0.5, timeout: float = 300.0) -> bool:
        """
        阻塞等待温度达到目标值（用于工作流串联）
        
        Args:
            target: 目标温度 (°C)
            tolerance: 允许误差范围 (°C)，默认 0.5
            timeout: 超时时间（秒），默认 300 秒
        
        Returns:
            bool: 是否成功达到目标温度（False 表示超时或错误）
        """
        try:
            self.data["status"] = "Busy"
            
            # 重置状态
            self._target_temperature_internal = float(target)
            self._tolerance_internal = float(tolerance)
            self._monitoring_internal = True
            self._alarm_triggered_internal = False
            
            # 更新 data 字典
            self.data["target_temperature"] = float(target)
            self.data["tolerance"] = float(tolerance)
            self.data["monitoring"] = True
            self.data["alarm_triggered"] = False
            self.data["alarm_status"] = "Monitoring"
            
            start_time = time_module.time()
            
            while self._monitoring_internal:
                # 读取当前温度
                current_temp = await self.read_temperature()
                
                # 检查是否达到目标温度
                if abs(current_temp - target) <= tolerance:
                    self.data["alarm_triggered"] = True
                    self.data["alarm_status"] = "Reached"
                    self._monitoring_internal = False
                    self.data["status"] = "Idle"
                    self.logger.info(f"温度已达到目标! 当前: {current_temp}°C, 目标: {target}°C")
                    return True
                
                # 检查超时
                if timeout > 0.0:
                    elapsed = time_module.time() - start_time
                    if elapsed > timeout:
                        self.data["alarm_status"] = "Timeout"
                        self._monitoring_internal = False
                        self.data["status"] = "Idle"
                        self.logger.warning(f"等待温度超时: {timeout}秒")
                        return False
                
                # 等待下一次检查
                if self._ros_node is not None:
                    await self._ros_node.sleep(self._monitor_interval)
                else:
                    await asyncio.sleep(self._monitor_interval)
            
            self.data["status"] = "Idle"
            return False
            
        except asyncio.CancelledError:
            self.data["alarm_status"] = "Cancelled"
            self.data["status"] = "Idle"
            raise
        except Exception as e:
            self.data["alarm_status"] = "Error"
            self.data["status"] = "Error"
            self.logger.error(f"等待温度失败: {e}")
            return False

    # ==================== 其他动作方法 ====================

    async def set_offset(self, offset: float) -> bool:
        try:
            self.data["status"] = "Busy"
            h, l = _encode_float_abcd(float(offset))
            self._write_multiple_registers(0x0108, [h, l])
            self.data["offset"] = float(offset)
            self.data["status"] = "Idle"
            return True
        except Exception:
            self.data["status"] = "Error"
            return False

    async def set_unit(self, unit: float) -> bool:
        try:
            self.data["status"] = "Busy"
            self._write_single_register(0x0130, int(unit))
            self.data["unit"] = UNIT_MAP.get(float(unit), "℃")
            self.data["status"] = "Idle"
            return True
        except Exception:
            self.data["status"] = "Error"
            return False

    async def set_slave_address(self, address: float) -> bool:
        try:
            self.data["status"] = "Busy"
            self._write_single_register(0x012C, int(address))
            self._slave_address = int(address)
            self.data["slave_address"] = float(address)
            self.data["status"] = "Idle"
            return True
        except Exception:
            self.data["status"] = "Error"
            return False

    async def set_baudrate(self, baudrate: float) -> bool:
        try:
            self.data["status"] = "Busy"
            code = BAUDRATE_REVERSE_MAP.get(float(baudrate))
            if code is None: return False
            self._write_single_register(0x012D, int(code))
            self._close_serial()
            self._baudrate = float(baudrate)
            self.data["baudrate"] = float(baudrate)
            self.data["status"] = "Idle"
            return True
        except Exception:
            self.data["status"] = "Error"
            return False