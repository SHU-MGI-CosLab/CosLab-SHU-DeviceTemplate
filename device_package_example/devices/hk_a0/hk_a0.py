import logging
from typing import Dict, Any, List, Optional

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

# 兼容不同版本 pymodbus
try:
    from pymodbus.client import ModbusSerialClient  # 3.x
except Exception:
    try:
        from pymodbus.client.sync import ModbusSerialClient  # 2.5.x
    except Exception:
        ModbusSerialClient = None

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


@device(
    id="hk_a0",
    category=["io_module"],
    description="HK-A0 模拟输出模块，6通道，Modbus RTU",
    display_name="HK-A0 输出模块"
)
class HKA0:
    """
    Huaikong Electronic HK-A0 Analog Output Module Driver (RS485 Modbus RTU)
    Supports 6 output channels (AO1-AO6), 12-bit resolution.
    Value scaling: Physical Value * 1000 = Register Value.
    """
    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')

        self.device_id = device_id or "hk_a0_module_1"
        self.config = config or {}

        # 日志配置
        self.logger = logging.getLogger(f"HKA0.{self.device_id}")

        # Modbus 连接参数
        self.port = self.config.get("port", "/dev/ttyUSB0")
        self.baudrate = self.config.get("baudrate", 9600)
        self.slave_address = self.config.get("slave_address", 1)
        self.channel_count = self.config.get("channel_count", 6)
        self.timeout = self.config.get("timeout", 1.0)

        # Modbus 客户端
        self.client: Optional[ModbusSerialClient] = None

        # 寄存器地址映射 (根据 HK-A0 手册)
        self.REG_OUTPUT_BASE = 0x0009  # 输出寄存器起始地址

        # Pre-fill self.data
        self.data = {
            "status": "Idle",
            "outputs": [0.0] * self.channel_count,
            "last_error": ""
        }

    @not_action
    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    @action(description="初始化设备")
    async def initialize(self) -> bool:
        """初始化 Modbus 连接"""
        try:
            if ModbusSerialClient is None:
                self.logger.error("pymodbus not installed")
                self.data["status"] = "Error"
                self.data["last_error"] = "pymodbus not installed"
                return False

            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity='N',
                stopbits=1,
                bytesize=8
            )

            if not self.client.connect():
                self.logger.error(f"Failed to connect to {self.port}")
                self.data["status"] = "Error"
                self.data["last_error"] = "Connection failed"
                return False

            self.data["status"] = "Idle"
            self.logger.info(f"HK-A0 initialized at {self.port}, Slave Addr: {self.slave_address}")
            return True

        except Exception as e:
            self.logger.error(f"Initialization error: {e}")
            self.data["status"] = "Error"
            self.data["last_error"] = str(e)
            return False

    @action(description="设置输出值")
    async def set_output(self, channel: int, value: float) -> bool:
        """
        设置输出值

        Args:
            channel[通道号]: 通道编号 (1-6)
            value[输出值]: 物理值 (0.0-5.0 V)
        """
        if not (1 <= channel <= self.channel_count):
            self.logger.error(f"Invalid channel: {channel}")
            self.data["last_error"] = f"Invalid channel: {channel}"
            return False

        if not (0.0 <= value <= 5.0):
            self.logger.error(f"Value out of range: {value}V (must be 0.0-5.0)")
            self.data["last_error"] = f"Value out of range: {value}V"
            return False

        try:
            # 缩放: 1.000V -> 1000
            raw_value = int(value * 1000)
            reg_addr = self.REG_OUTPUT_BASE + channel

            # 写入单个保持寄存器
            result = self.client.write_register(
                address=reg_addr,
                value=raw_value,
                slave=self.slave_address
            )

            if result.isError():
                self.logger.error(f"Modbus write error for Ch{channel}")
                self.data["last_error"] = "Modbus write error"
                return False

            self.data["outputs"][channel - 1] = value
            self.logger.info(f"Set HK-A0 Ch{channel} to {value}V (Raw: {raw_value})")
            return True

        except Exception as e:
            self.logger.error(f"Error setting output: {e}")
            self.data["last_error"] = str(e)
            return False

    @action(description="停止所有输出")
    async def stop_all(self) -> bool:
        """停止所有通道输出（设置为 0V）"""
        success = True
        for i in range(1, self.channel_count + 1):
            if not await self.set_output(i, 0.0):
                success = False
        return success

    @action(description="清理资源")
    async def cleanup(self) -> bool:
        """清理资源，关闭 Modbus 连接"""
        try:
            if self.client and self.client.is_socket_open():
                self.client.close()
            self.data["status"] = "Offline"
            self.logger.info("HK-A0 connection closed")
            return True
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}")
            return False

    @action(description="读取所有输出值")
    async def read_outputs(self) -> List[float]:
        """读取所有通道的当前输出值"""
        try:
            result = self.client.read_holding_registers(
                address=self.REG_OUTPUT_BASE + 1,
                count=self.channel_count,
                slave=self.slave_address
            )

            if result.isError():
                self.logger.error("Failed to read outputs")
                return self.data["outputs"]

            # 转换为物理值
            outputs = [reg / 1000.0 for reg in result.registers]
            self.data["outputs"] = outputs
            return outputs

        except Exception as e:
            self.logger.error(f"Error reading outputs: {e}")
            return self.data["outputs"]

    @property
    @topic_config()
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    @topic_config()
    def outputs(self) -> List[float]:
        return self.data.get("outputs", [0.0]*self.channel_count)
