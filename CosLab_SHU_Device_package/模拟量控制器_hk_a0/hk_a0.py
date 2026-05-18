import logging
from typing import Dict, Any, List

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

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
        self.logger = logging.getLogger(f"HKA0.{self.device_id}")
        
        # Core Parameters
        self.slave_address = self.config.get("slave_address", 1)
        self.channel_count = self.config.get("channel_count", 6)
        
        # Pre-fill self.data
        self.data = {
            "status": "Idle",
            "outputs": [0.0] * self.channel_count,
            "last_error": ""
        }

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    async def initialize(self) -> bool:
        """Initialization logic."""
        self.data["status"] = "Idle"
        self.logger.info(f"HK-A0 initialized at Slave Addr: {self.slave_address}")
        return True

    async def set_output(self, channel: int, value: float) -> bool:
        """
        Sets output value.
        :param channel: Channel number (1-6)
        :param value: Physical value (0.0-5.0 V)
        """
        if not (1 <= channel <= self.channel_count):
            self.logger.error(f"Invalid channel: {channel}")
            return False

        # Scaling: 1.000V -> 1000
        raw_value = int(value * 1000)
        reg_addr = 0x0009 + channel 
        
        self.logger.info(f"Setting HK-A0 Ch {channel} to {value}V (Raw: {raw_value})")
        
        # Logic to send Modbus command would go here
        self.data["outputs"][channel - 1] = value
        return True

    async def stop_all(self) -> bool:
        for i in range(1, self.channel_count + 1):
            await self.set_output(i, 0.0)
        return True

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def outputs(self) -> List[float]:
        return self.data.get("outputs", [0.0]*self.channel_count)
