"""
Runze SY-03B Ceramic Injection Pump Driver (T-08 Valve Head)
润泽 SY-03B 陶瓷注射泵驱动（T-08八通分配阀）

Model: ZSB-SY03B-T08
Syringe: 25mL (6000 steps full stroke)
Valve: T-08 (8-port distribution valve, C-port connects to 1-8)
Protocol: ASCII DT Format via RS232/RS485

Reference: SY-03B陶瓷阀芯(ASCII)V2.4 说明书
"""

import logging
import asyncio
import serial
import time as time_module
from typing import Dict, Any, Optional

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


@device(
    id="runze_sy03b_t08",
    category=["pump"],
    description="润泽 SY-03B 陶瓷注射泵，带 T-08 八通分配阀",
    display_name="润泽注射泵"
)
class RunzeSY03BT08:
    """
    Runze SY-03B Ceramic Injection Pump Driver
    25mL Syringe: 6000 steps full stroke = 240 steps/mL
    """
    
    _ros_node: "BaseROS2DeviceNode"
    
    # 进样器规格配置 (mL -> 步数/mL)
    SYRINGE_STEPS_PER_ML = {
        25.0: 240,  # 25mL ← 用户规格
    }
    
    # 状态码映射
    STATUS_MAP = {
        '0': "Idle", '1': "Busy", '2': "Busy", '3': "Busy",
        '4': "Busy", '5': "Busy", '6': "Idle", '7': "Idle",
        '8': "Idle", '9': "Idle",
    }
    
    # 错误码映射
    ERROR_CODES = {
        '?0': "Invalid Command", '?1': "Not Initialize",
        '?2': "Forbidden in Current State", '?3': "Valve Error",
        '?4': "Syringe Error", '?5': "Buffer Overflow",
        '?6': "Invalid Address",
    }
    
    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        
        self.device_id = device_id or "runze_sy03b_t08"
        self.config = config or {}
        self.logger = logging.getLogger(f"RunzeSY03BT08.{self.device_id}")
        
        # 串口配置
        self.port = self.config.get('port', 'COM4')
        self.baudrate = self.config.get('baudrate', 9600)
        self.address = self.config.get('address', 0)
        self.ascii_address = str(self.address + 1)
        
        # 进样器规格
        self.syringe_volume = self.config.get('syringe_volume', 25.0)
        self.steps_per_ml = self.SYRINGE_STEPS_PER_ML.get(self.syringe_volume, 240)
        self.max_steps = int(self.syringe_volume * self.steps_per_ml)
        
        # 串口连接
        self.serial: Optional[serial.Serial] = None
        self._velocity = 600
        
        # 数据字典 (预填充所有属性)
        self.data = {
            "status": "Offline",
            "valve_position": "0",
            "position": 0.0,
            "max_velocity": 0.5,
            "mode": 0,
            "plunger_position": "0",
            "velocity_grade": "600",
            "velocity_init": "600",
            "velocity_end": "600",
        }
        self._initialized = False
    
    @not_action
    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node
    
    def _ml_to_steps(self, ml: float) -> int:
        return int(ml * self.steps_per_ml)
    
    def _steps_to_ml(self, steps: int) -> float:
        return steps / self.steps_per_ml
    
    def _velocity_to_ml_s(self, velocity: int) -> float:
        return velocity / self.steps_per_ml * 0.5
    
    def _connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                return True
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1.0, write_timeout=1.0
            )
            time_module.sleep(0.1)
            self.logger.info(f"Serial port {self.port} opened")
            return True
        except Exception as e:
            self.logger.error(f"Failed to open {self.port}: {e}")
            return False
    
    def _disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def _send_command(self, command: str, wait_response: bool = True) -> str:
        if not self._connect():
            return ""
        frame = f"/{self.ascii_address}{command}\r"
        try:
            if self.serial.in_waiting > 0:
                self.serial.read(self.serial.in_waiting)
            self.serial.write(frame.encode('ascii'))
            self.logger.debug(f"Sent: {frame.strip()}")
            if not wait_response:
                return ""
            response = b''
            start_time = time_module.time()
            while time_module.time() - start_time < 5.0:
                if self.serial.in_waiting > 0:
                    response += self.serial.read(1)
                    if response.endswith(b'\r\n'):
                        break
                time_module.sleep(0.01)
            return response.decode('ascii', errors='ignore')
        except Exception as e:
            self.logger.error(f"Communication error: {e}")
            return ""
    
    def _parse_response(self, response: str) -> tuple:
        if not response:
            return ("error", "")
        response = response.replace('\x03', '').replace('\r', '').replace('\n', '').strip()
        if response.startswith('?'):
            error_code = response[:2]
            self.logger.error(f"Device error: {error_code}")
            return ("error", error_code)
        if response.startswith('/'):
            response = response[1:]
        if len(response) >= 1:
            return (response[0], response[1:] if len(response) > 1 else "")
        return ("unknown", response)
    
    def _wait_for_idle(self, timeout: float = 30.0) -> bool:
        start_time = time_module.time()
        while time_module.time() - start_time < timeout:
            response = self._send_command("Q")
            status, _ = self._parse_response(response)
            if status == '0':
                return True
            time_module.sleep(0.1)
        return False
    
    @action(description="初始化设备")
    async def initialize(self) -> bool:
        self.logger.info("Initializing SY-03B pump...")
        if not self._connect():
            self.data["status"] = "Offline"
            return False
        try:
            response = self._send_command("ZR")
            status, _ = self._parse_response(response)
            if status == 'error':
                self.data["status"] = "Error"
                return False
            start_time = time_module.time()
            while time_module.time() - start_time < 60.0:
                response = self._send_command("Q")
                status, _ = self._parse_response(response)
                if status in ['0', '6', '7', '8', '9']:
                    self._initialized = True
                    self.data["status"] = "Idle"
                    self.data["valve_position"] = "0"
                    self.data["position"] = 0.0
                    self.logger.info("Pump initialized")
                    return True
                if self._ros_node:
                    await self._ros_node.sleep(0.2)
                else:
                    await asyncio.sleep(0.2)
            return False
        except Exception as e:
            self.logger.error(f"Initialization failed: {e}")
            return False
    
    @action(description="清理资源")
    async def cleanup(self) -> bool:
        self._disconnect()
        self.data["status"] = "Offline"
        self._initialized = False
        return True
    
    # ==================== 属性 ====================
    @property
    def status(self) -> str:
        return self.data.get("status", "Offline")
    
    @property
    def valve_position(self) -> str:
        return self.data.get("valve_position", "0")
    
    @property
    def position(self) -> float:
        return self.data.get("position", 0.0)
    
    @property
    def max_velocity(self) -> float:
        return self.data.get("max_velocity", 0.5)
    
    @property
    def mode(self) -> int:
        return self.data.get("mode", 0)
    
    @property
    def plunger_position(self) -> str:
        return self.data.get("plunger_position", "0")
    
    @property
    def velocity_grade(self) -> str:
        return self.data.get("velocity_grade", "600")
    
    @property
    def velocity_init(self) -> str:
        return self.data.get("velocity_init", "600")
    
    @property
    def velocity_end(self) -> str:
        return self.data.get("velocity_end", "600")
    
    # ==================== 动作方法 ====================
    def set_valve_position(self, position: str) -> bool:
        if not self._initialized:
            return False
        try:
            port = int(position)
            if port < 1 or port > 8:
                return False
        except ValueError:
            return False
        self.data["status"] = "Busy"
        response = self._send_command(f"I{port}R")
        status, _ = self._parse_response(response)
        if status == 'error':
            self.data["status"] = "Error"
            return False
        if self._wait_for_idle(timeout=10.0):
            self.data["valve_position"] = position
            self.data["status"] = "Idle"
            return True
        self.data["status"] = "Error"
        return False
    
    def set_position(self, position: float, max_velocity: float = None) -> bool:
        if not self._initialized:
            return False
        if max_velocity is not None:
            self.set_max_velocity(max_velocity)
        self.data["status"] = "Busy"
        steps = max(0, min(self.max_steps, self._ml_to_steps(position)))
        response = self._send_command(f"A{steps}R")
        status, _ = self._parse_response(response)
        if status == 'error':
            self.data["status"] = "Error"
            return False
        timeout = 30.0 + abs(steps) / 100
        if self._wait_for_idle(timeout=timeout):
            self.data["position"] = position
            self.data["plunger_position"] = str(steps)
            self.data["status"] = "Idle"
            return True
        self.data["status"] = "Error"
        return False
    
    def pull_plunger(self, volume: float) -> bool:
        if not self._initialized:
            return False
        self.data["status"] = "Busy"
        steps = self._ml_to_steps(volume)
        response = self._send_command(f"P{steps}R")
        status, _ = self._parse_response(response)
        if status == 'error':
            self.data["status"] = "Error"
            return False
        timeout = 10.0 + steps / 100
        if self._wait_for_idle(timeout=timeout):
            current = int(self.data.get("plunger_position", "0"))
            new_steps = current + steps
            self.data["plunger_position"] = str(new_steps)
            self.data["position"] = self._steps_to_ml(new_steps)
            self.data["status"] = "Idle"
            return True
        self.data["status"] = "Error"
        return False
    
    def push_plunger(self, volume: float) -> bool:
        if not self._initialized:
            return False
        self.data["status"] = "Busy"
        steps = self._ml_to_steps(volume)
        response = self._send_command(f"D{steps}R")
        status, _ = self._parse_response(response)
        if status == 'error':
            self.data["status"] = "Error"
            return False
        timeout = 10.0 + steps / 100
        if self._wait_for_idle(timeout=timeout):
            current = int(self.data.get("plunger_position", "0"))
            new_steps = max(0, current - steps)
            self.data["plunger_position"] = str(new_steps)
            self.data["position"] = self._steps_to_ml(new_steps)
            self.data["status"] = "Idle"
            return True
        self.data["status"] = "Error"
        return False
    
    def set_max_velocity(self, velocity: float) -> bool:
        grade = max(10, min(6000, int(velocity * self.steps_per_ml * 2)))
        response = self._send_command(f"V{grade}R")
        status, _ = self._parse_response(response)
        if status != 'error':
            self._velocity = grade
            self.data["max_velocity"] = velocity
            self.data["velocity_grade"] = str(grade)
            return True
        return False
    
    def set_velocity_grade(self, velocity: str) -> bool:
        try:
            grade = max(10, min(6000, int(velocity)))
        except ValueError:
            return False
        response = self._send_command(f"V{grade}R")
        status, _ = self._parse_response(response)
        if status != 'error':
            self._velocity = grade
            self.data["max_velocity"] = self._velocity_to_ml_s(grade)
            self.data["velocity_grade"] = str(grade)
            return True
        return False
    
    def stop_operation(self) -> bool:
        response = self._send_command("TR")
        status, _ = self._parse_response(response)
        self.data["status"] = "Idle"
        return status != 'error'
    
    # 兼容接口
    def open(self) -> bool:
        return True
    
    def close(self) -> bool:
        return True
    
    def is_open(self) -> bool:
        return True
    
    def is_closed(self) -> bool:
        return False


DEVICE_CLASS = RunzeSY03BT08