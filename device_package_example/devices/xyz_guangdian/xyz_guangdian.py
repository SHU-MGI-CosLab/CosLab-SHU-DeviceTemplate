"""
XYZ光电工作台设备驱动 - 修复版
用于控制XYZ三轴运动平台和推杆装置
修复了Modbus通信错误和异步编程问题
"""

import logging
import time as time_module
from typing import Dict, Any, Optional
import struct

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
    id="xyz_guangdian",
    category=["motion"],
    description="XYZ 三维运动平台，支持三轴运动和推杆控制",
    display_name="XYZ 三维平台"
)
class XYZGuangdian:
    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        self.device_id = device_id or "unknown_device"
        self.config = config or {}
        self.logger = logging.getLogger(f"XYZGuangdian.{self.device_id}")
        
        # Modbus通信配置
        self.modbus_client = None
        self.port = self.config.get('port', 'COM35')
        self.baudrate = self.config.get('baudrate', 9600)
        self.timeout = self.config.get('timeout', 2.0)
        self.retry_count = self.config.get('retry_count', 3)
        self.retry_delay = self.config.get('retry_delay', 0.1)
        
        # 设备地址映射
        self.addr_mapping = {
            'x': 0x02,      # X轴地址
            'y': 0x03,      # Y轴地址
            'z': 0x01,      # Z轴地址
            'push_rod': 0x04  # 推杆地址
        }
        
        # 寄存器地址映射
        self.reg_mapping = {
            'enable': 0x07,     # 使能寄存器
            'stop': 0x08,       # 停止寄存器
            'clamp': 0x07,      # 夹紧寄存器
            'release': 0x08,    # 释放寄存器
            'position': 0x03,   # 位置读取寄存器
            'target_pos': 0x04, # 目标位置寄存器
            'speed': 0x05,      # 速度寄存器
            'accel': 0x06,      # 加速度寄存器
            'home': 0x02        # 回零寄存器
        }
        
        # 状态数据初始化
        self.data = {
            "status": "Idle",
            "position_x": 0.0,
            "position_y": 0.0,
            "position_z": 0.0,
            "push_rod_status": "released",  # 推杆状态：clamped/released
            "is_enabled": False,
            "is_homed": False,
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_z": 0.0,
            "temperature": 25.0,
            "error_code": 0
        }

    @not_action
    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        """与ROS节点关联"""
        self._ros_node = ros_node

    @action(description="初始化设备")
    async def initialize(self) -> bool:
        """初始化设备"""
        try:
            self.logger.info(f"初始化XYZ光电工作台 {self.device_id}")
            
            # 导入pymodbus
            try:
                from pymodbus.client import ModbusSerialClient
                from pymodbus.exceptions import ModbusException
            except ImportError as e:
                self.logger.error(f"缺少pymodbus库: {e}")
                self.data["status"] = "Error"
                self.data["error_code"] = 1001
                return False
            
            # 创建Modbus客户端
            self.modbus_client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.timeout
            )
            
            # 连接设备
            if not self.modbus_client.connect():
                self.logger.error(f"无法连接到设备端口 {self.port}")
                self.data["status"] = "Error"
                self.data["error_code"] = 1002
                return False
            
            # 测试通信
            success = await self._test_communication()
            if not success:
                self.logger.error("设备通信测试失败")
                self.data["status"] = "Error"
                self.data["error_code"] = 1003
                return False
            
            self.data["status"] = "Idle"
            self.logger.info(f"设备 {self.device_id} 初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"初始化失败: {e}")
            self.data["status"] = "Error"
            self.data["error_code"] = 1000
            return False

    async def _test_communication(self) -> bool:
        """测试Modbus通信"""
        for axis in ['x', 'y', 'z', 'push_rod']:
            address = self.addr_mapping[axis]
            try:
                # 读取状态寄存器
                response = await self._read_register_safe(address, self.reg_mapping['position'])
                if response is None:
                    self.logger.warning(f"轴 {axis} 通信测试失败")
                    return False
            except Exception as e:
                self.logger.warning(f"轴 {axis} 通信异常: {e}")
                return False
        
        self.logger.info("所有轴通信测试成功")
        return True

    async def _read_register_safe(self, address: int, register: int, count: int = 2) -> Optional[Any]:
        """安全的读取寄存器操作，带重试机制"""
        if not self.modbus_client or not self.modbus_client.connected:
            self.logger.error("Modbus客户端未连接")
            return None
        
        last_exception = None
        for attempt in range(self.retry_count):
            try:
                # 清空接收缓冲区
                if hasattr(self.modbus_client, 'socket'):
                    try:
                        self.modbus_client.socket.settimeout(0.01)
                        self.modbus_client.socket.recv(1024)
                    except:
                        pass
                
                # 读取寄存器
                response = self.modbus_client.read_holding_registers(
                    register, count, slave=address
                )
                
                if response.isError():
                    self.logger.warning(f"读取寄存器失败 (尝试 {attempt+1}/{self.retry_count}): {response}")
                    await self._ros_node.sleep(self.retry_delay)
                    continue
                
                # 成功读取
                if attempt > 0:
                    self.logger.info(f"第{attempt+1}次重试成功")
                return response
                
            except Exception as e:
                last_exception = e
                self.logger.warning(f"读取寄存器异常 (尝试 {attempt+1}/{self.retry_count}): {e}")
                await self._ros_node.sleep(self.retry_delay)
        
        self.logger.error(f"读取寄存器最终失败: {last_exception}")
        return None

    async def _write_register_safe(self, address: int, register: int, value: int) -> bool:
        """安全的写入寄存器操作，带重试机制"""
        if not self.modbus_client or not self.modbus_client.connected:
            self.logger.error("Modbus客户端未连接")
            return False
        
        # 检查数值范围
        if value < 0 or value > 65535:
            self.logger.error(f"寄存器值超出范围: {value} (0-65535)")
            return False
        
        last_exception = None
        for attempt in range(self.retry_count):
            try:
                # 清空接收缓冲区
                if hasattr(self.modbus_client, 'socket'):
                    try:
                        self.modbus_client.socket.settimeout(0.01)
                        self.modbus_client.socket.recv(1024)
                    except:
                        pass
                
                # 写入寄存器
                response = self.modbus_client.write_register(
                    register, value, slave=address
                )
                
                if response.isError():
                    self.logger.warning(f"写入寄存器失败 (尝试 {attempt+1}/{self.retry_count}): {response}")
                    await self._ros_node.sleep(self.retry_delay)
                    continue
                
                # 验证写入
                await self._ros_node.sleep(0.05)
                verify_response = await self._read_register_safe(address, register, 1)
                if verify_response and verify_response.registers[0] == value:
                    if attempt > 0:
                        self.logger.info(f"第{attempt+1}次重试写入成功")
                    return True
                else:
                    self.logger.warning(f"写入验证失败 (尝试 {attempt+1}/{self.retry_count})")
                    await self._ros_node.sleep(self.retry_delay)
                    continue
                    
            except Exception as e:
                last_exception = e
                self.logger.warning(f"写入寄存器异常 (尝试 {attempt+1}/{self.retry_count}): {e}")
                await self._ros_node.sleep(self.retry_delay)
        
        self.logger.error(f"写入寄存器最终失败: {last_exception}")
        return False

    @action(description="清理资源")
    async def cleanup(self) -> bool:
        """清理设备"""
        try:
            self.logger.info(f"清理设备 {self.device_id}")
            
            # 停止所有运动
            await self.stop_all()
            
            # 断开连接
            if self.modbus_client and self.modbus_client.connected:
                self.modbus_client.close()
            
            self.data["status"] = "Offline"
            self.logger.info(f"设备 {self.device_id} 清理完成")
            return True
            
        except Exception as e:
            self.logger.error(f"清理失败: {e}")
            return False

    # ========== 属性 ==========

    @property
    def status(self) -> str:
        """设备状态"""
        return self.data.get("status", "Idle")

    @property
    def position(self) -> Dict[str, float]:
        """获取当前位置"""
        return {
            "x": self.data.get("position_x", 0.0),
            "y": self.data.get("position_y", 0.0),
            "z": self.data.get("position_z", 0.0)
        }

    @property
    def is_homed(self) -> bool:
        """是否已回零"""
        return self.data.get("is_homed", False)

    @property
    def is_enabled(self) -> bool:
        """是否已使能"""
        return self.data.get("is_enabled", False)

    @property
    def push_rod_status(self) -> str:
        """推杆状态"""
        return self.data.get("push_rod_status", "released")

    @property
    def error_code(self) -> int:
        """错误代码"""
        return self.data.get("error_code", 0)

    # ========== 动作方法 ==========

    async def enable(self) -> bool:
        """使能所有轴"""
        try:
            self.logger.info("使能所有轴")
            
            success_count = 0
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                success = await self._write_register_safe(address, self.reg_mapping['enable'], 1)
                if success:
                    success_count += 1
            
            if success_count == 3:
                self.data["is_enabled"] = True
                self.data["status"] = "Idle"
                self.logger.info("所有轴使能成功")
                return True
            else:
                self.logger.warning(f"部分轴使能失败 (成功: {success_count}/3)")
                return False
                
        except Exception as e:
            self.logger.error(f"使能失败: {e}")
            return False

    async def disable(self) -> bool:
        """禁用所有轴"""
        try:
            self.logger.info("禁用所有轴")
            
            success_count = 0
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                success = await self._write_register_safe(address, self.reg_mapping['enable'], 0)
                if success:
                    success_count += 1
            
            if success_count > 0:
                self.data["is_enabled"] = False
                self.data["status"] = "Idle"
                self.logger.info(f"成功禁用 {success_count} 个轴")
                return True
            else:
                self.logger.error("所有轴禁用失败")
                return False
                
        except Exception as e:
            self.logger.error(f"禁用失败: {e}")
            return False

    async def go_home(self) -> bool:
        """回零操作 - 修复异步等待错误"""
        try:
            self.logger.info("执行回零操作")
            
            if not self.data["is_enabled"]:
                self.logger.error("设备未使能，无法回零")
                return False
            
            self.data["status"] = "Busy"
            
            # 修复：使用正确的异步等待方法
            home_tasks = []
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                # 发送回零指令
                success = await self._write_register_safe(address, self.reg_mapping['home'], 1)
                if not success:
                    self.logger.error(f"轴 {axis} 回零指令发送失败")
                    self.data["status"] = "Error"
                    return False
            
            # 修复：使用正确的异步等待
            await self._ros_node.sleep(2.0)  # 等待回零完成
            
            # 检查回零状态
            homed_axes = 0
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                response = await self._read_register_safe(address, self.reg_mapping['position'], 2)
                if response:
                    # 读取当前位置，如果接近0则认为回零成功
                    position = (response.registers[0] << 16) + response.registers[1]
                    if abs(position) < 100:  # 100个脉冲范围内认为回零成功
                        homed_axes += 1
                        # 更新位置数据
                        if axis == 'x':
                            self.data["position_x"] = 0.0
                        elif axis == 'y':
                            self.data["position_y"] = 0.0
                        elif axis == 'z':
                            self.data["position_z"] = 0.0
            
            if homed_axes == 3:
                self.data["is_homed"] = True
                self.data["status"] = "Idle"
                self.logger.info("回零成功")
                return True
            else:
                self.logger.warning(f"部分轴回零失败 (成功: {homed_axes}/3)")
                self.data["status"] = "Idle"
                return False
                
        except Exception as e:
            self.logger.error(f"回零失败: {e}")
            self.data["status"] = "Error"
            return False

    async def move_relative(self, x_delta: float = 0.0, y_delta: float = 0.0, 
                           z_delta: float = 0.0, wait_done: bool = True) -> bool:
        """相对移动 - 修复Modbus通信问题"""
        try:
            if not self.data["is_enabled"]:
                self.logger.error("设备未使能，无法移动")
                return False
            
            if not self.data["is_homed"]:
                self.logger.warning("设备未回零，移动可能不准确")
            
            self.logger.info(f"相对移动: X+{x_delta}, Y+{y_delta}, Z+{z_delta}")
            self.data["status"] = "Busy"
            
            # 转换为脉冲数 (假设1mm = 100脉冲)
            pulses = {
                'x': int(x_delta * 100),
                'y': int(y_delta * 100),
                'z': int(z_delta * 100)
            }
            
            # 发送移动指令
            success_count = 0
            for axis in ['x', 'y', 'z']:
                if pulses[axis] == 0:
                    continue
                    
                address = self.addr_mapping[axis]
                
                # 设置目标位置
                target_register = self.reg_mapping['target_pos']
                target_value = pulses[axis]
                
                # 将32位值拆分为两个16位寄存器
                high_word = (target_value >> 16) & 0xFFFF
                low_word = target_value & 0xFFFF
                
                # 写入目标位置
                success1 = await self._write_register_safe(address, target_register, low_word)
                success2 = await self._write_register_safe(address, target_register + 1, high_word)
                
                # 设置速度
                speed_value = 1000  # 默认速度
                speed_success = await self._write_register_safe(address, self.reg_mapping['speed'], speed_value)
                
                # 启动移动
                start_success = await self._write_register_safe(address, 0x09, 1)  # 启动寄存器
                
                if success1 and success2 and speed_success and start_success:
                    success_count += 1
                    self.logger.info(f"轴 {axis} 移动指令发送成功")
                else:
                    self.logger.warning(f"轴 {axis} 移动指令发送失败")
            
            if success_count == 0 and x_delta == 0 and y_delta == 0 and z_delta == 0:
                self.logger.info("无移动指令")
                self.data["status"] = "Idle"
                return True
            
            if wait_done:
                # 等待移动完成
                await self._ros_node.sleep(1.0)  # 修复：使用正确的异步等待
                
                # 检查是否到达目标
                all_reached = True
                for axis in ['x', 'y', 'z']:
                    if pulses[axis] == 0:
                        continue
                    
                    address = self.addr_mapping[axis]
                    response = await self._read_register_safe(address, self.reg_mapping['position'], 2)
                    if response:
                        current_pos = (response.registers[0] << 16) + response.registers[1]
                        target_pos = pulses[axis]
                        
                        # 更新位置数据
                        if axis == 'x':
                            self.data["position_x"] = current_pos / 100.0
                        elif axis == 'y':
                            self.data["position_y"] = current_pos / 100.0
                        elif axis == 'z':
                            self.data["position_z"] = current_pos / 100.0
                        
                        if abs(current_pos - target_pos) > 10:  # 10个脉冲误差范围内
                            all_reached = False
                            self.logger.warning(f"轴 {axis} 未到达目标位置 (当前: {current_pos}, 目标: {target_pos})")
            
            self.data["status"] = "Idle"
            if success_count > 0:
                self.logger.info(f"移动完成 (成功轴: {success_count})")
                return True
            else:
                self.logger.error("所有轴移动失败")
                return False
                
        except Exception as e:
            self.logger.error(f"移动失败: {e}")
            self.data["status"] = "Error"
            return False

    async def move_absolute(self, x: float = 0.0, y: float = 0.0, 
                           z: float = 0.0, wait_done: bool = True) -> bool:
        """绝对位置移动"""
        try:
            # 计算相对移动量
            current_pos = self.position
            x_delta = x - current_pos["x"]
            y_delta = y - current_pos["y"]
            z_delta = z - current_pos["z"]
            
            return await self.move_relative(x_delta, y_delta, z_delta, wait_done)
            
        except Exception as e:
            self.logger.error(f"绝对移动失败: {e}")
            return False

    async def clamp_glass(self) -> bool:
        """夹紧玻璃"""
        try:
            self.logger.info("执行玻璃夹紧")
            
            address = self.addr_mapping['push_rod']
            success = await self._write_register_safe(address, self.reg_mapping['clamp'], 1)
            
            if success:
                self.data["push_rod_status"] = "clamped"
                self.data["status"] = "Idle"
                self.logger.info("玻璃夹紧成功")
                return True
            else:
                self.logger.error("玻璃夹紧失败")
                self.data["status"] = "Error"
                return False
                
        except Exception as e:
            self.logger.error(f"夹紧失败: {e}")
            self.data["status"] = "Error"
            return False

    async def release_glass(self) -> bool:
        """释放玻璃"""
        try:
            self.logger.info("执行玻璃释放")
            
            address = self.addr_mapping['push_rod']
            success = await self._write_register_safe(address, self.reg_mapping['release'], 1)
            
            if success:
                self.data["push_rod_status"] = "released"
                self.data["status"] = "Idle"
                self.logger.info("玻璃释放成功")
                return True
            else:
                self.logger.error("玻璃释放失败")
                self.data["status"] = "Error"
                return False
                
        except Exception as e:
            self.logger.error(f"释放失败: {e}")
            self.data["status"] = "Error"
            return False

    async def stop_all(self) -> bool:
        """停止所有运动"""
        try:
            self.logger.info("停止所有运动")
            
            success_count = 0
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                success = await self._write_register_safe(address, self.reg_mapping['stop'], 1)
                if success:
                    success_count += 1
            
            # 停止推杆
            address = self.addr_mapping['push_rod']
            push_success = await self._write_register_safe(address, self.reg_mapping['stop'], 1)
            if push_success:
                success_count += 1
            
            self.data["status"] = "Idle"
            self.logger.info(f"停止指令发送完成 (成功: {success_count}/4)")
            return success_count > 0
            
        except Exception as e:
            self.logger.error(f"停止失败: {e}")
            return False

    async def get_position(self) -> Dict[str, float]:
        """获取当前位置"""
        try:
            positions = {}
            for axis in ['x', 'y', 'z']:
                address = self.addr_mapping[axis]
                response = await self._read_register_safe(address, self.reg_mapping['position'], 2)
                if response:
                    position = (response.registers[0] << 16) + response.registers[1]
                    positions[axis] = position / 100.0  # 转换为mm
                else:
                    positions[axis] = 0.0
            
            # 更新数据
            self.data["position_x"] = positions.get('x', 0.0)
            self.data["position_y"] = positions.get('y', 0.0)
            self.data["position_z"] = positions.get('z', 0.0)
            
            return positions
            
        except Exception as e:
            self.logger.error(f"获取位置失败: {e}")
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    async def reset_error(self) -> bool:
        """重置错误"""
        try:
            self.logger.info("重置设备错误")
            
            success_count = 0
            for axis in ['x', 'y', 'z', 'push_rod']:
                address = self.addr_mapping[axis]
                success = await self._write_register_safe(address, 0x00, 0)  # 错误复位寄存器
                if success:
                    success_count += 1
            
            if success_count > 0:
                self.data["error_code"] = 0
                self.data["status"] = "Idle"
                self.logger.info(f"错误重置成功 (成功: {success_count}/4)")
                return True
            else:
                self.logger.error("错误重置失败")
                return False
                
        except Exception as e:
            self.logger.error(f"错误重置失败: {e}")
            return False

    async def set_speed(self, axis: str, speed: float) -> bool:
        """设置轴速度 (mm/s)"""
        try:
            if axis not in ['x', 'y', 'z']:
                self.logger.error(f"无效的轴名称: {axis}")
                return False
            
            address = self.addr_mapping[axis]
            speed_value = int(speed * 100)  # 转换为脉冲/秒
            
            success = await self._write_register_safe(address, self.reg_mapping['speed'], speed_value)
            
            if success:
                self.logger.info(f"轴 {axis} 速度设置为 {speed} mm/s")
                return True
            else:
                self.logger.error(f"轴 {axis} 速度设置失败")
                return False
                
        except Exception as e:
            self.logger.error(f"设置速度失败: {e}")
            return False

    async def set_acceleration(self, axis: str, accel: float) -> bool:
        """设置轴加速度 (mm/s²)"""
        try:
            if axis not in ['x', 'y', 'z']:
                self.logger.error(f"无效的轴名称: {axis}")
                return False
            
            address = self.addr_mapping[axis]
            accel_value = int(accel * 10)  # 转换为脉冲/秒²
            
            success = await self._write_register_safe(address, self.reg_mapping['accel'], accel_value)
            
            if success:
                self.logger.info(f"轴 {axis} 加速度设置为 {accel} mm/s²")
                return True
            else:
                self.logger.error(f"轴 {axis} 加速度设置失败")
                return False
                
        except Exception as e:
            self.logger.error(f"设置加速度失败: {e}")
            return False