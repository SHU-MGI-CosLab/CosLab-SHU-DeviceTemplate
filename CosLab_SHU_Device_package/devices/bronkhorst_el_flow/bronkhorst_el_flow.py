"""
Bronkhorst EL-FLOW Prestige 质量流量控制器 (MFC) 驱动
"""

import logging
import traceback
from typing import Dict, Any

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

try:
    import propar
except ImportError:
    propar = None


class BronkhorstElFlow:
    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")

        self.device_id = device_id or "unknown_device"
        self.config = config or {}
        self.logger = logging.getLogger(f"BronkhorstElFlow.{self.device_id}")

        self.data = {
            "status": "Idle",
            "flow": 0.0,
            "setpoint": 0.0,
            "temperature": 0.0,
            "valve_output": 0.0,
            "capacity_unit": "",
            "user_tag": "",
            "level": False,
            "rssi": 0,
            "value": 0.0,
        }

        self._port = self.config.get("port") or kwargs.get("port", "COM12")
        self._baudrate = int(self.config.get("baudrate") or kwargs.get("baudrate", 38400))
        self._address = int(self.config.get("address") or kwargs.get("address", 3))
        self._channel = int(self.config.get("channel") or kwargs.get("channel", 1))
        self._threshold_pct = float(self.config.get("threshold") or kwargs.get("threshold", 2.0))

        self._instrument = None

        # 在 __init__ 中直接连接设备
        if propar is not None:
            try:
                self._instrument = propar.instrument(
                    self._port,
                    self._address,
                    baudrate=self._baudrate,
                )
                unit = self._instrument.readParameter(129)
                if unit is not None:
                    self.data["capacity_unit"] = str(unit)
                tag = self._instrument.readParameter(115)
                if tag is not None:
                    self.data["user_tag"] = str(tag)
                self._poll_values()
                print(f"[DEBUG] __init__ 中连接成功! unit={unit}, tag={tag}")
            except Exception as e:
                print(f"[DEBUG] __init__ 中连接失败: {e}")
                traceback.print_exc()
                self._instrument = None

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    async def initialize(self) -> bool:
        print(f"[DEBUG-INITIALIZE] 被调用了! instrument={self._instrument}")
        if self._instrument is not None:
            self.data["status"] = "Idle"
            print("[DEBUG-INITIALIZE] 设备已在__init__中连接, 返回 True")
            return True
        if propar is None:
            self.data["status"] = "Offline"
            return False
        try:
            self._instrument = propar.instrument(
                self._port,
                self._address,
                baudrate=self._baudrate,
            )
            unit = self._instrument.readParameter(129)
            if unit is not None:
                self.data["capacity_unit"] = str(unit)
            self._poll_values()
            self.data["status"] = "Idle"
            print("[DEBUG-INITIALIZE] 连接成功, 返回 True")
            return True
        except Exception as e:
            print(f"[DEBUG-INITIALIZE] 连接失败: {e}")
            self.data["status"] = "Offline"
            return False

    async def cleanup(self) -> bool:
        try:
            if self._instrument is not None:
                try:
                    self._instrument.writeParameter(206, 0.0)
                except Exception:
                    pass
                self._instrument = None
            self.data["status"] = "Offline"
            return True
        except Exception as e:
            self.data["status"] = "Offline"
            return False

    def _poll_values(self):
        if self._instrument is None:
            return
        try:
            flow_val = self._instrument.readParameter(205)
            if flow_val is not None:
                self.data["flow"] = float(flow_val)
                self.data["value"] = float(flow_val)
            sp_val = self._instrument.readParameter(206)
            if sp_val is not None:
                self.data["setpoint"] = float(sp_val)
            temp_val = self._instrument.readParameter(142)
            if temp_val is not None:
                self.data["temperature"] = float(temp_val)
            sp = self.data["setpoint"]
            fl = self.data["flow"]
            if sp > 0:
                self.data["level"] = abs(fl - sp) / sp * 100.0 <= self._threshold_pct
            else:
                self.data["level"] = abs(fl) < 0.01
        except Exception as e:
            self.logger.warning(f"读取设备数据失败: {e}")

    async def read_value(self, **kwargs) -> Dict[str, Any]:
        self.data["status"] = "Busy"
        try:
            self._poll_values()
            self.data["status"] = "Idle"
            return {
                "success": True,
                "value": self.data["flow"],
                "unit": self.data["capacity_unit"],
                "setpoint": self.data["setpoint"],
                "temperature": self.data["temperature"],
            }
        except Exception as e:
            self.data["status"] = "Idle"
            return {"success": False, "message": str(e)}

    async def set_threshold(self, threshold: float, **kwargs) -> bool:
        self._threshold_pct = float(threshold)
        self._poll_values()
        return True

    async def set_setpoint(self, setpoint: float, **kwargs) -> bool:
        setpoint = float(setpoint)
        if self._instrument is None:
            self.logger.error("设备未连接")
            return False
        self.data["status"] = "Busy"
        try:
            self._instrument.writeParameter(206, setpoint)
            self.data["setpoint"] = setpoint
            await self._ros_node.sleep(0.5)
            self._poll_values()
            self.data["status"] = "Idle"
            return True
        except Exception as e:
            self.logger.error(f"set_setpoint 失败: {e}")
            self.data["status"] = "Idle"
            return False

    async def stop(self, **kwargs) -> bool:
        return await self.set_setpoint(0.0)

    async def set_setpoint_percent(self, percent: float, **kwargs) -> bool:
        percent = float(percent)
        if self._instrument is None:
            return False
        self.data["status"] = "Busy"
        try:
            raw_value = int(percent / 100.0 * 32000)
            raw_value = max(0, min(32000, raw_value))
            self._instrument.setpoint = raw_value
            await self._ros_node.sleep(0.5)
            self._poll_values()
            self.data["status"] = "Idle"
            return True
        except Exception as e:
            self.data["status"] = "Idle"
            return False

    async def set_user_tag(self, tag: str, **kwargs) -> bool:
        tag = str(tag)[:12]
        if self._instrument is None:
            return False
        try:
            self._instrument.writeParameter(115, tag)
            self.data["user_tag"] = tag
            return True
        except Exception as e:
            return False

    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def flow(self) -> float:
        return self.data.get("flow", 0.0)

    @property
    def setpoint(self) -> float:
        return self.data.get("setpoint", 0.0)

    @property
    def temperature(self) -> float:
        return self.data.get("temperature", 0.0)

    @property
    def valve_output(self) -> float:
        return self.data.get("valve_output", 0.0)

    @property
    def capacity_unit(self) -> str:
        return self.data.get("capacity_unit", "")

    @property
    def user_tag(self) -> str:
        return self.data.get("user_tag", "")

    @property
    def level(self) -> bool:
        return self.data.get("level", False)

    @property
    def rssi(self) -> int:
        return self.data.get("rssi", 0)

    @property
    def value(self) -> float:
        return self.data.get("value", 0.0)