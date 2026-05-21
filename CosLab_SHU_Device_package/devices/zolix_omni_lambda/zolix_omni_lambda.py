import logging
import time as time_module
from typing import Dict, Any, Optional

try:
    import serial
except ImportError:
    serial = None

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None


class ZolixOmniLambda:
    """Zolix Omni-λ 单色仪/光谱仪驱动

    通信协议: Serial (USB/RS232), ASCII 文本指令, \\r 结束符
    默认波特率: 19200, 8N1
    指令格式: "<COMMAND> [参数]\\r"
    响应格式: ASCII 文本, 以 "OK" 或 "Exx" 结尾

    支持功能:
    - 波长绝对/相对移动 (nm)
    - 波数绝对移动 (cm⁻¹)
    - 光栅切换与查询
    - 光栅台切换
    - 出入口切换
    - 系统信息查询
    - IO 端口控制
    """

    _ros_node: "BaseROS2DeviceNode"

    # ---------- 错误码映射 ----------
    ERROR_CODES = {
        "E01": "Command not recognized",
        "E02": "Parameter out of range",
        "E03": "Device busy",
        "E04": "Communication error",
        "E05": "Hardware error",
        "E06": "Timeout",
    }

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')

        self.device_id = device_id or "unknown_device"
        self.config = config or {}
        self.logger = logging.getLogger(f"ZolixOmniLambda.{self.device_id}")

        # self.data 必须预填充所有 @property 对应的字段
        self.data: Dict[str, Any] = {
            "status": "Offline",
            "wavelength": 0.0,
            "wavenumber": 0.0,
            "grating": "1",
            "turret": "1",
            "exit_port": "0",
            "entrance_port": "0",
            "system_info": "",
        }

        # 串口配置 — 同时从 config 和 kwargs 中查找，兼容框架不同传参方式
        self._port = self.config.get("port") or kwargs.get("port", "COM11")
        self._baudrate = int(self.config.get("baudrate") or kwargs.get("baudrate", 19200))
        self._timeout = float(self.config.get("timeout") or kwargs.get("timeout", 5))
        self._ser: Optional[Any] = None

        self.logger.info(f"Config received: config={self.config}, kwargs_keys={list(kwargs.keys())}, using port={self._port}")

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    # ========== 通信层 ==========

    def _connect(self):
        """打开串口连接"""
        if serial is None:
            raise ImportError("pyserial is required. Install with: pip install pyserial")
        if self._ser is not None and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
        )
        self.logger.info(f"Serial port opened: {self._port} @ {self._baudrate}")

    def _disconnect(self):
        """关闭串口连接"""
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self.logger.info("Serial port closed")
        self._ser = None

    def _send_command(self, cmd: str, timeout: float = None) -> str:
        """发送 ASCII 指令并读取响应

        指令格式: "COMMAND [params]\\r"
        响应: 读取直到 "OK\\r" 或 "Exx\\r" 或超时

        Args:
            cmd: 指令字符串 (不含 \\r)
            timeout: 可选超时覆盖 (秒)

        Returns:
            响应字符串 (去除前导垃圾字节后)
        """
        if self._ser is None or not self._ser.is_open:
            self._connect()

        # 清空接收缓冲区
        self._ser.reset_input_buffer()

        # 发送指令
        full_cmd = f"{cmd}\r"
        self._ser.write(full_cmd.encode("ascii"))
        self.logger.debug(f"TX: {cmd}")

        # 读取响应 — 使用 buffer 累积 + 关键字匹配
        effective_timeout = timeout if timeout is not None else self._timeout
        buffer = ""
        start_time = time_module.time()

        while time_module.time() - start_time < effective_timeout:
            if self._ser.in_waiting > 0:
                chunk = self._ser.read(self._ser.in_waiting).decode("ascii", errors="replace")
                buffer += chunk

                # 检查是否收到完整响应 (以 OK 或 Exx 结尾)
                stripped = buffer.strip()
                if stripped.endswith("OK"):
                    break
                for code in self.ERROR_CODES:
                    if stripped.endswith(code):
                        break
                else:
                    time_module.sleep(0.05)
                    continue
                break
            else:
                time_module.sleep(0.05)

        self.logger.debug(f"RX: {buffer.strip()}")
        return buffer.strip()

    def _parse_response(self, response: str) -> Dict[str, Any]:
        """解析响应字符串

        Returns:
            {"success": bool, "data": str, "error": str or None}
        """
        if not response:
            return {"success": False, "data": "", "error": "No response (timeout)"}

        # 检查错误码
        for code, desc in self.ERROR_CODES.items():
            if code in response:
                return {"success": False, "data": response, "error": f"{code}: {desc}"}

        if "OK" in response:
            # 提取 OK 之前的数据部分
            idx = response.rfind("OK")
            data_part = response[:idx].strip()
            return {"success": True, "data": data_part, "error": None}

        # 没有明确的 OK 或 Exx，视为数据响应
        return {"success": True, "data": response, "error": None}

    def _wait_until_idle(self, timeout: float = 120.0):
        """轮询等待设备完成移动 (同步方法, 在底层串口通信中使用)"""
        start = time_module.time()
        while time_module.time() - start < timeout:
            resp = self._send_command("POSITION?")
            parsed = self._parse_response(resp)
            if parsed["success"]:
                return True
            if parsed.get("error", "").startswith("E03"):
                time_module.sleep(0.5)
                continue
            time_module.sleep(0.2)
        return False

    # ========== 生命周期 ==========

    async def initialize(self) -> bool:
        """初始化设备: 打开串口, 发送 Hello 联络指令, 查询初始状态"""
        try:
            self._connect()

            # 发送 Hello 联络指令
            resp = self._send_command("Hello")
            parsed = self._parse_response(resp)
            if not parsed["success"]:
                self.logger.warning(f"Hello command failed: {parsed['error']}")

            # 查询当前波长位置
            try:
                self._do_query_position()
            except Exception as e:
                self.logger.warning(f"Initial position query failed: {e}")

            # 查询系统信息
            try:
                self._do_query_system_info()
            except Exception as e:
                self.logger.warning(f"Initial system info query failed: {e}")

            self.data["status"] = "Idle"
            self.logger.info("Device initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"Initialization failed: {e}")
            self.data["status"] = "Offline"
            return False

    async def cleanup(self) -> bool:
        """清理资源: 关闭串口"""
        try:
            self._disconnect()
            self.data["status"] = "Offline"
            return True
        except Exception as e:
            self.logger.error(f"Cleanup failed: {e}")
            return False

    # ========== 内部查询方法 ==========

    def _do_query_position(self):
        """查询并更新当前波长位置"""
        resp = self._send_command("POSITION?")
        parsed = self._parse_response(resp)
        if parsed["success"] and parsed["data"]:
            try:
                wl = float(parsed["data"])
                self.data["wavelength"] = wl
                if wl > 0:
                    self.data["wavenumber"] = 1e7 / wl  # nm -> cm⁻¹
            except (ValueError, ZeroDivisionError):
                pass

    def _do_query_system_info(self):
        """查询并更新系统信息"""
        resp = self._send_command("SYSTEMINFO?")
        parsed = self._parse_response(resp)
        if parsed["success"]:
            self.data["system_info"] = parsed["data"]

    # ========== 动作方法 ==========
    # 注意：所有参数类型只能用 float 或 str，不能用 int
    # 因为 UniLab-OS 的 action type mapping 只支持 float 和 str

    async def move_to(self, wavelength: float, **kwargs) -> Dict[str, Any]:
        """绝对移动到指定波长

        Args:
            wavelength: 目标波长 (nm)

        Returns:
            {"success": bool, "wavelength": float}
        """
        wavelength = float(wavelength)
        self.data["status"] = "Busy"

        try:
            resp = self._send_command(f"MOVETO {wavelength}")
            parsed = self._parse_response(resp)

            if not parsed["success"]:
                self.data["status"] = "Idle"
                return {"success": False, "error": parsed["error"]}

            self._wait_until_idle()
            self._do_query_position()
            self.data["status"] = "Idle"
            return {"success": True, "wavelength": self.data["wavelength"]}

        except Exception as e:
            self.logger.error(f"move_to failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def move_relative(self, wavelength: float, **kwargs) -> Dict[str, Any]:
        """相对移动指定波长

        Args:
            wavelength: 相对移动量 (nm), 正值红移, 负值蓝移

        Returns:
            {"success": bool, "wavelength": float}
        """
        wavelength = float(wavelength)
        self.data["status"] = "Busy"

        try:
            resp = self._send_command(f"MOVE {wavelength}")
            parsed = self._parse_response(resp)

            if not parsed["success"]:
                self.data["status"] = "Idle"
                return {"success": False, "error": parsed["error"]}

            self._wait_until_idle()
            self._do_query_position()
            self.data["status"] = "Idle"
            return {"success": True, "wavelength": self.data["wavelength"]}

        except Exception as e:
            self.logger.error(f"move_relative failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def move_to_wavenumber(self, wavenumber: float, **kwargs) -> Dict[str, Any]:
        """绝对移动到指定波数

        Args:
            wavenumber: 目标波数 (cm⁻¹)

        Returns:
            {"success": bool, "wavenumber": float}
        """
        wavenumber = float(wavenumber)
        self.data["status"] = "Busy"

        try:
            resp = self._send_command(f"WaveNumber_abs {wavenumber}")
            parsed = self._parse_response(resp)

            if not parsed["success"]:
                self.data["status"] = "Idle"
                return {"success": False, "error": parsed["error"]}

            self._wait_until_idle()
            self._do_query_position()
            self.data["status"] = "Idle"
            return {"success": True, "wavenumber": self.data["wavenumber"]}

        except Exception as e:
            self.logger.error(f"move_to_wavenumber failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def set_grating(self, grating: str, **kwargs) -> Dict[str, Any]:
        """切换光栅

        Args:
            grating: 光栅编号 (1~3)

        Returns:
            {"success": bool, "grating": str}
        """
        grating_val = str(grating)
        self.data["status"] = "Busy"

        try:
            resp = self._send_command(f"GRATING {grating_val}")
            parsed = self._parse_response(resp)

            if not parsed["success"]:
                self.data["status"] = "Idle"
                return {"success": False, "error": parsed["error"]}

            self._wait_until_idle()
            self.data["grating"] = grating_val
            self.data["status"] = "Idle"
            return {"success": True, "grating": grating_val}

        except Exception as e:
            self.logger.error(f"set_grating failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def grating_home(self, **kwargs) -> Dict[str, Any]:
        """光栅重新定位 (回零)

        Returns:
            {"success": bool}
        """
        self.data["status"] = "Busy"

        try:
            resp = self._send_command("GRATINGHOME")
            parsed = self._parse_response(resp)

            self._wait_until_idle(timeout=60.0)
            self._do_query_position()
            self.data["status"] = "Idle"
            return {"success": parsed["success"], "error": parsed.get("error")}

        except Exception as e:
            self.logger.error(f"grating_home failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def set_turret(self, turret: str, **kwargs) -> Dict[str, Any]:
        """设置光栅台

        Args:
            turret: 光栅台编号

        Returns:
            {"success": bool, "turret": str}
        """
        turret_val = str(turret)
        self.data["status"] = "Busy"

        try:
            resp = self._send_command(f"TURRET {turret_val}")
            parsed = self._parse_response(resp)

            self._wait_until_idle()
            self.data["turret"] = turret_val
            self.data["status"] = "Idle"
            return {"success": parsed["success"], "turret": turret_val, "error": parsed.get("error")}

        except Exception as e:
            self.logger.error(f"set_turret failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def set_exit_port(self, port: str, **kwargs) -> Dict[str, Any]:
        """切换出口

        Args:
            port: 出口编号 (0=前置, 1=侧面)

        Returns:
            {"success": bool, "exit_port": str}
        """
        port_val = str(port)

        try:
            resp = self._send_command(f"EXITPORT {port_val}")
            parsed = self._parse_response(resp)

            if parsed["success"]:
                self.data["exit_port"] = port_val
            return {"success": parsed["success"], "exit_port": port_val, "error": parsed.get("error")}

        except Exception as e:
            self.logger.error(f"set_exit_port failed: {e}")
            return {"success": False, "error": str(e)}

    async def set_entrance_port(self, port: str, **kwargs) -> Dict[str, Any]:
        """切换入口

        Args:
            port: 入口编号 (0=前置, 1=侧面)

        Returns:
            {"success": bool, "entrance_port": str}
        """
        port_val = str(port)

        try:
            resp = self._send_command(f"ENTRANCEPORT {port_val}")
            parsed = self._parse_response(resp)

            if parsed["success"]:
                self.data["entrance_port"] = port_val
            return {"success": parsed["success"], "entrance_port": port_val, "error": parsed.get("error")}

        except Exception as e:
            self.logger.error(f"set_entrance_port failed: {e}")
            return {"success": False, "error": str(e)}

    async def stop(self, **kwargs) -> Dict[str, Any]:
        """停止当前移动

        Returns:
            {"success": bool}
        """
        try:
            resp = self._send_command("STOP")
            parsed = self._parse_response(resp)
            self.data["status"] = "Idle"
            return {"success": parsed["success"], "error": parsed.get("error")}

        except Exception as e:
            self.logger.error(f"stop failed: {e}")
            self.data["status"] = "Idle"
            return {"success": False, "error": str(e)}

    async def query_position(self, **kwargs) -> Dict[str, Any]:
        """查询当前波长位置

        Returns:
            {"success": bool, "wavelength": float, "wavenumber": float}
        """
        try:
            self._do_query_position()
            return {
                "success": True,
                "wavelength": self.data["wavelength"],
                "wavenumber": self.data["wavenumber"],
            }
        except Exception as e:
            self.logger.error(f"query_position failed: {e}")
            return {"success": False, "error": str(e)}

    async def query_system_info(self, **kwargs) -> Dict[str, Any]:
        """查询系统信息

        Returns:
            {"success": bool, "system_info": str}
        """
        try:
            self._do_query_system_info()
            return {"success": True, "system_info": self.data["system_info"]}
        except Exception as e:
            self.logger.error(f"query_system_info failed: {e}")
            return {"success": False, "error": str(e)}

    async def query_gratings(self, **kwargs) -> Dict[str, Any]:
        """查询光栅参数

        Returns:
            {"success": bool, "gratings_info": str}
        """
        try:
            resp = self._send_command("GRATINGS?")
            parsed = self._parse_response(resp)
            return {"success": parsed["success"], "gratings_info": parsed["data"], "error": parsed.get("error")}
        except Exception as e:
            self.logger.error(f"query_gratings failed: {e}")
            return {"success": False, "error": str(e)}

    async def set_port_output(self, value: str, **kwargs) -> Dict[str, Any]:
        """设置 IO 端口输出

        Args:
            value: 输出值字符串

        Returns:
            {"success": bool}
        """
        try:
            resp = self._send_command(f"PORT_OUTPUT {value}")
            parsed = self._parse_response(resp)
            return {"success": parsed["success"], "error": parsed.get("error")}
        except Exception as e:
            self.logger.error(f"set_port_output failed: {e}")
            return {"success": False, "error": str(e)}

    async def send_command(self, command: str, **kwargs) -> Dict[str, Any]:
        """发送自定义指令

        Args:
            command: 完整指令字符串 (不含 \\r)

        Returns:
            {"success": bool, "response": str}
        """
        try:
            resp = self._send_command(str(command))
            parsed = self._parse_response(resp)
            return {"success": parsed["success"], "response": parsed["data"], "error": parsed.get("error")}
        except Exception as e:
            self.logger.error(f"send_command failed: {e}")
            return {"success": False, "error": str(e)}

    # ========== 属性 (property) ==========
    # 注意：@property 返回类型只能用 float, str, bool
    # 不能用 int，否则 set_<property> 自动生成时会触发
    # ValueError: Unsupported action type: <class 'int'>

    @property
    def status(self) -> str:
        return self.data.get("status", "Offline")

    @property
    def wavelength(self) -> float:
        """当前波长 (nm)"""
        return float(self.data.get("wavelength", 0.0))

    @property
    def wavenumber(self) -> float:
        """当前波数 (cm⁻¹)"""
        return float(self.data.get("wavenumber", 0.0))

    @property
    def grating(self) -> str:
        """当前光栅号"""
        return str(self.data.get("grating", "1"))

    @property
    def turret(self) -> str:
        """当前光栅台号"""
        return str(self.data.get("turret", "1"))

    @property
    def exit_port(self) -> str:
        """当前出口 (0=前置, 1=侧面)"""
        return str(self.data.get("exit_port", "0"))

    @property
    def entrance_port(self) -> str:
        """当前入口 (0=前置, 1=侧面)"""
        return str(self.data.get("entrance_port", "0"))

    @property
    def system_info(self) -> str:
        """仪器系统信息"""
        return str(self.data.get("system_info", ""))
