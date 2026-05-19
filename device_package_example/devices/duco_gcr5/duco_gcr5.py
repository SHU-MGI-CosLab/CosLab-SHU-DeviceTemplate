"""
DUCO GCR5-910 协作机器人驱动（TCP 2000 端口纯文本协议）

真实命令格式（通过 Telnet 验证）:
  - poweron()      -> poweron success
  - poweroff()     -> poweroff success
  - enable()       -> enable success
  - disable()      -> disable success
  - run("program/20260421.jspf",70) -> run success
  - state          -> 4:0:2:x (机器人状态:程序状态:操作模式:子状态)
  - speed(50)      -> set speed 50%
  - clear()        -> clear alarm

状态解析:
  state[0]: 0=Start 4=PowerOff 5=Disable 6=Enable
  state[1]: 0=Stopped 2=Running 3=Paused
  state[2]: 2=Remote 5=Local
  state[3]: Local下 0=Manual 1=Auto
"""

import logging
import socket
import struct
import threading
import time as time_module
from typing import Dict, Any, List, Optional

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
    id="duco_gcr5",
    category=["robot"],
    description="DUCO GCR5-910 协作机器人，TCP 通信",
    display_name="DUCO 协作机器人"
)
class DucoGCR5:
    """新松 DUCO GCR5-910 协作机器人驱动（TCP 2000 文本协议版）"""

    _ros_node: "BaseROS2DeviceNode"

    ROBOT_STATE_MAP = {
        0: "SR_Start",
        1: "SR_Initialize",
        2: "SR_Logout",
        3: "SR_Login",
        4: "SR_PowerOff",
        5: "SR_Disable",
        6: "SR_Enable",
    }

    PROGRAM_STATE_MAP = {
        0: "SP_Stopped",
        1: "SP_Stopping",
        2: "SP_Running",
        3: "SP_Paused",
        4: "SP_Pausing",
        5: "SP_TaskRunning",
    }

    STATUS_MAP = {
        "SP_Stopped": "Idle",
        "SP_Stopping": "Busy",
        "SP_Running": "Running",
        "SP_Paused": "Paused",
        "SP_Pausing": "Busy",
        "SP_TaskRunning": "Running",
    }

    OP_MODE_MAP = {
        2: "Remote",
        5: "Local",
    }

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")
        self.device_id = device_id or "duco_gcr5"
        self.config = config or {}
        self.logger = logging.getLogger(f"DucoGCR5.{self.device_id}")

        # 连接参数
        self._ip = self.config.get("ip", "192.168.1.10")
        self._cmd_port = self.config.get("cmd_port", 2000)        # 命令端口
        self._status_port = self.config.get("status_port", 2001)  # 状态推送端口
        self._timeout = self.config.get("timeout", 5.0)

        # Socket
        self._cmd_socket: Optional[socket.socket] = None
        self._status_socket: Optional[socket.socket] = None

        # 状态监听
        self._status_thread: Optional[threading.Thread] = None
        self._running = False

        # 预填充
        self.data = {
            "status": "Offline",
            "position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tcp_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "robot_state_name": "SR_Start",
            "program_state_name": "SP_Stopped",
            "speed": 100.0,
            "joint_velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint_torque": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tcp_force": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "operation_mode_name": "Unknown",
            "error_message": "",
        }

    @not_action
    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    # ======================== TCP 命令通信 ========================

    def _connect_cmd(self) -> bool:
        """连接 TCP 2000 命令端口"""
        try:
            if self._cmd_socket:
                try:
                    self._cmd_socket.close()
                except Exception:
                    pass
            self._cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._cmd_socket.settimeout(self._timeout)
            self._cmd_socket.connect((self._ip, self._cmd_port))
            self.logger.info(f"命令端口 {self._ip}:{self._cmd_port} 连接成功")
            return True
        except Exception as e:
            self.logger.error(f"命令端口连接失败: {e}")
            self._cmd_socket = None
            return False

    def _send_cmd(self, cmd: str) -> Optional[str]:
        """发送命令并接收响应"""
        if not self._cmd_socket:
            if not self._connect_cmd():
                return None
        try:
            self._cmd_socket.sendall(cmd.encode("utf-8"))
            response = self._cmd_socket.recv(4096).decode("utf-8", errors="ignore").strip()
            self.logger.debug(f"CMD: {cmd} -> {response}")
            return response
        except socket.timeout:
            self.logger.warning(f"命令超时: {cmd}")
            return None
        except Exception as e:
            self.logger.error(f"命令发送失败: {e}")
            try:
                if self._cmd_socket:
                    self._cmd_socket.close()
            except Exception:
                pass
            self._cmd_socket = None
            return None

    def _is_success(self, resp: Optional[str]) -> bool:
        """判断响应是否成功"""
        if resp is None:
            return False
        low = resp.lower()
        return "success" in low or "ok" in low

    # ======================== TCP 状态端口 ========================

    def _connect_status(self) -> bool:
        """连接 TCP 2001 状态推送端口"""
        try:
            if self._status_socket:
                try:
                    self._status_socket.close()
                except Exception:
                    pass
            self._status_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._status_socket.settimeout(2.0)
            self._status_socket.connect((self._ip, self._status_port))
            self.logger.info(f"状态端口 {self._ip}:{self._status_port} 连接成功")
            return True
        except Exception as e:
            self.logger.error(f"状态端口连接失败: {e}")
            self._status_socket = None
            return False

    def _parse_status_packet(self, data: bytes):
        """解析状态包"""
        if len(data) < 1468:
            return
        try:
            joint_pos = struct.unpack_from("<7f", data, 0)[:6]
            joint_vel = struct.unpack_from("<7f", data, 28)[:6]
            joint_torque = struct.unpack_from("<7f", data, 84)[:6]
            tcp_pose = struct.unpack_from("<6f", data, 368)
            tcp_force = struct.unpack_from("<6f", data, 440)
            global_speed = struct.unpack_from("<B", data, 660)[0]
            operation_mode = struct.unpack_from("<b", data, 1448)[0]
            robot_state = struct.unpack_from("<b", data, 1449)[0]
            program_state = struct.unpack_from("<b", data, 1450)[0]
            error_code = struct.unpack_from("<I", data, 1456)[0]

            self.data["position"] = list(joint_pos)
            self.data["joint_velocity"] = list(joint_vel)
            self.data["joint_torque"] = list(joint_torque)
            self.data["tcp_pose"] = list(tcp_pose)
            self.data["tcp_force"] = list(tcp_force)
            self.data["speed"] = float(global_speed)
            self.data["operation_mode_name"] = self.OP_MODE_MAP.get(operation_mode, f"Mode_{operation_mode}")
            self.data["robot_state_name"] = self.ROBOT_STATE_MAP.get(robot_state, "Unknown")
            self.data["program_state_name"] = self.PROGRAM_STATE_MAP.get(program_state, "Unknown")
            self.data["error_message"] = "" if error_code == 0 else f"ErrorCode: {error_code}"

            if error_code != 0:
                self.data["status"] = "Error"
            elif robot_state < 6:
                self.data["status"] = "Idle"
            else:
                prog_name = self.PROGRAM_STATE_MAP.get(program_state, "SP_Stopped")
                self.data["status"] = self.STATUS_MAP.get(prog_name, "Idle")
        except Exception as e:
            self.logger.warning(f"状态包解析异常: {e}")

    def _status_listener(self):
        """状态监听线程"""
        buffer = b""
        while self._running:
            try:
                if not self._status_socket:
                    time_module.sleep(0.5)
                    continue
                chunk = self._status_socket.recv(4096)
                if not chunk:
                    self.logger.warning("状态端口连接断开")
                    break
                buffer += chunk
                while len(buffer) >= 1468:
                    packet = buffer[:1468]
                    buffer = buffer[1468:]
                    self._parse_status_packet(packet)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.logger.error(f"状态监听异常: {e}")
                break
        if self._running:
            self.data["status"] = "Offline"
            self.logger.info("状态监听线程退出")

    # ======================== 生命周期 ========================

    @action(description="初始化设备")
    async def initialize(self) -> bool:
        """初始化"""
        if not self._connect_cmd():
            self.data["status"] = "Offline"
            return False

        if not self._connect_status():
            self.logger.warning("状态端口连接失败，将继续但无法接收实时状态")

        self._running = True
        self._status_thread = threading.Thread(
            target=self._status_listener, daemon=True, name=f"duco_status_{self.device_id}"
        )
        self._status_thread.start()

        await self._ros_node.sleep(0.5)

        # 查询初始状态
        state = self._query_state_raw()
        if state:
            self.logger.info(f"初始 state={state}")
            if state[0] == 6:
                self.logger.info("✅ 机器人已使能")
            elif state[0] == 4:
                self.logger.info("⚠️ 机器人处于 PowerOff 状态，需要 poweron()")
            elif state[0] == 5:
                self.logger.info("⚠️ 机器人处于 Disable 状态，需要 enable()")

        self.logger.info("DUCO GCR5（TCP 2000）初始化完成")
        return True

    @action(description="清理资源")
    async def cleanup(self) -> bool:
        """清理"""
        self._running = False
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=3.0)
        if self._cmd_socket:
            try:
                self._cmd_socket.close()
            except Exception:
                pass
            self._cmd_socket = None
        if self._status_socket:
            try:
                self._status_socket.close()
            except Exception:
                pass
            self._status_socket = None
        self.data["status"] = "Offline"
        return True

    def _query_state_raw(self) -> Optional[List[int]]:
        """查询 state，返回 [机器人状态, 程序状态, 操作模式, 子状态]"""
        resp = self._send_cmd("state")
        if not resp:
            return None
        try:
            return [int(x) for x in resp.split(":")]
        except Exception:
            self.logger.warning(f"state 响应解析失败: {resp}")
            return None

    # ======================== 属性 ========================

    @property
    def status(self) -> str:
        return self.data.get("status", "Offline")

    @property
    def position(self) -> List[float]:
        return self.data.get("position", [0.0] * 6)

    @property
    def tcp_pose(self) -> List[float]:
        return self.data.get("tcp_pose", [0.0] * 6)

    @property
    def joint_velocity(self) -> List[float]:
        return self.data.get("joint_velocity", [0.0] * 6)

    @property
    def joint_torque(self) -> List[float]:
        return self.data.get("joint_torque", [0.0] * 6)

    @property
    def tcp_force(self) -> List[float]:
        return self.data.get("tcp_force", [0.0] * 6)

    @property
    def speed(self) -> float:
        return self.data.get("speed", 100.0)

    @property
    def robot_state_name(self) -> str:
        return self.data.get("robot_state_name", "SR_Start")

    @property
    def program_state_name(self) -> str:
        return self.data.get("program_state_name", "SP_Stopped")

    @property
    def operation_mode_name(self) -> str:
        return self.data.get("operation_mode_name", "Unknown")

    @property
    def error_message(self) -> str:
        return self.data.get("error_message", "")

    # ======================== 动作（基于 Telnet 验证格式） ========================

    async def power_on(self) -> str:
        """上电"""
        resp = self._send_cmd("poweron()")
        if self._is_success(resp):
            return "poweron success"
        else:
            return f"poweron fail: {resp or 'no response'}"

    async def power_off(self) -> str:
        """下电"""
        resp = self._send_cmd("poweroff()")
        if self._is_success(resp):
            return "poweroff success"
        else:
            return f"poweroff fail: {resp or 'no response'}"

    async def enable(self) -> str:
        """使能"""
        resp = self._send_cmd("enable()")
        if self._is_success(resp):
            return "enable success"
        else:
            return f"enable fail: {resp or 'no response'}"

    async def disable(self) -> str:
        """去使能"""
        resp = self._send_cmd("disable()")
        if self._is_success(resp):
            return "disable success"
        else:
            return f"disable fail: {resp or 'no response'}"

    async def run_program(self, name: str, speed: float = 0.0) -> str:
        """
        运行示教器工程
        Args:
            name: 工程名称（支持两种格式）
              - 简短名: "20260421" → 自动补全为 "program/20260421.jspf"
              - 完整路径: "program/xxx.jspf"
            speed: 速度百分比 (1-100)，传 0 使用默认速度 70
        """
        # 自动补全路径
        if not name.startswith("program/"):
            name = f"program/{name}.jspf"
        
        speed_val = int(speed) if speed > 0 else 70
        cmd = f'run("{name}",{speed_val})'
        resp = self._send_cmd(cmd)
        if self._is_success(resp):
            return "run success"
        else:
            return f"run fail: {resp or 'no response'}"

    async def pause(self) -> str:
        """暂停当前工程（需确认真实命令）"""
        resp = self._send_cmd("pause()")
        if self._is_success(resp):
            return "pause success"
        else:
            return f"pause fail: {resp or 'no response'}"

    async def resume(self) -> str:
        """恢复暂停的工程（需确认真实命令）"""
        resp = self._send_cmd("resume()")
        if self._is_success(resp):
            return "resume success"
        else:
            return f"resume fail: {resp or 'no response'}"

    async def stop(self) -> str:
        """停止当前工程（需确认真实命令）"""
        resp = self._send_cmd("stop()")
        if self._is_success(resp):
            return "stop success"
        else:
            return f"stop fail: {resp or 'no response'}"

    async def set_speed(self, speed: float) -> str:
        """设置全局速度百分比"""
        spd = int(max(1, min(100, float(speed))))
        resp = self._send_cmd(f"speed({spd})")
        if self._is_success(resp):
            self.data["speed"] = float(spd)
            return f"speed {spd} success"
        else:
            return f"speed fail: {resp or 'no response'}"

    async def query_state(self) -> str:
        """查询机器人状态（返回原始字符串）"""
        return self._send_cmd("state") or "state query fail"

    async def clear_error(self) -> str:
        """清除告警"""
        resp = self._send_cmd("clear()")
        if self._is_success(resp):
            self.data["error_message"] = ""
            return "clear success"
        else:
            return f"clear fail: {resp or 'no response'}"