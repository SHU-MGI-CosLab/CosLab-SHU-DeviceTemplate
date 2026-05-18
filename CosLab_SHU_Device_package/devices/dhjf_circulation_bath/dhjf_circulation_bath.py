"""
DHJF-2005A 低温恒温搅拌反应浴驱动（Modbus RTU, RS485）
修正版本 - 支持 pymodbus 3.x (device_id 参数) + 正确的方法名 + 详细日志

与 Uni‑Lab‑OS temperature 接口对齐：
- 属性：status("Idle"/"Busy"), temp(°C), temp_target(°C), stir_speed(RPM), temp_warning(°C)
- 扩展属性：segment_count, current_segment, run_time_h, run_time_m, low_temp_alarm, over_temp_alarm
"""
import logging
import inspect
from typing import Dict, Any, List, Tuple

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

# 兼容不同版本 pymodbus 的导入
try:
    from pymodbus.client import ModbusSerialClient  # 3.x
except Exception:
    try:
        from pymodbus.client.sync import ModbusSerialClient  # 2.5.x
    except Exception:
        ModbusSerialClient = None


class DHJFCirculationBath:
    """
    DHJF-2005A 低温恒温搅拌反应浴驱动（Modbus RTU, RS485）

    UI 操作示例（在 Web 端动作面板中可以直接调用）：
    - set_temp(25.0) 或 set_temperature(25.0)
    - set_segments(count=3)
    - set_segment(index=1, temperature=5, hours=0, minutes=30)
    - program(segments=[[5,0,30],[10,1,0],[3,0,10]])
    - start(), stop()
    - start_stirring()/stop_stirring(), start_circulation()/stop_circulation()
    - start_heating()/stop_heating(), start_cooling()/stop_cooling()

    温度整数以×100写入（两位小数），例如 25.00°C -> 2500。
    """

    _ros_node: "BaseROS2DeviceNode"

    # --- 寄存器映射（说明书通参表） ---
    REG_MACHINE_TYPE = 0x0000
    REG_SEGMENT_COUNT = 0x0001
    REG_CURRENT_SEGMENT = 0x0002
    REG_MEASURED_TEMP = 0x0003
    REG_DISPLAY_TEMP = 0x0004
    REG_RUN_TIME_H = 0x0005
    REG_RUN_TIME_M = 0x0006

    REG_SEG = {
        1: (0x0007, 0x0008, 0x0009),   # 温度、小时、分钟
        2: (0x000A, 0x000B, 0x000C),
        3: (0x000D, 0x000E, 0x000F),
        4: (0x0010, 0x0011, 0x0012),
        5: (0x0013, 0x0014, 0x0015),
    }

    REG_CTRL = 0x0016
    BIT_POWER_KEY   = 15
    BIT_RUN         = 14
    BIT_STIRRING    = 13
    BIT_CIRCULATION = 12
    BIT_COOL_OUT    = 11  # R（只读输出状态）
    BIT_HEAT_OUT    = 10  # R
    BIT_COOL_KEY    = 9
    BIT_HEAT_KEY    = 8

    REG_ALARM = 0x0017
    BIT_RUN_RESULT = 12
    BIT_RUN_STATE  = 11
    BIT_LOW_ALARM  = 10
    BIT_OVER_ALARM = 9

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        self.device_id = device_id or "dhjf_circulation_bath"
        self.config = config or {}
        self.logger = logging.getLogger(f"DHJFCirculationBath.{self.device_id}")
        self.logger.setLevel(logging.DEBUG)  # 启用详细日志

        self.port = self.config.get('port', 'COM4')
        self.slave_id = int(self.config.get('slave_id', 1))
        self.baudrate = int(self.config.get('baudrate', 9600))
        self.timeout = float(self.config.get('timeout', 1.0))

        self.client = None
        self._connected = False  # 添加连接状态标志
        
        # 预填充所有属性（硬约束3）
        self.data = {
            "status": "Idle",
            "temp": 0.0,
            "temp_target": 0.0,
            "stir_speed": 0.0,
            "temp_warning": 0.0,
            "segment_count": 1,
            "current_segment": 1,
            "run_time_h": 0,
            "run_time_m": 0,
            "low_temp_alarm": False,
            "over_temp_alarm": False,
        }
        
        self.logger.info(f"[INIT] DHJF-2005A 初始化: port={self.port}, slave_id={self.slave_id}, baudrate={self.baudrate}")

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node
        self.logger.info(f"[POST_INIT] ROS node 已设置")

    # --- 兼容 2.5/3.x 的读写封装 ---
    def _detect_param_name(self, func) -> str:
        """检测 pymodbus 版本对应的参数名"""
        sig = inspect.signature(func)
        params = list(sig.parameters.keys())
        self.logger.debug(f"[DETECT] 函数 {func.__name__} 参数: {params}")
        
        if "device_id" in params:
            return "device_id"
        elif "slave" in params:
            return "slave"
        elif "unit" in params:
            return "unit"
        else:
            self.logger.warning(f"[DETECT] 未找到已知参数名，将尝试无参数调用")
            return None

    def _connect(self) -> bool:
        """建立 Modbus 连接"""
        if self._connected and self.client:
            return True
            
        if ModbusSerialClient is None:
            self.logger.error("[CONNECT] 缺少 pymodbus，请安装: pip install pymodbus pyserial")
            return False
        
        self.logger.info(f"[CONNECT] 正在连接串口 {self.port}...")
        self.client = ModbusSerialClient(
            port=self.port, baudrate=self.baudrate,
            bytesize=8, parity='N', stopbits=1, timeout=self.timeout
        )
        ok = bool(self.client.connect())
        if not ok:
            self.logger.error(f"[CONNECT] 串口连接失败: {self.port}")
            self.client = None
            self._connected = False
            return False
        
        self.logger.info(f"[CONNECT] 串口连接成功: {self.port}")
        self._connected = True
        return True

    def _disconnect(self):
        if self.client:
            self.client.close()
            self.client = None
            self._connected = False
            self.logger.info("[DISCONNECT] 串口已关闭")

    def _read_regs(self, addr: int, count: int = 1):
        """读取寄存器 - 兼容 pymodbus 2.x/3.x"""
        if not self._connect():
            self.logger.error(f"[READ] 无法连接串口，读取 0x{addr:04X} 失败")
            return None
        
        fn = self.client.read_holding_registers
        param_name = self._detect_param_name(fn)
        
        kw = {"address": addr, "count": count}
        if param_name:
            kw[param_name] = self.slave_id
        
        self.logger.debug(f"[READ] 调用 read_holding_registers: {kw}")
        
        try:
            res = fn(**kw)
            if hasattr(res, "isError") and res.isError():
                self.logger.error(f"[READ] 读取失败: 0x{addr:04X}, 错误: {res}")
                return None
            regs = getattr(res, "registers", None)
            if regs:
                self.logger.info(f"[READ] 读取成功: 0x{addr:04X} = {regs}")
            return regs
        except Exception as e:
            self.logger.error(f"[READ] 读取异常: 0x{addr:04X}, {e}")
            return None

    def _write_reg(self, addr: int, value: int) -> bool:
        """写入寄存器 - 兼容 pymodbus 2.x/3.x"""
        if not self._connect():
            self.logger.error(f"[WRITE] 无法连接串口，写入 0x{addr:04X} 失败")
            return False
        
        fn = self.client.write_register
        param_name = self._detect_param_name(fn)
        
        val = int(value) & 0xFFFF
        kw = {"address": addr, "value": val}
        if param_name:
            kw[param_name] = self.slave_id
        
        self.logger.debug(f"[WRITE] 调用 write_register: {kw}")
        
        try:
            res = fn(**kw)
            if hasattr(res, "isError") and res.isError():
                self.logger.error(f"[WRITE] 写入失败: 0x{addr:04X}={val}, 错误: {res}")
                return False
            self.logger.info(f"[WRITE] 写入成功: 0x{addr:04X}={val}")
            return True
        except Exception as e:
            self.logger.error(f"[WRITE] 写入异常: 0x{addr:04X}={val}, {e}")
            return False

    def _set_bit(self, addr: int, bit: int, enable: bool) -> bool:
        """设置控制字的某个位"""
        self.logger.info(f"[SET_BIT] 设置 0x{addr:04X} Bit{bit} = {enable}")
        
        regs = self._read_regs(addr, 1)
        if not regs:
            self.logger.error(f"[SET_BIT] 无法读取当前值，设置失败")
            return False
        
        cur = regs[0] & 0xFFFF
        self.logger.debug(f"[SET_BIT] 当前值: 0x{cur:04X} ({cur})")
        
        new_val = (cur | (1 << bit)) if enable else (cur & ~(1 << bit))
        self.logger.debug(f"[SET_BIT] 新值: 0x{new_val:04X} ({new_val})")
        
        ok = self._write_reg(addr, new_val)
        if ok:
            # 验证写入结果
            verify = self._read_regs(addr, 1)
            if verify:
                self.logger.info(f"[SET_BIT] 验证结果: 0x{verify[0]:04X}")
        return ok

    async def _pulse_bit(self, addr: int, bit: int, pulse_sec: float = 0.1) -> bool:
        """按键位脉冲触发：置位→短等待→清零。"""
        ok = self._set_bit(addr, bit, True)
        if not ok:
            return False
        if getattr(self, "_ros_node", None):
            await self._ros_node.sleep(max(pulse_sec, 0.05))
        return self._set_bit(addr, bit, False)

    def _refresh(self):
        t = self._read_regs(self.REG_MEASURED_TEMP, 1)
        if t:
            self.data["temp"] = t[0] * 0.01
        s1t = self._read_regs(self.REG_SEG[1][0], 1)
        if s1t:
            self.data["temp_target"] = s1t[0] * 0.01
        sc = self._read_regs(self.REG_SEGMENT_COUNT, 1)
        if sc:
            self.data["segment_count"] = sc[0]
        cs = self._read_regs(self.REG_CURRENT_SEGMENT, 1)
        if cs:
            self.data["current_segment"] = cs[0]
        hh = self._read_regs(self.REG_RUN_TIME_H, 1)
        if hh:
            self.data["run_time_h"] = hh[0]
        mm = self._read_regs(self.REG_RUN_TIME_M, 1)
        if mm:
            self.data["run_time_m"] = mm[0]
        al = self._read_regs(self.REG_ALARM, 1)
        if al:
            v = al[0]
            self.data["low_temp_alarm"] = bool((v >> self.BIT_LOW_ALARM) & 1)
            self.data["over_temp_alarm"] = bool((v >> self.BIT_OVER_ALARM) & 1)

    # --- 生命周期 ---
    async def initialize(self) -> bool:
        self.logger.info("[INITIALIZE] 开始初始化设备...")
        self.data["status"] = "Busy"
        if not self._connect():
            self.logger.error("[INITIALIZE] 连接失败")
            self.data["status"] = "Idle"
            return False
        self._refresh()
        self.data["status"] = "Idle"
        self.logger.info("[INITIALIZE] 初始化完成")
        return True

    async def cleanup(self) -> bool:
        self._disconnect()
        self.data["status"] = "Idle"
        return True

    # --- 属性（对齐 temperature 接口） ---
    @property
    def status(self) -> str:
        return self.data.get("status", "Idle")

    @property
    def temp(self) -> float:
        return self.data.get("temp", 0.0)

    @property
    def temp_target(self) -> float:
        return self.data.get("temp_target", 0.0)

    @property
    def stir_speed(self) -> float:
        return self.data.get("stir_speed", 0.0)

    @property
    def temp_warning(self) -> float:
        return self.data.get("temp_warning", 0.0)

    # --- 基本动作 ---
    async def set_temp(self, temp: float) -> bool:
        """设置第1段设定温度。示例：set_temp(25.0)
        Args:
            temp: 目标温度（°C）
        """
        self.logger.info(f"[SET_TEMP] 设置温度: {temp}°C")
        result = await self.set_temperature(temp)
        self.logger.info(f"[SET_TEMP] 结果: {result}")
        return result

    async def set_temperature(self, temperature: float) -> bool:
        """设置第1段设定温度（对齐 temperature 接口）。示例：set_temperature(25.0)
        """
        val = int(round(temperature * 100)) & 0xFFFF
        addr = self.REG_SEG[1][0]  # 0x0007
        self.logger.info(f"[SET_TEMPERATURE] 写入寄存器 0x{addr:04X} = {val} (温度 {temperature}°C)")
        ok = self._write_reg(addr, val)
        if ok:
            self.data["temp_target"] = float(temperature)
            # 验证写入
            verify = self._read_regs(addr, 1)
            if verify:
                actual = verify[0] * 0.01
                self.logger.info(f"[SET_TEMPERATURE] 验证读回: {actual}°C (原始值 {verify[0]})")
        return ok

    async def start(self) -> bool:
        """开始程序运行。示例：start()
        """
        self.data["status"] = "Busy"
        ok = self._set_bit(self.REG_CTRL, self.BIT_RUN, True)
        self.data["status"] = "Idle"
        return ok

    async def stop(self) -> bool:
        """停止程序运行。示例：stop()
        """
        self.data["status"] = "Busy"
        ok = self._set_bit(self.REG_CTRL, self.BIT_RUN, False)
        self.data["status"] = "Idle"
        return ok

    async def start_stirring(self) -> bool:
        """开启搅拌。示例：start_stirring()
        """
        return self._set_bit(self.REG_CTRL, self.BIT_STIRRING, True)

    async def stop_stirring(self) -> bool:
        """关闭搅拌。示例：stop_stirring()
        """
        return self._set_bit(self.REG_CTRL, self.BIT_STIRRING, False)

    async def start_circulation(self) -> bool:
        """开启循环。示例：start_circulation()
        """
        self.logger.info("[START_CIRCULATION] 开启循环")
        result = self._set_bit(self.REG_CTRL, self.BIT_CIRCULATION, True)
        self.logger.info(f"[START_CIRCULATION] 结果: {result}")
        return result

    async def stop_circulation(self) -> bool:
        """关闭循环。示例：stop_circulation()
        """
        self.logger.info("[STOP_CIRCULATION] 关闭循环")
        result = self._set_bit(self.REG_CTRL, self.BIT_CIRCULATION, False)
        self.logger.info(f"[STOP_CIRCULATION] 结果: {result}")
        return result

    async def start_heating(self) -> bool:
        """按下加热键（按键脉冲）。示例：start_heating()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_HEAT_KEY, 0.1)

    async def stop_heating(self) -> bool:
        """松开加热键。示例：stop_heating()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_HEAT_KEY, 0.1)

    async def start_cooling(self) -> bool:
        """按下制冷键（按键脉冲）。示例：start_cooling()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_COOL_KEY, 0.1)

    async def stop_cooling(self) -> bool:
        """松开制冷键。示例：stop_cooling()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_COOL_KEY, 0.1)

    async def power_on(self) -> bool:
        """按电源键（按键脉冲）。示例：power_on()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_POWER_KEY, 0.1)

    async def power_off(self) -> bool:
        """按电源键关闭（按键脉冲）。示例：power_off()
        """
        return await self._pulse_bit(self.REG_CTRL, self.BIT_POWER_KEY, 0.1)

    # --- 多段程序动作 ---
    async def set_segments(self, count: int) -> bool:
        """设置程序段数（1~5）。示例：set_segments(3)
        """
        if count < 1 or count > 5:
            return False
        ok = self._write_reg(self.REG_SEGMENT_COUNT, int(count))
        if ok:
            self.data["segment_count"] = int(count)
        return ok

    async def set_segment(self, index: int, temperature: float, hours: int, minutes: int) -> bool:
        """设置指定段的温度与时长。示例：set_segment(2, 10.0, 1, 0)
        """
        if index not in self.REG_SEG or hours < 0 or minutes < 0 or minutes > 59:
            return False
        reg_t, reg_h, reg_m = self.REG_SEG[index]
        ok = True
        ok &= self._write_reg(reg_t, int(round(temperature * 100)) & 0xFFFF)
        ok &= self._write_reg(reg_h, int(hours) & 0xFFFF)
        ok &= self._write_reg(reg_m, int(minutes) & 0xFFFF)
        if index == 1 and ok:
            self.data["temp_target"] = float(temperature)
        return ok

    async def program(self, segments: List[Tuple[float, int, int]]) -> bool:
        """批量设置 1~5 段程序。示例：program([[5,0,30],[10,1,0],[3,0,10]])
        """
        n = len(segments)
        if n < 1 or n > 5:
            return False
        ok = await self.set_segments(n)
        for i, triplet in enumerate(segments, start=1):
            t, h, m = float(triplet[0]), int(triplet[1]), int(triplet[2])
            ok &= await self.set_segment(i, t, h, m)
        return ok