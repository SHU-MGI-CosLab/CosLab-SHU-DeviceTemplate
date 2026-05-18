"""
兰格 BT100-2J 蠕动泵驱动
通信协议: RS485, 1200bps, 8N1E (8data + 1even parity + 1stop)

帧格式: flag(E9) + addr + len + PDU + FCS
字节填充: flag 之后，E8→E8 00, E9→E8 01 (len/FCS 按原始数据计算)
"""
import logging
import time as time_module
from typing import Dict, Any, Optional

import serial

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None


class LongerBT100:
    """兰格 BT100-2J 蠕动泵驱动 (WJ/RJ ASCII 协议)"""
    _ros_node: "BaseROS2DeviceNode"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')
        self.device_id = device_id or "longer_bt100"
        self.config = config or {}
        self.port = self.config.get("port", "COM4")
        self.baudrate = self.config.get("baudrate", 1200)
        self.address = self.config.get("address", 1)
        # 1200bps 下一帧约百毫秒量级，略放大避免 read 截断
        self._serial_timeout = float(self.config.get("serial_timeout", 0.5))
        self.logger = logging.getLogger(f"LongerBT100.{self.device_id}")
        self.ser: Optional[serial.Serial] = None

        # 内部状态跟踪
        self._current_speed = 0.0      # 当前转速 (RPM)
        self._current_direction = 1    # 当前方向 (1=顺时针CW, 0=逆时针CCW)
        self._current_fullspeed = False # 当前是否全速模式

        # 预填充所有属性字段（硬约束）
        self.data = {
            "status": "Idle",
            "speed": 0.0,
            "direction": "CW",
            "is_fullspeed": False
        }

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    async def initialize(self) -> bool:
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                parity=serial.PARITY_EVEN,  # 协议要求: 8data + 1even parity + 1stop
                stopbits=serial.STOPBITS_ONE,
                timeout=self._serial_timeout,
            )
            self.data["status"] = "Idle"
            self.logger.info(f"Connected to BT100-2J on {self.port} at {self.baudrate}bps")
            return True
        except Exception as e:
            self.logger.error(f"Serial Error: {e}")
            self.data["status"] = "Error"
            return False

    async def cleanup(self) -> bool:
        """关闭串口连接"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.data["status"] = "Offline"
            return True
        except Exception as e:
            self.logger.error(f"Cleanup Error: {e}")
            return False

    # ========== 字节填充/解填充 ==========

    def _escape_encode(self, data: bytes) -> bytes:
        """编码: E8→E8 00, E9→E8 01 (应用于 flag 之后的内容)"""
        result = bytearray()
        for b in data:
            if b == 0xE8:
                result.extend([0xE8, 0x00])
            elif b == 0xE9:
                result.extend([0xE8, 0x01])
            else:
                result.append(b)
        return bytes(result)

    def _escape_decode(self, data: bytes) -> bytes:
        """解码: E8 00→E8, E8 01→E9"""
        result = bytearray()
        i = 0
        while i < len(data):
            if i + 1 < len(data) and data[i] == 0xE8:
                if data[i + 1] == 0x00:
                    result.append(0xE8)
                    i += 2
                elif data[i + 1] == 0x01:
                    result.append(0xE9)
                    i += 2
                else:
                    result.append(data[i])
                    i += 1
            else:
                result.append(data[i])
                i += 1
        return bytes(result)

    # ========== 帧构建/解析 ==========

    def _calc_fcs(self, addr: int, pdu: bytes) -> int:
        """计算校验字节 FCS: addr ^ len(pdu) ^ pdu 各字节异或"""
        xor_val = addr ^ len(pdu)
        for b in pdu:
            xor_val ^= b
        return xor_val & 0xFF

    def _build_frame(self, pdu: bytes) -> bytes:
        """构建完整帧: flag(E9) + escaped(addr + len + pdu + fcs)"""
        # 计算原始 FCS（按未填充数据）
        fcs = self._calc_fcs(self.address, pdu)
        # 原始数据: addr + len + pdu + fcs
        raw_payload = bytes([self.address, len(pdu)]) + pdu + bytes([fcs])
        # 对 flag 之后的内容进行字节填充
        escaped_payload = self._escape_encode(raw_payload)
        # 完整帧
        return bytes([0xE9]) + escaped_payload

    def _read_exact(self, n: int) -> Optional[bytes]:
        """读取精确 n 字节，超时返回 None"""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _read_response_frame(self, timeout_extra: float = 0.2) -> Optional[bytes]:
        """读取应答帧: 同步 0xE9 帧头，然后读取并解填充"""
        if not self.ser or not self.ser.is_open:
            return None

        # 同步帧头
        start_time = time_module.time()
        while time_module.time() - start_time < timeout_extra + self._serial_timeout:
            b = self.ser.read(1)
            if not b:
                self.logger.debug("sync 0xE9: timeout")
                return None
            if b[0] == 0xE9:
                break
        else:
            self.logger.warning("sync 0xE9: 超时未找到帧头")
            return None

        # 读取后续数据（先读 addr + len，再根据 len 读剩余）
        # 注意：由于字节填充，物理长度可能大于逻辑长度
        # 策略：读取足够多的字节，然后解填充
        raw_buf = bytearray()
        max_read = 64  # 读取最多 64 字节
        start_time = time_module.time()
        while len(raw_buf) < max_read:
            b = self.ser.read(1)
            if not b:
                break
            raw_buf.append(b[0])
            # 检查是否已读取完整帧（解填充后）
            decoded = self._escape_decode(bytes(raw_buf))
            if len(decoded) >= 3:
                addr, pdu_len = decoded[0], decoded[1]
                expected_total = 3 + pdu_len + 1  # addr(1) + len(1) + pdu(pdu_len) + fcs(1)
                if len(decoded) >= expected_total:
                    return bytes([0xE9]) + bytes(raw_buf[:len(decoded) + (len(raw_buf) - len(decoded) + 1) // 1])
            if time_module.time() - start_time > timeout_extra + self._serial_timeout:
                break

        # 尝试解填充
        decoded = self._escape_decode(bytes(raw_buf))
        if len(decoded) >= 3:
            return bytes([0xE9]) + bytes(raw_buf)
        return None

    def _parse_response(self, raw_frame: bytes) -> Optional[dict]:
        """解析应答帧，返回解填充后的帧数据"""
        if not raw_frame or len(raw_frame) < 5:
            return None
        # 跳过帧头 0xE9，解填充剩余内容
        # payload 格式: addr(1) + len(1) + PDU(len) + FCS(1)
        payload = self._escape_decode(raw_frame[1:])
        if len(payload) < 3:
            return None
        addr, pdu_len = payload[0], payload[1]
        # 需要的总长度: addr(1) + len(1) + PDU(pdu_len) + FCS(1) = 3 + pdu_len
        if len(payload) < 3 + pdu_len:
            self.logger.warning(f"Payload 长度不足: got {len(payload)}, need {3 + pdu_len}")
            return None
        pdu = payload[2:2 + pdu_len]
        fcs = payload[2 + pdu_len]
        # 校验 FCS
        expected_fcs = self._calc_fcs(addr, pdu)
        if fcs != expected_fcs:
            self.logger.warning(f"FCS 校验失败: got 0x{fcs:02X}, expected 0x{expected_fcs:02X}")
            return None
        return {
            "addr": addr,
            "pdu": pdu,
            "fcs": fcs
        }

    # ========== 命令发送 ==========

    def _send_wj_command(self, speed: float, run_state: int, dir_state: int, fullspeed: int = 0) -> Optional[bytes]:
        """发送 WJ 设置运行参数命令

        PDU格式: WJ(57 4A) + 转速(2字节,高位在前,单位0.1RPM) + 全速启停(1字节) + 方向(1字节)
        
        参数:
            speed: 转速 (RPM)
            run_state: 启停状态 (1=运行, 0=停止)
            dir_state: 方向状态 (1=顺时针CW, 0=逆时针CCW)
            fullspeed: 全速标志 (1=全速, 0=正常)

        示例 (50.0 RPM 正常顺时针运行):
            E9 01 06 57 4A 01 F4 01 01 EF
        """
        # 转速转换为 0.1 RPM 单位 (高位在前)
        speed_val = int(speed * 10)
        speed_hi = (speed_val >> 8) & 0xFF
        speed_lo = speed_val & 0xFF

        # 全速/启停状态字节: Bit0=启停, Bit1=全速
        status_byte = (run_state & 0x01) | ((fullspeed & 0x01) << 1)

        # 方向状态字节: Bit0=方向 (1=CW, 0=CCW)
        dir_byte = dir_state & 0x01

        # 构建 PDU: WJ + 转速 + 状态 + 方向
        pdu = bytearray([0x57, 0x4A, speed_hi, speed_lo, status_byte, dir_byte])

        # 构建完整帧
        frame = self._build_frame(pdu)
        
        self.logger.info(f"[TX] Frame: {frame.hex()} (speed={speed}RPM, run={run_state}, dir={dir_state}, fullspeed={fullspeed})")
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        # 广播地址(31)无应答
        if self.address == 31:
            self.logger.info("广播地址，无应答")
            return None

        response = self._read_response_frame()
        if response:
            self.logger.info(f"[RX] Response: {response.hex()}")
        else:
            self.logger.warning("[RX] 无有效应答帧")
        return response

    def _send_rj_command(self) -> Optional[dict]:
        """发送 RJ 读取运行参数命令

        PDU格式: RJ(52 4A)
        响应格式: RJ + 转速(2字节) + 全速启停(1字节) + 方向(1字节)

        示例:
            发送: E9 01 02 52 4A 1B
            响应: E9 01 06 52 4A 02 E8 01 01 01 F4 (解码后)
                  转速=0x02E8=740→74.0RPM, 启停=01运行, 全速=0正常, 方向=01顺时针
        """
        pdu = bytearray([0x52, 0x4A])
        frame = self._build_frame(pdu)

        self.logger.info(f"[TX] RJ Query: {frame.hex()}")
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        response = self._read_response_frame()
        if not response:
            self.logger.warning("[RX] RJ 无有效应答帧")
            return None
        self.logger.info(f"[RX] Response: {response.hex()}")

        parsed = self._parse_response(response)
        if not parsed:
            return None

        pdu_resp = parsed["pdu"]
        # PDU 格式: RJ(52 4A) + 转速2字节 + 状态1字节 + 方向1字节
        if len(pdu_resp) < 6:
            self.logger.warning(f"RJ 响应 PDU 长度不足: {len(pdu_resp)}")
            return None

        speed_val = (pdu_resp[2] << 8) | pdu_resp[3]
        speed = speed_val / 10.0
        status_byte = pdu_resp[4]
        dir_byte = pdu_resp[5]

        run_state = status_byte & 0x01          # Bit0: 启停
        fullspeed = (status_byte >> 1) & 0x01    # Bit1: 全速
        direction = "CW" if (dir_byte & 0x01) else "CCW"

        return {
            "speed": speed,
            "running": run_state == 1,
            "fullspeed": fullspeed == 1,
            "direction": direction
        }

    # ========== 动作方法 (Uni-Lab-OS 标准接口) ==========

    async def start(self, **kwargs):
        """启动泵转动（使用当前转速和方向）"""
        self._send_wj_command(
            self._current_speed,
            1,  # 运行
            self._current_direction,
            1 if self._current_fullspeed else 0
        )
        self.data["status"] = "Busy"
        self.logger.info(f"Pump {self.device_id} STARTED at {self._current_speed} RPM")

    async def stop(self, **kwargs):
        """停止泵转动"""
        self._send_wj_command(
            self._current_speed,
            0,  # 停止
            self._current_direction,
            0
        )
        self.data["status"] = "Idle"
        self.logger.info(f"Pump {self.device_id} STOPPED")

    async def set_speed(self, speed: float, **kwargs):
        """设置转速 (单位: RPM)

        注意:
        - 如果泵正在运行，会立即改变转速并继续运行
        - 如果泵停止，只更新内部转速值，下次 start() 时使用
        """
        self._current_speed = float(speed)

        # 如果泵正在运行，立即更新转速
        if self.data["status"] == "Busy":
            self._send_wj_command(
                self._current_speed,
                1,
                self._current_direction,
                1 if self._current_fullspeed else 0
            )

        self.data["speed"] = self._current_speed
        self.logger.info(f"Pump {self.device_id} SPEED set to {self._current_speed} RPM")

    async def set_direction(self, direction: str, **kwargs):
        """设置转动方向

        参数:
            direction: "CW" (顺时针) 或 "CCW" (逆时针)
        """
        if direction.upper() == "CW":
            self._current_direction = 1
            self.data["direction"] = "CW"
        elif direction.upper() == "CCW":
            self._current_direction = 0
            self.data["direction"] = "CCW"
        else:
            self.logger.warning(f"Invalid direction: {direction}, using CW")
            self._current_direction = 1
            self.data["direction"] = "CW"

        # 如果泵正在运行，立即更新方向
        if self.data["status"] == "Busy":
            self._send_wj_command(
                self._current_speed,
                1,
                self._current_direction,
                1 if self._current_fullspeed else 0
            )

        self.logger.info(f"Pump {self.device_id} DIRECTION set to {self.data['direction']}")

    async def set_fullspeed(self, enable: bool, **kwargs):
        """设置全速模式

        参数:
            enable: True 启用全速模式，False 正常模式
        """
        self._current_fullspeed = bool(enable)
        self.data["is_fullspeed"] = self._current_fullspeed

        # 如果泵正在运行，立即更新
        if self.data["status"] == "Busy":
            self._send_wj_command(
                self._current_speed,
                1,
                self._current_direction,
                1 if self._current_fullspeed else 0
            )

        self.logger.info(f"Pump {self.device_id} FULLSPEED set to {self._current_fullspeed}")

    async def read_status(self, **kwargs):
        """读取泵的当前运行状态"""
        result = self._send_rj_command()
        if result:
            self.data["speed"] = result["speed"]
            self.data["status"] = "Busy" if result["running"] else "Idle"
            self.data["direction"] = result["direction"]
            self.data["is_fullspeed"] = result["fullspeed"]
            self._current_speed = result["speed"]
            self._current_direction = 1 if result["direction"] == "CW" else 0
            self._current_fullspeed = result["fullspeed"]
            self.logger.info(
                f"Pump status: speed={result['speed']}RPM, "
                f"running={result['running']}, dir={result['direction']}, "
                f"fullspeed={result['fullspeed']}"
            )
        return result

    async def run_for_duration(self, duration: float, speed: float = None, direction: str = None, **kwargs):
        """定时运行泵

        启动泵运行指定时间后自动停止。

        参数:
            duration: 运行时间（秒）
            speed: 转速 (RPM)，可选，不指定则使用当前设置
            direction: 方向 ("CW"/"CCW")，可选，不指定则使用当前设置

        示例:
            run_for_duration(duration=30, speed=50.0, direction="CW")
            run_for_duration(duration=60)  # 使用当前设置运行60秒
        """
        import asyncio

        # 设置参数（如果提供）
        if speed is not None:
            await self.set_speed(speed)
        if direction is not None:
            await self.set_direction(direction)

        # 记录开始时间
        self.logger.info(f"Pump {self.device_id} starting timed run: {duration}s at {self._current_speed} RPM {self.data['direction']}")

        # 启动泵
        await self.start()

        # 等待指定时间（使用 _ros_node.sleep 避免阻塞事件循环）
        if self._ros_node is not None:
            await self._ros_node.sleep(duration)
        else:
            # 回退：单独运行时使用 asyncio.sleep
            await asyncio.sleep(duration)

        # 停止泵
        await self.stop()

        self.logger.info(f"Pump {self.device_id} completed timed run of {duration} seconds")
        return {
            "duration": duration,
            "speed": self._current_speed,
            "direction": self.data["direction"],
            "status": "completed"
        }

    # ========== 属性定义 (Uni-Lab-OS 标准接口) ==========

    @property
    def status(self) -> str:
        """泵状态: Idle / Busy / Error"""
        return self.data.get("status", "Idle")

    @property
    def speed(self) -> float:
        """当前转速 (RPM)"""
        return self.data.get("speed", 0.0)

    @property
    def direction(self) -> str:
        """转动方向: CW (顺时针) / CCW (逆时针)"""
        return self.data.get("direction", "CW")

    @property
    def is_fullspeed(self) -> bool:
        """是否全速运行"""
        return self.data.get("is_fullspeed", False)


# ========== 独立测试入口 ==========

def _main():
    import argparse
    import asyncio

    parser = argparse.ArgumentParser(
        description="单独测试兰格 BT100-2J 驱动"
    )
    parser.add_argument("--port", default="COM4", help="串口，如 COM4 或 /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1200)
    parser.add_argument("--address", type=int, default=1)
    parser.add_argument("--device-id", default="test_bt100", dest="device_id")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument(
        "--demo-run",
        action="store_true",
        help="执行短时 run→sleep→stop 演示（注意安全）"
    )
    parser.add_argument("--demo-speed", type=float, default=50.0, dest="demo_speed")
    parser.add_argument("--demo-direction", choices=("CW", "CCW"), default="CW", dest="demo_direction")
    parser.add_argument("--demo-seconds", type=float, default=3.0, dest="demo_seconds")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    async def run():
        config = {
            "port": args.port,
            "baudrate": args.baudrate,
            "address": args.address,
        }
        pump = LongerBT100(device_id=args.device_id, config=config)
        if not await pump.initialize():
            return 1
        try:
            print("=" * 50)
            print("读取泵状态...")
            st = await pump.read_status()
            print(f"read_status: {st}")
            print(f"pump.data: {pump.data}")

            if args.demo_run:
                print("=" * 50)
                print(f"演示: 设置转速 {args.demo_speed} RPM, 方向 {args.demo_direction}")
                await pump.set_speed(args.demo_speed)
                await pump.set_direction(args.demo_direction)
                print("启动泵...")
                await pump.start()
                print(f"运行 {args.demo_seconds} 秒...")
                await asyncio.sleep(args.demo_seconds)
                print("停止泵...")
                await pump.stop()
                st2 = await pump.read_status()
                print(f"演示后状态: {st2}")
        finally:
            await pump.cleanup()
        return 0

    sys.exit(asyncio.run(run()))


if __name__ == "__main__":
    import sys
    _main()