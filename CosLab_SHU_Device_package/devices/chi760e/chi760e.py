"""
辰华 CHI760E 电化学工作站驱动
基于 hardpotato 开源项目的宏命令（Macro）控制方案

控制链路：
  Python → 生成 .mcr 宏命令文件 → subprocess 启动 chi760e.exe /runmacro → 读取 .txt 数据文件

支持技术：CV / LSV / CA / OCP / NPV / EIS

参考：
  - hardpotato: https://github.com/jrlLAB/hardpotato
  - CHI Macro 格式: c\\x02\\0\\0\\n 头部 + 参数行 + run + save + forcequit
"""

import logging
import os
import json
import time as time_module
import subprocess
from datetime import datetime
from typing import Dict, Any, Optional

try:
    import numpy as np
except ImportError:
    np = None

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None


class CHI760E:
    """辰华 CHI760E 电化学工作站

    通过 CHI 软件的宏命令（Macro）机制控制仪器。
    每次实验：生成 .mcr → subprocess 调用 chi760e.exe /runmacro → 等待完成 → 解析数据文件。
    """

    _ros_node: "BaseROS2DeviceNode"

    # ==================== 初始化 ====================

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and 'id' in kwargs:
            device_id = kwargs.pop('id')
        if config is None and 'config' in kwargs:
            config = kwargs.pop('config')

        self.device_id = device_id or "chi760e"
        self.config = config or {}
        self.logger = logging.getLogger(f"CHI760E.{self.device_id}")

        # self.data 预填充所有 @property 字段
        self.data = {
            "status": "Idle",
            "technique": "",
            "last_data_file": "",
            "last_experiment_time": "",
            "data_folder": "",
        }

        # 配置参数（从 config 和 kwargs 双重读取）
        self._chi_exe_path = (
            self.config.get("chi_exe_path")
            or kwargs.get("chi_exe_path", "")
        )
        self._data_folder = (
            self.config.get("data_folder")
            or kwargs.get("data_folder", "")
        )
        self._default_sens = float(
            self.config.get("default_sens")
            or kwargs.get("default_sens", 1e-6)
        )

        self.data["data_folder"] = self._data_folder

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        self._ros_node = ros_node

    async def initialize(self) -> bool:
        """初始化：验证 CHI 软件路径和数据目录"""
        # 验证 CHI 软件路径
        if not self._chi_exe_path:
            self.logger.error("chi_exe_path not configured")
            self.data["status"] = "Error"
            return False

        if not os.path.isfile(self._chi_exe_path):
            self.logger.error(f"CHI software not found: {self._chi_exe_path}")
            self.data["status"] = "Error"
            return False

        # 创建数据目录
        if self._data_folder:
            os.makedirs(self._data_folder, exist_ok=True)
        else:
            self.logger.error("data_folder not configured")
            self.data["status"] = "Error"
            return False

        self.data["status"] = "Idle"
        self.logger.info(
            f"Initialized: exe={self._chi_exe_path}, data={self._data_folder}"
        )
        return True

    async def cleanup(self) -> bool:
        """清理"""
        self.data["status"] = "Offline"
        self.logger.info("Cleanup complete")
        return True

    # ==================== 宏命令核心 ====================

    def _generate_macro_header(self, header_text: str) -> str:
        """生成 CHI 宏命令文件头部（与 hardpotato 完全一致）"""
        folder = self._data_folder.replace("\\", "/")
        head = (
            "c\x02\0\0\n"
            f"folder: {folder}\n"
            "fileoverride\n"
            f"header: {header_text}\n\n"
        )
        return head

    def _generate_macro_footer(self, file_name: str) -> str:
        """生成宏命令文件尾部"""
        foot = (
            f"\nrun\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            " forcequit: yesiamsure\n"
        )
        return foot

    def _write_macro(self, macro_text: str, file_name: str) -> str:
        """写入 .mcr 宏命令文件，返回文件完整路径"""
        mcr_path = os.path.join(self._data_folder, f"{file_name}.mcr")
        with open(mcr_path, "wb") as f:
            f.write(macro_text.encode("ascii"))
        self.logger.debug(f"Macro written: {mcr_path}")
        return mcr_path

    def _run_macro(self, mcr_path: str) -> bool:
        """调用 CHI 软件执行宏命令（阻塞等待完成）"""
        exe = self._chi_exe_path.replace("\\", "/")
        mcr = mcr_path.replace("\\", "/")
        command = f'"{exe}" /runmacro:"{mcr}"'
        self.logger.info(f"Executing: {command}")
        try:
            result = subprocess.run(
                command,
                shell=True,
                timeout=3600,  # 最长 1 小时
                capture_output=True,
            )
            self.logger.info(f"CHI exited with code {result.returncode}")
            return True
        except subprocess.TimeoutExpired:
            self.logger.error("CHI software execution timed out (1h)")
            return False
        except Exception as e:
            self.logger.error(f"Failed to execute CHI software: {e}")
            return False

    def _save_metadata(self, file_name: str, technique: str, params: dict):
        """保存实验元数据 JSON"""
        meta = {
            "device_id": self.device_id,
            "technique": technique,
            "timestamp": datetime.now().isoformat(),
            "parameters": params,
            "data_file": f"{file_name}.txt",
            "macro_file": f"{file_name}.mcr",
        }
        meta_path = os.path.join(self._data_folder, f"{file_name}_meta.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        self.logger.debug(f"Metadata saved: {meta_path}")

    def _parse_data_file(self, file_name: str, search_text: str) -> dict:
        """解析 CHI 输出的 .txt 数据文件

        CHI 数据文件格式：
        - 头部包含实验参数
        - 数据起始行包含 search_text（如 "Potential/V," 或 "Time/sec,"）
        - 数据行为 CSV 格式
        """
        file_path = os.path.join(self._data_folder, f"{file_name}.txt")
        if not os.path.isfile(file_path):
            self.logger.warning(f"Data file not found: {file_path}")
            return {"raw_file": file_path, "data": []}

        result = {"raw_file": file_path, "header_lines": [], "data": []}

        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()

            # 找到数据起始行
            skip_rows = 0
            for i, line in enumerate(lines):
                if search_text in line:
                    skip_rows = i + 1
                    break

            # 保存 header
            result["header_lines"] = [l.strip() for l in lines[:skip_rows]]

            # 解析数据
            if np is not None and skip_rows > 0:
                try:
                    data = np.loadtxt(file_path, delimiter=",", skiprows=skip_rows)
                    result["data"] = data.tolist()
                    result["shape"] = list(data.shape)
                except Exception as e:
                    self.logger.warning(f"numpy loadtxt failed: {e}")
                    # 回退到手动解析
                    for line in lines[skip_rows:]:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            parts = line.split(",")
                            try:
                                row = [float(x) for x in parts if x.strip()]
                                result["data"].append(row)
                            except ValueError:
                                continue
            else:
                # 无 numpy 时手动解析
                for line in lines[skip_rows:]:
                    line = line.strip()
                    if line and not line.startswith("#"):
                        parts = line.split(",")
                        try:
                            row = [float(x) for x in parts if x.strip()]
                            result["data"].append(row)
                        except ValueError:
                            continue

        except Exception as e:
            self.logger.error(f"Failed to parse data file: {e}")

        return result

    def _generate_timestamp_filename(self, technique: str) -> str:
        """生成带时间戳的文件名"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{technique}_{ts}"

    # ==================== 电化学技术 ====================

    async def run_cv(self, ei: float = -0.2, ev1: float = 0.2, ev2: float = -0.2,
                     ef: float = -0.2, sr: float = 0.1, de: float = 0.001,
                     n_sweeps: float = 2.0, sens: float = 0.0,
                     qt: float = 2.0, resistance: float = 0.0,
                     **kwargs) -> str:
        """运行循环伏安法 (Cyclic Voltammetry)

        Args:
            ei: 初始电位 (V)
            ev1: 第一顶点电位 (V)
            ev2: 第二顶点电位 (V)
            ef: 终止电位 (V)
            sr: 扫描速率 (V/s)
            de: 采样间隔 (V)
            n_sweeps: 扫描圈数
            sens: 灵敏度 (A/V)，0 则使用默认值
            qt: 静置时间 (s)
            resistance: 溶液电阻 (Ω)，用于 IR 补偿，0 为不补偿
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "CV"

        if sens == 0.0:
            sens = self._default_sens
        n_sweeps_int = int(n_sweeps) + 1  # CHI 的 final E 默认开启，需 +1

        # 确定扫描方向
        if ev1 > ev2:
            eh, el, pn = ev1, ev2, "p"
        else:
            eh, el, pn = ev2, ev1, "n"

        file_name = self._generate_timestamp_filename("CV")
        header = f"CV ei={ei} ev1={ev1} ev2={ev2} ef={ef} sr={sr}"

        # 构建宏命令
        head = self._generate_macro_header(header)
        body = (
            f"tech=cv\n"
            f"ei={ei}\n"
            f"eh={eh}\n"
            f"el={el}\n"
            f"pn={pn}\n"
            f"cl={n_sweeps_int}\n"
            f"efon\n"
            f"ef={ef}\n"
            f"si={de}\n"
            f"qt={qt}\n"
            f"v={sr}\n"
            f"sens={sens}"
        )
        if resistance:
            body += f"\nmir={resistance}\nircompon"
            foot_extra = "\nircompoff"
        else:
            foot_extra = ""
        foot = (
            f"\nrun"
            f"{foot_extra}\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f" forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        # 执行
        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ei=ei, ev1=ev1, ev2=ev2, ef=ef, sr=sr, de=de,
                      n_sweeps=int(n_sweeps), sens=sens, qt=qt, resistance=resistance)
        self._save_metadata(file_name, "CV", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"CV completed: {data_file}")
        return data_file

    async def run_lsv(self, ei: float = -0.2, ef: float = 0.2,
                      sr: float = 0.1, de: float = 0.001,
                      sens: float = 0.0, qt: float = 2.0,
                      resistance: float = 0.0,
                      **kwargs) -> str:
        """运行线性扫描伏安法 (Linear Sweep Voltammetry)

        Args:
            ei: 初始电位 (V)
            ef: 终止电位 (V)
            sr: 扫描速率 (V/s)
            de: 采样间隔 (V)
            sens: 灵敏度 (A/V)
            qt: 静置时间 (s)
            resistance: 溶液电阻 (Ω)
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "LSV"

        if sens == 0.0:
            sens = self._default_sens

        file_name = self._generate_timestamp_filename("LSV")
        header = f"LSV ei={ei} ef={ef} sr={sr}"

        head = self._generate_macro_header(header)
        body = (
            f"tech=lsv\n"
            f"ei={ei}\n"
            f"ef={ef}\n"
            f"v={sr}\n"
            f"si={de}\n"
            f"qt={qt}\n"
            f"sens={sens}"
        )
        if resistance:
            body += f"\nmir={resistance}\nircompon"
            foot_extra = "\nircompoff"
        else:
            foot_extra = ""
        foot = (
            f"\nrun"
            f"{foot_extra}\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f" forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ei=ei, ef=ef, sr=sr, de=de, sens=sens, qt=qt, resistance=resistance)
        self._save_metadata(file_name, "LSV", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"LSV completed: {data_file}")
        return data_file

    async def run_ca(self, ei: float = 0.2, dt: float = 0.001,
                     ttot: float = 2.0, sens: float = 0.0,
                     qt: float = 2.0, resistance: float = 0.0,
                     **kwargs) -> str:
        """运行计时安培法 (Chronoamperometry)

        Args:
            ei: 阶跃电位 (V)
            dt: 采样间隔 (s)
            ttot: 总时间 (s)
            sens: 灵敏度 (A/V)
            qt: 静置时间 (s)
            resistance: 溶液电阻 (Ω)
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "CA"

        if sens == 0.0:
            sens = self._default_sens

        file_name = self._generate_timestamp_filename("CA")
        header = f"CA ei={ei} ttot={ttot}"

        head = self._generate_macro_header(header)
        body = (
            f"tech=i-t\n"
            f"ei={ei}\n"
            f"st={ttot}\n"
            f"si={dt}\n"
            f"qt={qt}\n"
            f"sens={sens}"
        )
        if resistance:
            body += f"\nmir={resistance}\nircompon"
            foot_extra = "\nircompoff"
        else:
            foot_extra = ""
        foot = (
            f"\nrun"
            f"{foot_extra}\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f" forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ei=ei, dt=dt, ttot=ttot, sens=sens, qt=qt, resistance=resistance)
        self._save_metadata(file_name, "CA", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"CA completed: {data_file}")
        return data_file

    async def run_ocp(self, ttot: float = 10.0, dt: float = 0.01,
                      qt: float = 2.0,
                      **kwargs) -> str:
        """运行开路电位 (Open Circuit Potential)

        Args:
            ttot: 总时间 (s)
            dt: 采样间隔 (s)
            qt: 静置时间 (s)
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "OCP"

        file_name = self._generate_timestamp_filename("OCP")
        header = f"OCP ttot={ttot}"

        head = self._generate_macro_header(header)
        body = (
            f"tech=ocpt\n"
            f"st={ttot}\n"
            f"eh=10\n"
            f"el=-10\n"
            f"si={dt}\n"
            f"qt={qt}"
        )
        foot = (
            f"\nrun\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f"forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ttot=ttot, dt=dt, qt=qt)
        self._save_metadata(file_name, "OCP", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"OCP completed: {data_file}")
        return data_file

    async def run_npv(self, ei: float = 0.5, ef: float = -0.5,
                      de: float = 0.01, pw: float = 0.1,
                      sw: float = 0.05, prod: float = 10.0,
                      sens: float = 0.0, qt: float = 2.0,
                      **kwargs) -> str:
        """运行常规脉冲伏安法 (Normal Pulse Voltammetry)

        Args:
            ei: 初始电位 (V)
            ef: 终止电位 (V)
            de: 电位增量 (V)
            pw: 脉冲宽度 (s)
            sw: 采样宽度 (s)
            prod: 脉冲周期 (s)
            sens: 灵敏度 (A/V)
            qt: 静置时间 (s)
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "NPV"

        if sens == 0.0:
            sens = self._default_sens

        file_name = self._generate_timestamp_filename("NPV")
        header = f"NPV ei={ei} ef={ef}"

        head = self._generate_macro_header(header)
        body = (
            f"tech=NPV\n"
            f"ei={ei}\n"
            f"ef={ef}\n"
            f"incre={de}\n"
            f"pw={pw}\n"
            f"sw={sw}\n"
            f"prod={prod}\n"
            f"qt={qt}\n"
            f"sens={sens}"
        )
        foot = (
            f"\nrun\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f" forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ei=ei, ef=ef, de=de, pw=pw, sw=sw, prod=prod, sens=sens, qt=qt)
        self._save_metadata(file_name, "NPV", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"NPV completed: {data_file}")
        return data_file

    async def run_eis(self, ei: float = 0.0, fl: float = 1.0,
                      fh: float = 100000.0, amp: float = 0.01,
                      sens: float = 0.0, qt: float = 2.0,
                      **kwargs) -> str:
        """运行电化学阻抗谱 (Electrochemical Impedance Spectroscopy)

        Args:
            ei: 直流偏置电位 (V)
            fl: 最低频率 (Hz)
            fh: 最高频率 (Hz)
            amp: 交流振幅 (V)
            sens: 灵敏度 (A/V)
            qt: 静置时间 (s)
        Returns:
            数据文件路径
        """
        self.data["status"] = "Busy"
        self.data["technique"] = "EIS"

        if sens == 0.0:
            sens = self._default_sens

        file_name = self._generate_timestamp_filename("EIS")
        header = f"EIS ei={ei} fl={fl} fh={fh} amp={amp}"

        head = self._generate_macro_header(header)
        body = (
            f"tech=imp\n"
            f"ei={ei}\n"
            f"fl={fl}\n"
            f"fh={fh}\n"
            f"amp={amp}\n"
            f"sens={sens}\n"
            f"qt={qt}"
        )
        foot = (
            f"\nrun\n"
            f"save:{file_name}\n"
            f"tsave:{file_name}\n"
            f"forcequit: yesiamsure\n"
        )
        macro_text = head + body + foot

        mcr_path = self._write_macro(macro_text, file_name)
        params = dict(ei=ei, fl=fl, fh=fh, amp=amp, sens=sens, qt=qt)
        self._save_metadata(file_name, "EIS", params)

        success = self._run_macro(mcr_path)

        data_file = os.path.join(self._data_folder, f"{file_name}.txt")
        self.data["last_data_file"] = data_file
        self.data["last_experiment_time"] = datetime.now().isoformat()
        self.data["status"] = "Idle" if success else "Error"

        self.logger.info(f"EIS completed: {data_file}")
        return data_file

    # ==================== 通用动作 ====================

    async def stop_operation(self, **kwargs) -> bool:
        """终止当前实验

        注意：CHI 宏命令模式下无法直接中止正在运行的实验。
        此方法通过 taskkill 强制结束 CHI 软件进程。
        """
        try:
            exe_name = os.path.basename(self._chi_exe_path)
            subprocess.run(
                f'taskkill /f /im "{exe_name}"',
                shell=True,
                capture_output=True,
            )
            self.logger.warning("Force-killed CHI software process")
        except Exception as e:
            self.logger.error(f"Failed to stop: {e}")
            return False

        self.data["status"] = "Idle"
        return True

    async def list_data_files(self, **kwargs) -> str:
        """列出数据目录下所有 .txt 数据文件

        Returns:
            文件列表的 JSON 字符串
        """
        if not self._data_folder or not os.path.isdir(self._data_folder):
            return "[]"
        files = sorted([
            f for f in os.listdir(self._data_folder)
            if f.endswith(".txt") and not f.endswith("_meta.json")
        ])
        return json.dumps(files, ensure_ascii=False)

    async def read_data(self, file_name: str = "", **kwargs) -> str:
        """读取并解析指定数据文件

        Args:
            file_name: 文件名（不含路径），留空则读取最后一次实验
        Returns:
            JSON 格式的解析结果
        """
        if not file_name:
            file_name = os.path.basename(self.data.get("last_data_file", ""))
        if not file_name:
            return json.dumps({"error": "No data file specified"})

        # 去掉 .txt 后缀
        base_name = file_name.replace(".txt", "")

        # 根据文件名前缀判断数据格式
        tech_prefix = base_name.split("_")[0].upper()
        if tech_prefix in ("CV", "LSV"):
            search_text = "Potential/V,"
        elif tech_prefix in ("CA", "OCP"):
            search_text = "Time/sec,"
        elif tech_prefix == "NPV":
            search_text = "Potential/V,"
        elif tech_prefix == "EIS":
            search_text = "Freq/Hz,"  # EIS 可能用 Freq 开头
        else:
            search_text = "Potential/V,"

        result = self._parse_data_file(base_name, search_text)
        # 如果第一种 search_text 没找到，用备选
        if not result["data"] and search_text != "Time/sec,":
            result = self._parse_data_file(base_name, "Time/sec,")
        if not result["data"]:
            result = self._parse_data_file(base_name, "Z'/ohm,")

        return json.dumps(result, ensure_ascii=False)

    # ==================== 属性（@property）====================

    @property
    def status(self) -> str:
        """设备状态: Idle / Busy / Error / Offline"""
        return self.data.get("status", "Idle")

    @property
    def technique(self) -> str:
        """当前/上一次执行的电化学技术"""
        return self.data.get("technique", "")

    @property
    def last_data_file(self) -> str:
        """最后一次实验的数据文件路径"""
        return self.data.get("last_data_file", "")

    @property
    def last_experiment_time(self) -> str:
        """最后一次实验的时间"""
        return self.data.get("last_experiment_time", "")

    @property
    def data_folder(self) -> str:
        """数据保存目录"""
        return self.data.get("data_folder", "")
