import logging
import time as time_module
import numpy as np
from typing import Dict, Any, Optional

try:
    from unilabos.ros.nodes.base_device_node import BaseROS2DeviceNode
except ImportError:
    BaseROS2DeviceNode = None

try:
    from harvesters.core import Harvester
except ImportError:
    Harvester = None


class DahengHdR630c:
    """度申 DO3THINK M3S630-H-O2C (HD-R630C-U3) USB3 Vision 彩色工业相机驱动。

    通过 Harvesters (GenICam/GenTL) 通用接口与相机通信，支持单帧采集、
    连续采集、曝光时间和增益设置等功能。

    依赖：
        - harvesters: pip install harvesters
        - GenTL Producer: 度申 DVPCameraTL64.cti
    """

    _ros_node: "BaseROS2DeviceNode"

    # 默认 .cti 文件路径（度申 64 位）
    DEFAULT_CTI_PATH = r"C:\Program Files (x86)\Do3think\DVP2 x64\DVPCameraTL64.cti"

    def __init__(self, device_id: str = None, config: Dict[str, Any] = None, **kwargs):
        if device_id is None and "id" in kwargs:
            device_id = kwargs.pop("id")
        if config is None and "config" in kwargs:
            config = kwargs.pop("config")
        self.device_id = device_id or "unknown_device"
        self.config = config or {}
        self.logger = logging.getLogger(f"DahengHdR630c.{self.device_id}")

        # ── 预填充所有属性字段的默认值（硬约束 #3）──
        self.data: Dict[str, Any] = {
            "status": "Idle",
            "exposure_time": 10000.0,      # 默认曝光时间 10ms (10000μs)
            "gain": 0.0,                   # 默认增益 0dB
            "frame_rate": 0.0,             # 当前帧率
            "image_width": 0,              # 图像宽度 px
            "image_height": 0,             # 图像高度 px
            "is_streaming": False,         # 是否连续采集中
            "last_frame_id": 0,            # 最后一帧帧号
            # 兼容 sensor 基础接口
            "level": False,                # 设备是否就绪（True=已连接）
            "last_image_path": "",         # 最后保存的图片路径
            "rssi": 0,                     # 信号强度（USB相机固定为100）
        }

        # 内部状态
        self._harvester: Optional[Harvester] = None
        self._ia = None  # ImageAcquirer
        self._last_image: Optional[np.ndarray] = None

        # 从 config 读取配置
        self._cti_path: str = self.config.get("cti_path", self.DEFAULT_CTI_PATH)
        self._device_index: int = self.config.get("device_index", 0)
        self._default_exposure: float = self.config.get("default_exposure_time", 10000.0)
        self._default_gain: float = self.config.get("default_gain", 0.0)

    def post_init(self, ros_node: "BaseROS2DeviceNode"):
        """框架回调：注入 ROS 节点引用。"""
        self._ros_node = ros_node

    # ════════════════════════════════════════════
    #  生命周期方法
    # ════════════════════════════════════════════

    async def initialize(self) -> bool:
        """初始化相机连接。

        通过 Harvesters + GenTL 枚举设备并打开相机，配置默认参数。

        Returns:
            bool: 初始化是否成功
        """
        if Harvester is None:
            self.logger.error("harvesters 未安装。请执行: pip install harvesters")
            self.data["status"] = "Offline"
            return False

        try:
            self.data["status"] = "Busy"

            # 创建 Harvester 并加载 GenTL Producer
            self._harvester = Harvester()
            cti_path = self._cti_path
            self.logger.info(f"加载 GenTL Producer: {cti_path}")
            
            # 如果 config 传入的路径无效，使用默认路径兜底
            import os
            if not os.path.isfile(cti_path):
                cti_path = self.DEFAULT_CTI_PATH
                self.logger.warning(f"config 中 cti_path 无效，使用默认路径: {cti_path}")
            
            self._harvester.add_file(cti_path)
            self.logger.info("GenTL Producer 已加载，开始枚举设备...")
            self._harvester.update()

            dev_count = len(self._harvester.device_info_list)
            self.logger.info(f"枚举完成，发现 {dev_count} 台设备。")

            if dev_count == 0:
                # 尝试多次枚举（某些相机需要等待）
                import asyncio
                for retry in range(3):
                    self.logger.info(f"重试枚举第 {retry + 1} 次...")
                    await self._ros_node.sleep(1.0)
                    self._harvester.update()
                    dev_count = len(self._harvester.device_info_list)
                    self.logger.info(f"重试结果：发现 {dev_count} 台设备。")
                    if dev_count > 0:
                        break

            if dev_count == 0:
                self.logger.error("多次枚举后仍未发现相机设备。")
                self.data["status"] = "Offline"
                return False

            self.logger.info(f"发现 {dev_count} 台相机设备。")
            for i, dev in enumerate(self._harvester.device_info_list):
                self.logger.info(f"  设备 {i}: {dev.display_name} (model={dev.model})")

            # 打开指定索引的相机
            self._ia = self._harvester.create(self._device_index)

            # 读取相机节点参数
            node_map = self._ia.remote_device.node_map

            # 读取图像尺寸
            try:
                self.data["image_width"] = node_map.Width.value
                self.data["image_height"] = node_map.Height.value
            except Exception:
                self.logger.warning("无法读取图像尺寸。")

            # 设置默认曝光时间
            try:
                node_map.ExposureTime.value = self._default_exposure
                self.data["exposure_time"] = node_map.ExposureTime.value
            except Exception:
                self.logger.warning("无法设置曝光时间，尝试 ExposureTimeAbs...")
                try:
                    node_map.ExposureTimeAbs.value = self._default_exposure
                    self.data["exposure_time"] = node_map.ExposureTimeAbs.value
                except Exception:
                    self.logger.warning("无法设置曝光时间。")

            # 设置默认增益
            try:
                node_map.Gain.value = self._default_gain
                self.data["gain"] = node_map.Gain.value
            except Exception:
                try:
                    node_map.GainRaw.value = int(self._default_gain)
                    self.data["gain"] = float(node_map.GainRaw.value)
                except Exception:
                    self.logger.warning("无法设置增益。")

            # 读取帧率
            try:
                self.data["frame_rate"] = node_map.AcquisitionFrameRate.value
            except Exception:
                self.logger.warning("无法读取帧率。")

            # 更新传感器兼容属性
            self.data["level"] = True
            self.data["rssi"] = 100  # USB 直连，信号满格

            self.data["status"] = "Idle"
            self.logger.info(
                f"相机初始化成功: {self.data['image_width']}x{self.data['image_height']}, "
                f"曝光={self.data['exposure_time']}μs, 增益={self.data['gain']}dB"
            )
            return True

        except Exception as e:
            self.logger.error(f"相机初始化失败: {e}")
            self.data["status"] = "Offline"
            self.data["level"] = False
            return False

    async def cleanup(self) -> bool:
        """关闭相机并释放资源。

        Returns:
            bool: 清理是否成功
        """
        try:
            if self._ia is not None:
                # 如果正在采集，先停止
                if self.data["is_streaming"]:
                    self._ia.stop()
                    self.data["is_streaming"] = False

                self._ia.destroy()
                self._ia = None
                self.logger.info("ImageAcquirer 已销毁。")

            if self._harvester is not None:
                self._harvester.reset()
                self._harvester = None
                self.logger.info("Harvester 已释放。")

            self.data["status"] = "Offline"
            self.data["level"] = False
            self.data["rssi"] = 0
            return True

        except Exception as e:
            self.logger.error(f"相机清理失败: {e}")
            self.data["status"] = "Offline"
            return False

    # ════════════════════════════════════════════
    #  动作方法
    # ════════════════════════════════════════════

    async def snap(self) -> Optional[np.ndarray]:
        """单帧采集。

        从相机采集一帧图像并返回 numpy 数组。

        Returns:
            numpy.ndarray or None: BGR 格式图像数据，采集失败返回 None
        """
        if self._ia is None:
            self.logger.error("相机未初始化，无法采集。")
            return None

        try:
            self.data["status"] = "Busy"
            was_streaming = self.data["is_streaming"]

            # 如果没有在连续采集，先启动采集
            if not was_streaming:
                self._ia.start()

            # 获取一帧图像（超时 5 秒）
            with self._ia.fetch(timeout=5.0) as buffer:
                component = buffer.payload.components[0]

                # 获取图像数据
                width = component.width
                height = component.height

                # 从 buffer 拿到 numpy 数据
                image_data = component.data.reshape(height, width, -1) if component.data.ndim == 1 else component.data

                if image_data.ndim == 2:
                    # 灰度图像，转成 3 通道
                    self._last_image = np.stack([image_data] * 3, axis=-1).copy()
                elif image_data.shape[2] == 3:
                    # RGB -> BGR (OpenCV 格式)
                    self._last_image = image_data[:, :, ::-1].copy()
                elif image_data.shape[2] == 4:
                    # RGBA -> BGR
                    self._last_image = image_data[:, :, 2::-1].copy()
                else:
                    self._last_image = image_data.copy()

                self.data["last_frame_id"] = self.data["last_frame_id"] + 1
                self.data["image_width"] = width
                self.data["image_height"] = height

            if not was_streaming:
                self._ia.stop()

            # 保存图片到文件
            save_dir = self.config.get("save_dir", "./captured_images")
            import os
            os.makedirs(save_dir, exist_ok=True)

            timestamp = time_module.strftime("%Y%m%d_%H%M%S")
            filename = f"snap_{self.data['last_frame_id']}_{timestamp}.png"
            filepath = os.path.join(save_dir, filename)

            try:
                import cv2
                cv2.imwrite(filepath, self._last_image)
                self.data["last_image_path"] = os.path.abspath(filepath)
                self.logger.info(f"图片已保存: {os.path.abspath(filepath)}")
            except ImportError:
                # 没有 cv2，用 PIL 保存
                try:
                    from PIL import Image
                    # self._last_image 是 BGR 格式，转 RGB
                    rgb_image = self._last_image[:, :, ::-1]
                    Image.fromarray(rgb_image).save(filepath)
                    self.data["last_image_path"] = os.path.abspath(filepath)
                    self.logger.info(f"图片已保存: {os.path.abspath(filepath)}")
                except ImportError:
                    self.logger.warning("未安装 cv2 或 PIL，无法保存图片到文件。")

            self.data["status"] = "Streaming" if was_streaming else "Idle"
            self.logger.info(
                f"采集成功: frame_id={self.data['last_frame_id']}, "
                f"shape={self._last_image.shape if self._last_image is not None else 'N/A'}"
            )
            return self._last_image

        except Exception as e:
            self.logger.error(f"单帧采集失败: {e}")
            if not was_streaming:
                try:
                    self._ia.stop()
                except Exception:
                    pass
            self.data["status"] = "Idle"
            return None

    async def start_stream(self):
        """开始连续采集。

        启动相机的连续数据流，后续可通过 snap() 获取最新帧。
        """
        if self._ia is None:
            self.logger.error("相机未初始化，无法开始连续采集。")
            return

        if self.data["is_streaming"]:
            self.logger.warning("相机已在连续采集中。")
            return

        try:
            self.data["status"] = "Busy"
            self._ia.start()
            self.data["is_streaming"] = True
            self.data["status"] = "Streaming"
            self.logger.info("连续采集已启动。")
        except Exception as e:
            self.logger.error(f"启动连续采集失败: {e}")
            self.data["status"] = "Idle"

    async def stop_stream(self):
        """停止连续采集。"""
        if self._ia is None:
            self.logger.error("相机未初始化。")
            return

        if not self.data["is_streaming"]:
            self.logger.warning("相机未在连续采集中。")
            return

        try:
            self._ia.stop()
            self.data["is_streaming"] = False
            self.data["status"] = "Idle"
            self.logger.info("连续采集已停止。")
        except Exception as e:
            self.logger.error(f"停止连续采集失败: {e}")

    async def set_exposure_time(self, exposure_time: float):
        """设置曝光时间。

        Args:
            exposure_time: 曝光时间，单位 μs（微秒）
        """
        if self._ia is None:
            self.logger.error("相机未初始化，无法设置曝光时间。")
            return

        try:
            node_map = self._ia.remote_device.node_map
            try:
                node_map.ExposureTime.value = exposure_time
                self.data["exposure_time"] = node_map.ExposureTime.value
            except Exception:
                node_map.ExposureTimeAbs.value = exposure_time
                self.data["exposure_time"] = node_map.ExposureTimeAbs.value
            self.logger.info(f"曝光时间已设置为 {self.data['exposure_time']} μs")
        except Exception as e:
            self.logger.error(f"设置曝光时间失败: {e}")

    async def set_gain(self, gain: float):
        """设置增益。

        Args:
            gain: 增益值，单位 dB
        """
        if self._ia is None:
            self.logger.error("相机未初始化，无法设置增益。")
            return

        try:
            node_map = self._ia.remote_device.node_map
            try:
                node_map.Gain.value = gain
                self.data["gain"] = node_map.Gain.value
            except Exception:
                node_map.GainRaw.value = int(gain)
                self.data["gain"] = float(node_map.GainRaw.value)
            self.logger.info(f"增益已设置为 {self.data['gain']} dB")
        except Exception as e:
            self.logger.error(f"设置增益失败: {e}")

    # ════════════════════════════════════════════
    #  属性 (Properties)
    # ════════════════════════════════════════════

    @property
    def status(self) -> str:
        """设备状态: "Idle" / "Busy" / "Streaming" / "Offline" """
        return self.data.get("status", "Idle")

    @property
    def exposure_time(self) -> float:
        """当前曝光时间（μs）。"""
        return self.data.get("exposure_time", 10000.0)

    @property
    def gain(self) -> float:
        """当前增益（dB）。"""
        return self.data.get("gain", 0.0)

    @property
    def frame_rate(self) -> float:
        """当前帧率（fps）。"""
        return self.data.get("frame_rate", 0.0)

    @property
    def image_width(self) -> int:
        """图像宽度（px）。"""
        return self.data.get("image_width", 0)

    @property
    def image_height(self) -> int:
        """图像高度（px）。"""
        return self.data.get("image_height", 0)

    @property
    def is_streaming(self) -> bool:
        """是否在连续采集中。"""
        return self.data.get("is_streaming", False)

    @property
    def last_frame_id(self) -> int:
        """最后一帧的帧号。"""
        return self.data.get("last_frame_id", 0)

    @property
    def level(self) -> bool:
        """设备是否在线（兼容 sensor 基础接口）。"""
        return self.data.get("level", False)

    @property
    def rssi(self) -> int:
        """信号强度（兼容 sensor 基础接口，USB 直连固定为 100）。"""
        return self.data.get("rssi", 0)

    def get_last_image(self) -> Optional[np.ndarray]:
        """获取最近一次采集的图像数据。

        Returns:
            numpy.ndarray or None: BGR 格式图像数据
        """
        return self._last_image