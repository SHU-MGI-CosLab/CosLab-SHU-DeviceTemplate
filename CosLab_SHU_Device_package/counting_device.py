"""Example counting device — demonstrates @device decorator with rich output."""

import logging
from typing import Any, Dict, Optional

from rich.console import Console
from rich.table import Table

from unilabos.registry.decorators import device, action, topic_config


console = Console()


@device(
    id="example_counter",
    category=["example"],
    description="示例计数设备 — 演示 @device 装饰器和外部包依赖",
    display_name="计数器",
)
class CountingDevice:
    """示例计数设备，演示外部设备包的注册表写法。"""

    def __init__(
        self,
        device_id: Optional[str] = None,
        config: Optional[Dict[str, Any]] = None,
        **kwargs,
    ):
        """
        初始化计数设备。

        Args:
            device_id[设备ID]: 设备实例 ID，默认使用 example_counter。
            config[设备配置]: 可包含 step，控制默认计数步长。
        """
        self.device_id = device_id or "example_counter"
        self.config = config or {}
        self.logger = logging.getLogger(f"CountingDevice.{self.device_id}")
        self.data: Dict[str, Any] = {
            "count": 0,
            "status": "idle",
            "step": self.config.get("step", 1),
        }

    def post_init(self, ros_node: Any) -> None:
        self._ros_node = ros_node

    @action(description="增加计数")
    def increment(self, amount: int = 1) -> Dict[str, Any]:
        """
        增加计数。

        Args:
            amount[增加数量]: 本次要增加的计数值。
        """
        self.data["count"] += amount
        self.data["status"] = "counting"
        self._print_status()
        return {"success": True, "count": self.data["count"]}

    @action(description="减少计数")
    def decrement(self, amount: int = 1) -> Dict[str, Any]:
        """
        减少计数。

        Args:
            amount[减少数量]: 本次要减少的计数值。
        """
        self.data["count"] -= amount
        self.data["status"] = "counting"
        self._print_status()
        return {"success": True, "count": self.data["count"]}

    @action(description="重置计数器")
    def reset(self) -> Dict[str, Any]:
        """重置计数器。"""
        self.data["count"] = 0
        self.data["status"] = "idle"
        return {"success": True, "count": 0}

    @property
    @topic_config()
    def count(self) -> int:
        return self.data.get("count", 0)

    @property
    @topic_config()
    def status(self) -> str:
        return self.data.get("status", "idle")

    def _print_status(self) -> None:
        table = Table(title=f"Counter [{self.device_id}]")
        table.add_column("Field", style="cyan")
        table.add_column("Value", style="green")
        table.add_row("count", str(self.data["count"]))
        table.add_row("status", self.data["status"])
        console.print(table)
