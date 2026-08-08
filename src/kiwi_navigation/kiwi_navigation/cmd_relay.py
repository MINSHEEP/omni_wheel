"""将 Nav2 平滑后的 Twist 安全转发到 Kiwi 底盘唯一速度入口."""

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter


def clamp_finite(value: float, limit: float) -> float:
    """拒绝 NaN/Inf，并把单轴速度限制在对称区间内."""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def sanitize_command(
    vx: float, vy: float, wz: float, limits: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    """只保留 Kiwi 底盘使用的 vx、vy、wz，其他 Twist 分量由调用方保持为零."""
    return (
        clamp_finite(vx, limits[0]),
        clamp_finite(vy, limits[1]),
        clamp_finite(wz, limits[2]),
    )


class KiwiNavCmdRelay(Node):
    """转发 Nav2 命令；输入超时后持续发布零速，避免底盘保持上一条指令."""

    def __init__(self) -> None:
        """从 YAML 读取全部接口和限值，并建立订阅、发布与超时定时器."""
        super().__init__("kiwi_nav_cmd_relay")
        # 只声明类型，不提供业务默认值；实际值必须来自 cmd_relay.yaml。
        self.declare_parameter("input_topic", Parameter.Type.STRING)
        self.declare_parameter("output_topic", Parameter.Type.STRING)
        self.declare_parameter("publish_rate_hz", Parameter.Type.DOUBLE)
        self.declare_parameter("command_timeout_s", Parameter.Type.DOUBLE)
        self.declare_parameter("max_linear_speed_m_s", Parameter.Type.DOUBLE)
        self.declare_parameter("max_lateral_speed_m_s", Parameter.Type.DOUBLE)
        self.declare_parameter("max_angular_speed_rad_s", Parameter.Type.DOUBLE)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._timeout_s = float(self.get_parameter("command_timeout_s").value)
        self._limits = (
            float(self.get_parameter("max_linear_speed_m_s").value),
            float(self.get_parameter("max_lateral_speed_m_s").value),
            float(self.get_parameter("max_angular_speed_rad_s").value),
        )
        self._validate_parameters(input_topic, output_topic, publish_rate_hz)

        self._last_input_ns = None
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist, input_topic, self._on_command, 10
        )
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._stop_if_stale)
        self.get_logger().info(
            f"Nav2 速度链路: {input_topic} -> {output_topic}, "
            f"超时 {self._timeout_s:.2f}s"
        )

    def _validate_parameters(
        self, input_topic: str, output_topic: str, publish_rate_hz: float
    ) -> None:
        """启动时拒绝缺失或不合理的 YAML，避免静默使用隐藏默认值."""
        if not input_topic or not output_topic:
            raise ValueError("input_topic and output_topic must be set in YAML")
        values = (publish_rate_hz, self._timeout_s, *self._limits)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("relay rate, timeout and velocity limits must be positive")

    def _on_command(self, command: Twist) -> None:
        """收到 Nav2 命令后立即限幅并发布，记录最后一次有效输入时间."""
        vx, vy, wz = sanitize_command(
            command.linear.x, command.linear.y, command.angular.z, self._limits
        )
        output = Twist()
        output.linear.x = vx
        output.linear.y = vy
        output.angular.z = wz
        self._publisher.publish(output)
        self._last_input_ns = self.get_clock().now().nanoseconds

    def _stop_if_stale(self) -> None:
        """Nav2 未启动或命令中断时持续发零速，覆盖底盘节点中保存的旧命令."""
        now_ns = self.get_clock().now().nanoseconds
        if self._last_input_ns is None:
            self._publisher.publish(Twist())
            return
        age_s = (now_ns - self._last_input_ns) / 1.0e9
        if age_s > self._timeout_s:
            self._publisher.publish(Twist())


def main(args=None) -> None:
    """启动速度中继并在退出前发布零速."""
    rclpy.init(args=args)
    node = KiwiNavCmdRelay()
    try:
        rclpy.spin(node)
    finally:
        node._publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
