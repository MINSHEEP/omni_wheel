"""给不发布协方差的 RF2O 里程计补上履带平台专属的不确定度。"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def set_covariance(
    message: Odometry,
    *,
    pose_xy_variance: float,
    pose_yaw_variance: float,
    twist_vx_variance: float,
    twist_vy_variance: float,
    twist_wz_variance: float,
) -> Odometry:
    """只设置平面估计量；其余轴标为高不确定度。"""
    message.pose.covariance = [0.0] * 36
    message.twist.covariance = [0.0] * 36
    for covariance, values in (
        (
            message.pose.covariance,
            (pose_xy_variance, pose_xy_variance, 1e6, 1e6, 1e6, pose_yaw_variance),
        ),
        (
            message.twist.covariance,
            (twist_vx_variance, twist_vy_variance, 1e6, 1e6, 1e6, twist_wz_variance),
        ),
    ):
        for index, value in zip((0, 7, 14, 21, 28, 35), values):
            covariance[index] = value
    return message


class OdometryCovarianceAdapter(Node):
    def __init__(self) -> None:
        super().__init__("rf2o_covariance_adapter")
        self.source_topic = self._topic("source_topic")
        self.output_topic = self._topic("output_topic")
        if self.source_topic == self.output_topic:
            raise ValueError("RF2O source_topic and output_topic must differ")
        self.odom_frame = self._topic("odom_frame")
        self.base_frame = self._topic("base_frame")
        self.variances = {
            name: self._variance(name)
            for name in (
                "pose_xy_variance", "pose_yaw_variance", "twist_vx_variance",
                "twist_vy_variance", "twist_wz_variance",
            )
        }
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscriber = self.create_subscription(
            Odometry, self.source_topic, self._on_odometry, 10
        )
        self.get_logger().info(
            f"RF2O covariance: {self.source_topic} -> {self.output_topic}"
        )

    def _topic(self, name: str) -> str:
        value = self.declare_parameter(name, "").value
        if not isinstance(value, str) or not value:
            raise ValueError(f"{name} must be a nonempty string")
        return value

    def _variance(self, name: str) -> float:
        value = float(self.declare_parameter(name, 0.0).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _on_odometry(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self.odom_frame
            or message.child_frame_id != self.base_frame
        ):
            self.get_logger().error(
                "RF2O odometry frame mismatch: "
                f"{message.header.frame_id} -> {message.child_frame_id}"
            )
            return
        self.publisher.publish(set_covariance(message, **self.variances))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryCovarianceAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
