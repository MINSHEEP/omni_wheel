"""为 Mid-360S 原始 IMU 补齐协方差，并保持 FAST-LIO2 原始数据链路独立。"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def adapt_imu(
    raw: Imu,
    angular_variance: tuple[float, float, float],
    accel_variance: tuple[float, float, float],
    deadband: tuple[float, float, float],
) -> Imu:
    """生成 EKF 输入：保留原始六轴量，补协方差并只对角速度应用死区。"""
    message = Imu()
    message.header = raw.header
    message.orientation.w = 1.0
    message.orientation_covariance[0] = -1.0

    angular = (raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z)
    corrected = tuple(
        0.0 if abs(value) <= threshold else value
        for value, threshold in zip(angular, deadband)
    )
    message.angular_velocity.x = corrected[0]
    message.angular_velocity.y = corrected[1]
    message.angular_velocity.z = corrected[2]
    for index, variance in zip((0, 4, 8), angular_variance):
        message.angular_velocity_covariance[index] = variance

    message.linear_acceleration = raw.linear_acceleration
    for index, variance in zip((0, 4, 8), accel_variance):
        message.linear_acceleration_covariance[index] = variance
    return message


class Mid360ImuAdapter(Node):
    """把 `/livox/imu` 转为适合 robot_localization 使用的独立话题。"""

    def __init__(self) -> None:
        super().__init__("mid360_imu_adapter")
        input_topic = str(self._required("input_topic", Parameter.Type.STRING))
        output_topic = str(self._required("output_topic", Parameter.Type.STRING))
        self.angular_variance = self._vector3("angular_velocity_variance", positive=True)
        self.accel_variance = self._vector3("linear_acceleration_variance", positive=True)
        self.deadband = self._vector3("angular_velocity_deadband_rad_s", positive=False)

        # 输出使用 Reliable，兼容 robot_localization 的订阅请求；输入仍采用传感器 QoS。
        self.publisher = self.create_publisher(Imu, output_topic, 20)
        self.subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, qos_profile_sensor_data
        )
        self.get_logger().info(f"Mid-360S IMU adapter: {input_topic} -> {output_topic}")

    def _required(self, name: str, parameter_type: Parameter.Type) -> object:
        """读取无代码默认值的必填参数，缺失时立即给出明确错误。"""
        parameter = self.declare_parameter(name, parameter_type)
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise RuntimeError(f"required parameter '{name}' is missing; load mid360_imu.yaml")
        return parameter.value

    def _vector3(self, name: str, *, positive: bool) -> tuple[float, float, float]:
        """校验三轴方差或死区，避免错误 YAML 静默进入滤波链路。"""
        values = self._required(name, Parameter.Type.DOUBLE_ARRAY)
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise RuntimeError(f"parameter '{name}' must contain three numbers")
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            raise RuntimeError(f"parameter '{name}' must contain finite numbers")
        if positive and not all(value > 0.0 for value in result):
            raise RuntimeError(f"parameter '{name}' must be positive")
        if not positive and not all(value >= 0.0 for value in result):
            raise RuntimeError(f"parameter '{name}' must be non-negative")
        return result  # type: ignore[return-value]

    def _on_imu(self, raw: Imu) -> None:
        """保留时间戳和原始六轴数据，只补协方差并应用可配置角速度死区。"""
        self.publisher.publish(
            adapt_imu(
                raw,
                self.angular_variance,
                self.accel_variance,
                self.deadband,
            )
        )


def main(args: list[str] | None = None) -> None:
    """ROS 2 进程入口。"""
    rclpy.init(args=args)
    node = Mid360ImuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
