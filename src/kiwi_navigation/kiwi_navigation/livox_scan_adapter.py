"""将 Livox CustomMsg 转成 SLAM Toolbox/Nav2 统一使用的 LaserScan."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def project_points(
    points: Iterable[object],
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    min_height: float,
    max_height: float,
) -> list[float]:
    """把三维点按高度过滤并投影到二维角度栅格，保留每束最近点."""
    bin_count = int(math.ceil((angle_max - angle_min) / angle_increment))
    ranges = [math.inf] * bin_count
    for point in points:
        x = float(point.x)
        y = float(point.y)
        z = float(point.z)
        distance = math.hypot(x, y)
        if not all(math.isfinite(value) for value in (x, y, z, distance)):
            continue
        if not range_min <= distance <= range_max or not min_height <= z <= max_height:
            continue
        angle = math.atan2(y, x)
        if angle < angle_min or angle >= angle_max:
            continue
        index = int((angle - angle_min) / angle_increment)
        if 0 <= index < bin_count and distance < ranges[index]:
            ranges[index] = distance
    return ranges


class LivoxScanAdapter(Node):
    """将 Mid-360 原始点云转换成雷达型号无关的 `/scan` 话题."""

    def __init__(self) -> None:
        """读取 YAML 参数并建立 Livox 订阅和 LaserScan 发布器."""
        super().__init__("livox_scan_adapter")
        self._declare_types()
        self._read_parameters()
        self._publisher = self.create_publisher(
            LaserScan, self._output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            CustomMsg,
            self._input_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Livox scan adapter: {self._input_topic} -> {self._output_topic}, "
            f"frame={self._output_frame}"
        )

    def _declare_types(self) -> None:
        """只声明参数类型，实际值全部来自 radar_mid360.yaml."""
        self.declare_parameter("input_topic", Parameter.Type.STRING)
        self.declare_parameter("output_topic", Parameter.Type.STRING)
        self.declare_parameter("output_frame", Parameter.Type.STRING)
        for name in (
            "angle_min_rad",
            "angle_max_rad",
            "angle_increment_rad",
            "range_min_m",
            "range_max_m",
            "min_height_m",
            "max_height_m",
            "scan_time_s",
        ):
            self.declare_parameter(name, Parameter.Type.DOUBLE)

    def _read_parameters(self) -> None:
        """读取并校验雷达适配参数，阻止错误量程或角度进入导航."""
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._angle_min = float(self.get_parameter("angle_min_rad").value)
        self._angle_max = float(self.get_parameter("angle_max_rad").value)
        self._angle_increment = float(
            self.get_parameter("angle_increment_rad").value
        )
        self._range_min = float(self.get_parameter("range_min_m").value)
        self._range_max = float(self.get_parameter("range_max_m").value)
        self._min_height = float(self.get_parameter("min_height_m").value)
        self._max_height = float(self.get_parameter("max_height_m").value)
        self._scan_time = float(self.get_parameter("scan_time_s").value)
        values = (
            self._angle_min,
            self._angle_max,
            self._angle_increment,
            self._range_min,
            self._range_max,
            self._min_height,
            self._max_height,
            self._scan_time,
        )
        if not self._input_topic or not self._output_topic:
            raise ValueError("Livox scan topics must be set in YAML")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Livox scan parameters must be finite")
        if self._angle_max <= self._angle_min or self._angle_increment <= 0.0:
            raise ValueError("Livox scan angle range is invalid")
        if self._range_min <= 0.0 or self._range_max <= self._range_min:
            raise ValueError("Livox scan range is invalid")
        if self._max_height <= self._min_height or self._scan_time <= 0.0:
            raise ValueError("Livox scan height or scan time is invalid")

    def _on_cloud(self, message: CustomMsg) -> None:
        """将一帧 Livox 点云投影为 LaserScan，并保留原始时间戳."""
        scan = LaserScan()
        scan.header = message.header
        if self._output_frame:
            scan.header.frame_id = self._output_frame
        scan.angle_min = self._angle_min
        scan.angle_increment = self._angle_increment
        scan.ranges = project_points(
            message.points,
            self._angle_min,
            self._angle_max,
            self._angle_increment,
            self._range_min,
            self._range_max,
            self._min_height,
            self._max_height,
        )
        scan.angle_max = scan.angle_min + len(scan.ranges) * scan.angle_increment
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.scan_time = self._scan_time
        scan.time_increment = self._scan_time / max(len(scan.ranges), 1)
        self._publisher.publish(scan)


def main(args: Sequence[str] | None = None) -> None:
    """启动 Livox 到 LaserScan 的统一接口节点."""
    rclpy.init(args=args)
    node = LivoxScanAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
