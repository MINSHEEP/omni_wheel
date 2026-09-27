"""将 FAST-LIO2 机体系点云投影为供 AMCL/Nav2 使用的水平扫描。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class ProjectionConfig:
    """base_footprint -> body -> lidar_link 外参与扫描裁剪范围。"""

    base_to_body_xyz: tuple[float, float, float]
    base_to_body_rpy: tuple[float, float, float]
    body_to_lidar_xyz: tuple[float, float, float]
    body_to_lidar_rpy: tuple[float, float, float]
    z_min: float = 0.10
    z_max: float = 1.2
    range_min: float = 0.5
    range_max: float = 20.0
    beam_count: int = 720

    def __post_init__(self) -> None:
        for name in ("base_to_body_xyz", "base_to_body_rpy",
                     "body_to_lidar_xyz", "body_to_lidar_rpy"):
            values = getattr(self, name)
            if len(values) != 3 or any(not math.isfinite(value) for value in values):
                raise ValueError(f"{name} 必须显式提供三个有限的标定值")
        if not all(math.isfinite(value) for value in
                   (self.z_min, self.z_max, self.range_min, self.range_max)):
            raise ValueError("高度和量程必须是有限数值")
        if not 0 <= self.z_min < self.z_max:
            raise ValueError("高度范围无效")
        if not 0 < self.range_min < self.range_max:
            raise ValueError("量程范围无效")
        if isinstance(self.beam_count, bool) or not isinstance(self.beam_count, int) or self.beam_count < 4:
            raise ValueError("beam_count 必须是至少为 4 的整数")


def _rotation_from_rpy(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    """使用 ROS 固定轴 RPY：Rz(yaw) * Ry(pitch) * Rx(roll)。"""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def project_points(points: Iterable[tuple[float, float, float]],
                   config: ProjectionConfig) -> tuple[float, ...]:
    """高度在底盘系裁剪，束角/距离在真实雷达原点计算。"""
    rotation = _rotation_from_rpy(config.base_to_body_rpy)
    lidar_rotation = _rotation_from_rpy(config.body_to_lidar_rpy)
    tx, ty, tz = config.base_to_body_xyz
    lx, ly, lz = config.body_to_lidar_xyz
    ranges = [math.inf] * config.beam_count
    increment = 2.0 * math.pi / config.beam_count
    for point in points:
        try:
            x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        except (TypeError, ValueError, OverflowError, IndexError):
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        bx = rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + tx
        by = rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + ty
        bz = rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + tz
        if not config.z_min <= bz <= config.z_max:
            continue
        # FAST-LIO2 的 body 云在 IMU 坐标；LaserScan 必须从雷达原点发射，
        # 不能把前置雷达的光束假装从 base_footprint 发出，否则会误清障。
        shifted = (x - lx, y - ly, z - lz)
        px = sum(lidar_rotation[row][0] * shifted[row] for row in range(3))
        py = sum(lidar_rotation[row][1] * shifted[row] for row in range(3))
        distance = math.hypot(px, py)
        if not config.range_min <= distance <= config.range_max:
            continue
        # atan2 的 +pi 与 -pi 是同一束，统一落入第一个角度桶。
        angle = math.atan2(py, px)
        index = int(math.floor((angle + math.pi) / increment)) % config.beam_count
        ranges[index] = min(ranges[index], distance)
    return tuple(ranges)


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2

    class Mid360ScanProjector(Node):
        def __init__(self) -> None:
            super().__init__("mid360_scan_projector")

            def parameter(name, default):
                return self.declare_parameter(name, default).value

            # NaN 默认值保留 DOUBLE_ARRAY 类型；遗漏外参由 ProjectionConfig 拒绝。
            missing = [math.nan, math.nan, math.nan]
            xyz = parameter("base_to_body_xyz", missing)
            rpy = parameter("base_to_body_rpy", missing)
            lidar_xyz = parameter("body_to_lidar_xyz", missing)
            lidar_rpy = parameter("body_to_lidar_rpy", missing)
            self.config = ProjectionConfig(
                tuple(xyz), tuple(rpy), tuple(lidar_xyz), tuple(lidar_rpy),
                z_min=float(parameter("z_min", 0.10)),
                z_max=float(parameter("z_max", 1.2)),
                range_min=float(parameter("range_min", 0.5)),
                range_max=float(parameter("range_max", 20.0)),
                beam_count=int(parameter("beam_count", 720)),
            )
            self.body_frame = str(parameter("body_frame", "body"))
            self.lidar_frame = str(parameter("lidar_frame", "lidar_link"))
            input_topic = str(parameter("input_topic", "/cloud_registered_body"))
            output_topic = str(parameter("output_topic", "/scan_mid360"))
            if not self.body_frame or not self.lidar_frame or self.body_frame == self.lidar_frame:
                raise ValueError("body_frame 和 lidar_frame 必须是不同的非空坐标系")
            if not input_topic or not output_topic or input_topic == output_topic:
                raise ValueError("点云输入与 LaserScan 输出话题必须不同且非空")
            self.publisher = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
            self.subscription = self.create_subscription(
                PointCloud2, input_topic, self.on_cloud, qos_profile_sensor_data
            )

        def on_cloud(self, cloud: PointCloud2) -> None:
            if cloud.header.frame_id != self.body_frame:
                self.get_logger().warn(
                    f"丢弃点云：期望 {self.body_frame}，收到 {cloud.header.frame_id}"
                )
                return
            try:
                points = point_cloud2.read_points(
                    cloud, field_names=("x", "y", "z"), skip_nans=False
                )
                ranges = project_points(points, self.config)
            except (ValueError, KeyError) as error:
                self.get_logger().warn(f"丢弃无效点云：{error}")
                return
            scan = LaserScan()
            scan.header.stamp = cloud.header.stamp
            scan.header.frame_id = self.lidar_frame
            scan.angle_min = -math.pi
            scan.angle_increment = 2.0 * math.pi / self.config.beam_count
            scan.angle_max = scan.angle_min + (self.config.beam_count - 1) * scan.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = 0.0
            scan.range_min = self.config.range_min
            scan.range_max = self.config.range_max
            scan.ranges = ranges
            self.publisher.publish(scan)

    rclpy.init()
    node = None
    try:
        node = Mid360ScanProjector()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
