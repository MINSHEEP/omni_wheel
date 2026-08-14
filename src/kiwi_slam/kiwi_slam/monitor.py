"""Runtime health monitor for laser, filtered odometry, and required TF links."""

from __future__ import annotations

import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from kiwi_slam.health import quaternion_norm, sample_rate_hz, valid_range_ratio


def value(key: str, data: object) -> KeyValue:
    """Build one diagnostic key/value pair."""
    item = KeyValue()
    item.key = key
    item.value = str(data)
    return item


class KiwiSlamMonitor(Node):
    """Publish health diagnostics and expose an on-demand Trigger check service."""

    def __init__(self) -> None:
        super().__init__("kiwi_slam_monitor")
        self.scan_topic = str(self._required("scan_topic", Parameter.Type.STRING))
        self.odom_topic = str(self._required("odom_topic", Parameter.Type.STRING))
        self.map_topic = str(self._required("map_topic", Parameter.Type.STRING))
        self.laser_frame = str(self._required("laser_frame", Parameter.Type.STRING))
        self.base_frame = str(self._required("base_frame", Parameter.Type.STRING))
        self.odom_frame = str(self._required("odom_frame", Parameter.Type.STRING))
        self.map_frame = str(self._required("map_frame", Parameter.Type.STRING))
        self.timeout_sec = float(self._required("timeout_sec", Parameter.Type.DOUBLE))
        self.map_timeout_sec = float(self._required("map_timeout_sec", Parameter.Type.DOUBLE))
        self.min_valid_ratio = float(
            self._required("min_valid_scan_ratio", Parameter.Type.DOUBLE)
        )
        self.require_map_tf = bool(self._required("require_map_tf", Parameter.Type.BOOL))
        self.debug_log = bool(self._required("debug_log", Parameter.Type.BOOL))

        self.last_scan: LaserScan | None = None
        self.last_odom: Odometry | None = None
        self.last_map: OccupancyGrid | None = None
        self.scan_received = 0.0
        self.odom_received = 0.0
        self.map_received = 0.0
        self.scan_times: deque[float] = deque(maxlen=30)
        self.odom_times: deque[float] = deque(maxlen=30)
        self.map_times: deque[float] = deque(maxlen=30)
        self.last_level = -1

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        # SLAM 地图使用 transient-local，晚启动的监视器也能收到最近一帧地图。
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/kiwi_slam/diagnostics", 10
        )
        self.check_service = self.create_service(Trigger, "~/check", self._on_check)
        self.timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"monitor scan={self.scan_topic} odom={self.odom_topic} map={self.map_topic} "
            f"frames={self.odom_frame}->{self.base_frame}->{self.laser_frame}"
        )

    def _required(self, name: str, parameter_type: Parameter.Type) -> object:
        """声明由 monitor.yaml 提供的必填参数，不在代码中维护回退值。"""
        parameter = self.declare_parameter(name, parameter_type)
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise RuntimeError(f"required parameter '{name}' is missing; load monitor.yaml")
        return parameter.value

    def _on_scan(self, message: LaserScan) -> None:
        """Store the newest scan and receipt time without blocking the driver."""
        now = time.monotonic()
        self.last_scan = message
        self.scan_received = now
        self.scan_times.append(now)

    def _on_odom(self, message: Odometry) -> None:
        """Store the newest filtered odometry and receipt time."""
        now = time.monotonic()
        self.last_odom = message
        self.odom_received = now
        self.odom_times.append(now)

    def _on_map(self, message: OccupancyGrid) -> None:
        """保存最新栅格地图和接收时间，用于判断 SLAM 是否已处理首帧雷达。"""
        now = time.monotonic()
        self.last_map = message
        self.map_received = now
        self.map_times.append(now)

    def _stream_status(self) -> list[DiagnosticStatus]:
        """Validate message freshness, scan content, frames, and quaternion length."""
        now = time.monotonic()
        statuses: list[DiagnosticStatus] = []

        if self.last_scan is None:
            statuses.append(self._status("laser_scan", DiagnosticStatus.ERROR, "no scan received"))
        else:
            scan = self.last_scan
            ratio = valid_range_ratio(scan.ranges, scan.range_min, scan.range_max)
            age = now - self.scan_received
            level = DiagnosticStatus.OK
            message = "scan valid"
            if scan.header.frame_id != self.laser_frame:
                level, message = DiagnosticStatus.ERROR, "unexpected scan frame"
            elif age > self.timeout_sec:
                level, message = DiagnosticStatus.ERROR, "scan timeout"
            elif ratio < self.min_valid_ratio:
                level, message = DiagnosticStatus.WARN, "too few valid ranges"
            statuses.append(
                self._status(
                    "laser_scan",
                    level,
                    message,
                    frame=scan.header.frame_id,
                    age_sec=f"{age:.3f}",
                    rate_hz=f"{sample_rate_hz(self.scan_times):.2f}",
                    points=len(scan.ranges),
                    valid_ratio=f"{ratio:.3f}",
                )
            )

        if self.last_odom is None:
            statuses.append(
                self._status("filtered_odom", DiagnosticStatus.ERROR, "no odometry received")
            )
        else:
            odom = self.last_odom
            age = now - self.odom_received
            orientation = odom.pose.pose.orientation
            norm = quaternion_norm(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            level = DiagnosticStatus.OK
            message = "odometry valid"
            if odom.header.frame_id != self.odom_frame or odom.child_frame_id != self.base_frame:
                level, message = DiagnosticStatus.ERROR, "unexpected odometry frames"
            elif age > self.timeout_sec:
                level, message = DiagnosticStatus.ERROR, "odometry timeout"
            elif abs(norm - 1.0) > 0.01:
                level, message = DiagnosticStatus.ERROR, "invalid odometry quaternion"
            statuses.append(
                self._status(
                    "filtered_odom",
                    level,
                    message,
                    frame=f"{odom.header.frame_id}->{odom.child_frame_id}",
                    age_sec=f"{age:.3f}",
                    rate_hz=f"{sample_rate_hz(self.odom_times):.2f}",
                    quaternion_norm=f"{norm:.6f}",
                )
            )
        if self.last_map is None:
            statuses.append(self._status("map", DiagnosticStatus.ERROR, "no map received"))
        else:
            grid = self.last_map
            age = now - self.map_received
            level = DiagnosticStatus.OK
            message = "map valid"
            if grid.header.frame_id != self.map_frame:
                level, message = DiagnosticStatus.ERROR, "unexpected map frame"
            elif grid.info.width == 0 or grid.info.height == 0 or not grid.data:
                level, message = DiagnosticStatus.ERROR, "empty map"
            elif age > self.map_timeout_sec:
                level, message = DiagnosticStatus.WARN, "map update timeout"
            statuses.append(
                self._status(
                    "map",
                    level,
                    message,
                    frame=grid.header.frame_id,
                    age_sec=f"{age:.3f}",
                    rate_hz=f"{sample_rate_hz(self.map_times):.2f}",
                    size=f"{grid.info.width}x{grid.info.height}",
                    resolution=f"{grid.info.resolution:.3f}",
                )
            )
        return statuses

    def _tf_status(self) -> list[DiagnosticStatus]:
        """Check the TF edges required by scan matching and mapping."""
        pairs = [
            (self.base_frame, self.laser_frame, "base_to_laser_tf"),
            (self.odom_frame, self.base_frame, "odom_to_base_tf"),
        ]
        if self.require_map_tf:
            pairs.append((self.map_frame, self.odom_frame, "map_to_odom_tf"))

        statuses: list[DiagnosticStatus] = []
        for target, source, name in pairs:
            try:
                transform = self.tf_buffer.lookup_transform(target, source, Time())
                translation = transform.transform.translation
                statuses.append(
                    self._status(
                        name,
                        DiagnosticStatus.OK,
                        "transform available",
                        transform=f"{target}<-{source}",
                        xyz=(
                            f"{translation.x:.3f},{translation.y:.3f},"
                            f"{translation.z:.3f}"
                        ),
                    )
                )
            except TransformException as error:
                statuses.append(
                    self._status(
                        name,
                        DiagnosticStatus.ERROR,
                        "transform unavailable",
                        transform=f"{target}<-{source}",
                        error=str(error),
                    )
                )
        return statuses

    def _status(self, name: str, level: int, message: str, **details: object) -> DiagnosticStatus:
        """Create one diagnostic status with stable hardware and key names."""
        status = DiagnosticStatus()
        status.name = f"kiwi_slam/{name}"
        status.hardware_id = "kiwi_base+LSLIDAR-M10P"
        status.level = level
        status.message = message
        status.values = [value(key, data) for key, data in details.items()]
        return status

    def _build_diagnostics(self) -> DiagnosticArray:
        """Build the complete diagnostic message used by timer and service."""
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = self._stream_status() + self._tf_status()
        return message

    def _publish_diagnostics(self) -> None:
        """Publish health periodically and log only changes unless debug mode is enabled."""
        message = self._build_diagnostics()
        self.diagnostic_pub.publish(message)
        level = max(status.level for status in message.status)
        if self.debug_log or level != self.last_level:
            report = " | ".join(
                f"{status.name.rsplit('/', 1)[-1]}={status.message}" for status in message.status
            )
            self._log_report(level, report)
        self.last_level = level

    def _log_report(self, level: int, report: str) -> None:
        """在固定调用位置输出对应级别，避免 Humble logger 在状态切换时抛出异常。"""
        # rclpy Humble 会按源码调用位置缓存日志级别；不同 severity 不能共用同一调用行。
        if level >= DiagnosticStatus.ERROR:
            self.get_logger().error(report)
        elif level >= DiagnosticStatus.WARN:
            self.get_logger().warning(report)
        else:
            self.get_logger().info(report)

    def _on_check(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Return the current complete health report on demand."""
        diagnostics = self._build_diagnostics()
        self.diagnostic_pub.publish(diagnostics)
        response.success = all(
            status.level < DiagnosticStatus.ERROR for status in diagnostics.status
        )
        response.message = " | ".join(
            f"{status.name}:{status.message}" for status in diagnostics.status
        )
        return response


def main(args: list[str] | None = None) -> None:
    """Run the monitor until ROS shutdown."""
    rclpy.init(args=args)
    node = KiwiSlamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C 是正常退出路径，不将其打印成节点异常。
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
