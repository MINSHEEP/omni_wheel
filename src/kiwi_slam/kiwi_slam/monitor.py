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
from std_msgs.msg import Bool, String
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
        # A static map_server publishes once; preserve dynamic SLAM checks by default.
        self.map_is_static = self.declare_parameter("map_is_static", False).value
        # 只由履带建图入口启用；全向底盘和静态地图不依赖这些状态话题。
        self.mapping_gate_ready_topic = str(
            self.declare_parameter("mapping_gate_ready_topic", "").value
        )
        self.estimation_state_topic = str(
            self.declare_parameter("estimation_state_topic", "").value
        )
        self.inertial_valid_topic = str(self.declare_parameter("inertial_valid_topic", "").value)
        self.inertial_reason_topic = str(self.declare_parameter("inertial_reason_topic", "").value)
        self.inertial_valid = None
        self.inertial_received = 0.0
        self.inertial_reason = "waiting_inertial_status"
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
        self.map_stamp_ns: int | None = None
        self.map_stamp_changed_at = 0.0
        self.mapping_gate_ready: bool | None = None
        self.mapping_gate_received = 0.0
        self.estimation_state: str | None = None
        self.estimation_state_received = 0.0
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
        self.state_subs = []
        if self.mapping_gate_ready_topic:
            self.state_subs.append(self.create_subscription(
                Bool, self.mapping_gate_ready_topic, self._on_mapping_gate, map_qos
            ))
        if self.estimation_state_topic:
            self.state_subs.append(self.create_subscription(
                String, self.estimation_state_topic, self._on_estimation_state, map_qos
            ))
        if self.inertial_valid_topic:
            self.state_subs.append(self.create_subscription(
                Bool, self.inertial_valid_topic, self._on_inertial_valid, map_qos))
        if self.inertial_reason_topic:
            self.state_subs.append(self.create_subscription(
                String, self.inertial_reason_topic, self._on_inertial_reason, map_qos))
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
        """分别记录地图重发和时间戳更新，避免把周期重发的旧地图视为进展。"""
        now = time.monotonic()
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp_ns != self.map_stamp_ns:
            self.map_stamp_ns = stamp_ns
            self.map_stamp_changed_at = now
        self.last_map = message
        self.map_received = now
        self.map_times.append(now)

    def _on_mapping_gate(self, message: Bool) -> None:
        self.mapping_gate_ready = bool(message.data)
        self.mapping_gate_received = time.monotonic()

    def _on_estimation_state(self, message: String) -> None:
        self.estimation_state = message.data
        self.estimation_state_received = time.monotonic()

    def _on_inertial_valid(self, message: Bool) -> None:
        self.inertial_valid = bool(message.data)
        self.inertial_received = time.monotonic()

    def _on_inertial_reason(self, message: String) -> None:
        self.inertial_reason = message.data

    def _inertial_status(self) -> list[DiagnosticStatus]:
        if not self.inertial_valid_topic:
            return []
        age = time.monotonic()-self.inertial_received
        usable = self.inertial_valid is True and age <= self.timeout_sec
        return [self._status("inertial_fallback", DiagnosticStatus.OK if usable else DiagnosticStatus.WARN,
            "bounded inertial fallback available" if usable else "inertial fallback unavailable",
            reason=self.inertial_reason, age_sec=f"{age:.3f}")]

    def _mapping_progress(self, now: float, unchanged_sec: float) -> tuple[int, str]:
        """静止时允许旧关键帧，但不让健康接收时间掩盖上游停更。"""
        if self.estimation_state_topic:
            if self.estimation_state is None:
                return DiagnosticStatus.WARN, "waiting for estimation state"
            if now - self.estimation_state_received > self.timeout_sec:
                return DiagnosticStatus.ERROR, "estimation state timeout"
            if self.estimation_state not in ("VALID", "INERTIAL"):
                return DiagnosticStatus.ERROR, f"mapping paused: estimation {self.estimation_state}"
            if self.estimation_state == "INERTIAL":
                return DiagnosticStatus.WARN, "mapping paused: inertial prediction only"
        if self.mapping_gate_ready_topic:
            if self.mapping_gate_ready is None:
                return DiagnosticStatus.WARN, "waiting for mapping gate"
            if now - self.mapping_gate_received > self.timeout_sec:
                return DiagnosticStatus.ERROR, "mapping gate state timeout"
            if not self.mapping_gate_ready:
                return DiagnosticStatus.WARN, "mapping paused: gate not ready"
        if unchanged_sec > self.map_timeout_sec and (
            self.last_odom is None or now - self.odom_received > self.timeout_sec
        ):
            return DiagnosticStatus.ERROR, "map stale: upstream odometry unavailable"
        # SLAM Toolbox 可能在静止时重发最后一帧；旧 stamp 本身不是定位故障。
        return DiagnosticStatus.OK, "map valid"

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
            stamp_ns = grid.header.stamp.sec * 1_000_000_000 + grid.header.stamp.nanosec
            header_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000
            unchanged = now - self.map_stamp_changed_at
            level = DiagnosticStatus.OK
            message = "map valid"
            if grid.header.frame_id != self.map_frame:
                level, message = DiagnosticStatus.ERROR, "unexpected map frame"
            elif grid.info.width == 0 or grid.info.height == 0 or not grid.data:
                level, message = DiagnosticStatus.ERROR, "empty map"
            elif not self.map_is_static and age > self.map_timeout_sec:
                level, message = DiagnosticStatus.WARN, "map update timeout"
            elif not self.map_is_static:
                level, message = self._mapping_progress(now, unchanged)
            statuses.append(
                self._status(
                    "map",
                    level,
                    message,
                    frame=grid.header.frame_id,
                    age_sec=f"{age:.3f}",
                    header_age_sec=f"{header_age:.3f}",
                    stamp_unchanged_sec=f"{unchanged:.3f}",
                    mapping_gate_ready=self.mapping_gate_ready,
                    estimation_state=self.estimation_state,
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
                stamp = transform.header.stamp
                stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                age_sec = (
                    (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000
                    if stamp_ns else 0.0
                )
                # 只有 URDF 的 base→laser 可使用零时间戳；里程计与地图变换必须持续更新。
                stale = name != "base_to_laser_tf" and (
                    stamp_ns == 0 or age_sec > self.timeout_sec
                )
                statuses.append(
                    self._status(
                        name,
                        DiagnosticStatus.ERROR if stale else DiagnosticStatus.OK,
                        "transform stale" if stale else "transform available",
                        transform=f"{target}<-{source}",
                        age_sec=f"{age_sec:.3f}",
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
        message.status = self._stream_status() + self._tf_status() + self._inertial_status()
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
