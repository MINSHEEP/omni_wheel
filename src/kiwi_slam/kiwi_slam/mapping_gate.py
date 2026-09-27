"""在底盘标定和首批扫描稳定后，才向 SLAM Toolbox 放行激光数据。"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


@dataclass(frozen=True)
class MappingGateConfig:
    """建图首帧门控阈值；所有时间均使用单调接收时间。"""

    stationary_duration_s: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_yaw_step_rad: float
    max_message_age_s: float
    required_stable_scans: int
    min_valid_scan_ratio: float
    min_scan_overlap_ratio: float
    max_median_range_delta_m: float

    def __post_init__(self) -> None:
        positive = (
            self.stationary_duration_s,
            self.max_linear_speed_m_s,
            self.max_angular_speed_rad_s,
            self.max_yaw_step_rad,
            self.max_message_age_s,
            self.max_median_range_delta_m,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("mapping gate positive thresholds must be finite")
        if self.required_stable_scans < 2:
            raise ValueError("required_stable_scans must be at least 2")
        for value in (self.min_valid_scan_ratio, self.min_scan_overlap_ratio):
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError("scan ratios must be in (0, 1]")


class MappingStartupGate:
    """可独立测试的首帧门控状态机。

    默认要求 IMU 标定 ready；纯激光配置可显式关闭该要求。之后还
    要求里程计连续静止，并确认若干相邻 LaserScan 的共同有效束距离一致。任何
    运动、断流或异常扫描都会清空本轮证据；门一旦打开则保持打开，避免正常
    建图运动期间反复断流。
    """

    def __init__(self, config: MappingGateConfig, require_imu_ready: bool = True) -> None:
        self.config = config
        self.require_imu_ready = require_imu_ready
        self.imu_ready = not require_imu_ready
        self.opened = False
        self.stationary_since: float | None = None
        self.last_odom_time: float | None = None
        self.previous_yaw: float | None = None
        self.previous_ranges: list[float] | None = None
        self.stable_scan_count = 0

    def set_imu_ready(self, ready: bool) -> None:
        """更新底盘标定状态；只在状态改变时重建启动证据。"""
        if not self.require_imu_ready:
            return
        if ready == self.imu_ready or self.opened:
            return
        self.imu_ready = ready
        self._reset_evidence()

    def observe_odometry(
        self,
        received_at: float,
        velocity_x: float,
        velocity_y: float,
        angular_z: float,
        yaw: float,
    ) -> None:
        """输入一帧里程计，用速度、yaw 连续性和接收间隔确认静止。"""
        if self.opened:
            return
        values = (received_at, velocity_x, velocity_y, angular_z, yaw)
        if not self.imu_ready or any(
            not math.isfinite(value) for value in values
        ):
            self._reset_evidence()
            return

        gap_valid = self.last_odom_time is not None and (
            0.0
            < received_at - self.last_odom_time
            <= self.config.max_message_age_s
        )
        yaw_step = 0.0
        if gap_valid and self.previous_yaw is not None:
            # remainder 正确处理 +pi/-pi 过零，避免稳定绝对 yaw 被误判为整圈跳变。
            yaw_step = abs(
                math.remainder(yaw - self.previous_yaw, 2.0 * math.pi)
            )

        speeds_stable = (
            abs(velocity_x) <= self.config.max_linear_speed_m_s
            and abs(velocity_y) <= self.config.max_linear_speed_m_s
            and abs(angular_z) <= self.config.max_angular_speed_rad_s
        )
        sample_stable = speeds_stable and (
            not gap_valid or yaw_step <= self.config.max_yaw_step_rad
        )

        if not sample_stable or (
            self.last_odom_time is not None and not gap_valid
        ):
            self._reset_evidence()
        if sample_stable:
            if self.stationary_since is None:
                self.stationary_since = received_at
            self.last_odom_time = received_at
            self.previous_yaw = yaw

    def observe_scan(
        self,
        received_at: float,
        ranges: Sequence[float],
        range_min: float,
        range_max: float,
    ) -> bool:
        """输入一帧扫描；返回本帧是否应转发给 SLAM。"""
        if self.opened:
            return True
        if not self._odom_is_ready(received_at):
            self._reset_scan_evidence()
            return False
        if (
            not math.isfinite(range_min)
            or not math.isfinite(range_max)
            or range_min < 0.0
            or range_max <= range_min
            or not ranges
        ):
            self._reset_scan_evidence()
            return False

        current = [float(value) for value in ranges]
        current_valid = [
            value
            for value in current
            if math.isfinite(value) and range_min <= value <= range_max
        ]
        if (
            len(current_valid) / len(current)
            < self.config.min_valid_scan_ratio
        ):
            self._reset_scan_evidence()
            return False

        if self.previous_ranges is None or len(self.previous_ranges) != len(
            current
        ):
            self.previous_ranges = current
            self.stable_scan_count = 1
            return False

        paired_deltas = [
            abs(current_value - previous_value)
            for current_value, previous_value in zip(
                current, self.previous_ranges
            )
            if math.isfinite(current_value)
            and math.isfinite(previous_value)
            and range_min <= current_value <= range_max
            and range_min <= previous_value <= range_max
        ]
        previous_valid_count = sum(
            1
            for value in self.previous_ranges
            if math.isfinite(value) and range_min <= value <= range_max
        )
        overlap_base = min(len(current_valid), previous_valid_count)
        overlap_ratio = (
            len(paired_deltas) / overlap_base if overlap_base else 0.0
        )
        scans_consistent = (
            overlap_ratio >= self.config.min_scan_overlap_ratio
            and paired_deltas
            and statistics.median(paired_deltas)
            <= self.config.max_median_range_delta_m
        )

        # 不一致帧本身可作为下一轮基准，但绝不能把不连续的两段计数拼接起来。
        self.previous_ranges = current
        self.stable_scan_count = (
            self.stable_scan_count + 1 if scans_consistent else 1
        )
        if self.stable_scan_count >= self.config.required_stable_scans:
            self.opened = True
        return self.opened

    def _odom_is_ready(self, now: float) -> bool:
        """确认必需的标定已完成、静止时间达标且里程计仍新鲜。"""
        return (
            self.imu_ready
            and self.stationary_since is not None
            and self.last_odom_time is not None
            and math.isfinite(now)
            and now - self.stationary_since
            >= self.config.stationary_duration_s
            and 0.0
            <= now - self.last_odom_time
            <= self.config.max_message_age_s
        )

    def _reset_scan_evidence(self) -> None:
        self.previous_ranges = None
        self.stable_scan_count = 0

    def _reset_evidence(self) -> None:
        self.stationary_since = None
        self.last_odom_time = None
        self.previous_yaw = None
        self._reset_scan_evidence()


def _yaw_from_odometry(message: Odometry) -> float:
    """从单位四元数提取平面 yaw；异常四元数返回 NaN 触发状态机重置。"""
    orientation = message.pose.pose.orientation
    values = (orientation.x, orientation.y, orientation.z, orientation.w)
    norm = math.sqrt(sum(value * value for value in values))
    if (
        any(not math.isfinite(value) for value in values)
        or abs(norm - 1.0) > 0.01
    ):
        return math.nan
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0
        - 2.0
        * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class MappingScanGateNode(Node):
    """保持原始 /scan 契约，并只把通过启动检查的新鲜扫描发布到内部话题。"""

    def __init__(self) -> None:
        super().__init__("kiwi_mapping_scan_gate")
        self.input_scan_topic = str(
            self._required("input_scan_topic", Parameter.Type.STRING)
        )
        self.output_scan_topic = str(
            self._required("output_scan_topic", Parameter.Type.STRING)
        )
        self.odom_topic = str(
            self._required("odom_topic", Parameter.Type.STRING)
        )
        self.imu_ready_topic = str(
            self._required("imu_ready_topic", Parameter.Type.STRING)
        )
        self.require_imu_ready = bool(
            self.declare_parameter("require_imu_ready", True).value
        )
        self.gate_ready_topic = str(
            self._required("gate_ready_topic", Parameter.Type.STRING)
        )
        self.laser_frame = str(
            self._required("laser_frame", Parameter.Type.STRING)
        )
        self.odom_frame = str(
            self._required("odom_frame", Parameter.Type.STRING)
        )
        self.base_frame = str(
            self._required("base_frame", Parameter.Type.STRING)
        )

        config = MappingGateConfig(
            stationary_duration_s=self._positive("stationary_duration_s"),
            max_linear_speed_m_s=self._positive("max_linear_speed_m_s"),
            max_angular_speed_rad_s=self._positive("max_angular_speed_rad_s"),
            max_yaw_step_rad=self._positive("max_yaw_step_rad"),
            max_message_age_s=self._positive("max_message_age_s"),
            required_stable_scans=self._positive_integer(
                "required_stable_scans"
            ),
            min_valid_scan_ratio=self._ratio("min_valid_scan_ratio"),
            min_scan_overlap_ratio=self._ratio("min_scan_overlap_ratio"),
            max_median_range_delta_m=self._positive(
                "max_median_range_delta_m"
            ),
        )
        if self.input_scan_topic == self.output_scan_topic:
            raise ValueError(
                "input_scan_topic and output_scan_topic must differ"
            )
        self.gate = MappingStartupGate(config, self.require_imu_ready)
        self.last_warning: dict[str, float] = {}

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scan_pub = self.create_publisher(
            LaserScan, self.output_scan_topic, qos_profile_sensor_data
        )
        self.ready_pub = self.create_publisher(
            Bool, self.gate_ready_topic, state_qos
        )
        self.ready_sub = (
            self.create_subscription(
                Bool, self.imu_ready_topic, self._on_imu_ready, state_qos
            )
            if self.require_imu_ready
            else None
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.ready_pub.publish(Bool(data=False))
        condition = (
            "calibrated IMU, stationary odometry and consistent scans"
            if self.require_imu_ready
            else "stationary laser odometry and consistent scans"
        )
        self.get_logger().info(
            f"mapping gate {self.input_scan_topic}->{self.output_scan_topic}; "
            f"waiting for {condition}"
        )

    def _required(self, name: str, parameter_type: Parameter.Type) -> object:
        parameter = self.declare_parameter(name, parameter_type)
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise RuntimeError(f"required parameter '{name}' is missing")
        if isinstance(parameter.value, str) and not parameter.value:
            raise ValueError(f"required parameter '{name}' must not be empty")
        return parameter.value

    def _positive(self, name: str) -> float:
        value = float(self._required(name, Parameter.Type.DOUBLE))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _positive_integer(self, name: str) -> int:
        value = int(self._required(name, Parameter.Type.INTEGER))
        if value < 1:
            raise ValueError(f"{name} must be positive")
        return value

    def _ratio(self, name: str) -> float:
        value = float(self._required(name, Parameter.Type.DOUBLE))
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError(f"{name} must be in (0, 1]")
        return value

    def _on_imu_ready(self, message: Bool) -> None:
        was_ready = self.gate.imu_ready
        self.gate.set_imu_ready(bool(message.data))
        if self.gate.imu_ready and not was_ready:
            self.get_logger().info(
                "base IMU calibration ready; verifying stationary scans "
                "before mapping"
            )

    def _on_odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self.odom_frame
            or message.child_frame_id != self.base_frame
        ):
            self.gate.observe_odometry(
                time.monotonic(), math.nan, 0.0, 0.0, 0.0
            )
            self._warn_throttled(
                "odom_frame", "mapping gate received unexpected odom frames"
            )
            return
        twist = message.twist.twist
        self.gate.observe_odometry(
            time.monotonic(),
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            _yaw_from_odometry(message),
        )

    def _on_scan(self, message: LaserScan) -> None:
        if message.header.frame_id != self.laser_frame:
            self.gate.observe_scan(time.monotonic(), (), 0.0, 0.0)
            self._warn_throttled(
                "scan_frame", "mapping gate received unexpected scan frame"
            )
            return
        was_open = self.gate.opened
        should_publish = self.gate.observe_scan(
            time.monotonic(),
            message.ranges,
            message.range_min,
            message.range_max,
        )
        if self.gate.opened and not was_open:
            self.ready_pub.publish(Bool(data=True))
            stable_count = self.gate.stable_scan_count
            self.get_logger().info(
                f"mapping scan gate opened after {stable_count} stable scans"
            )
        if should_publish:
            # 原消息原样转发，保留 frame、采样时间戳及 sensor-data QoS 语义。
            self.scan_pub.publish(message)

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.last_warning[key] = now


def main(args: list[str] | None = None) -> None:
    """运行建图首帧门控直到 ROS 关闭。"""
    rclpy.init(args=args)
    node = MappingScanGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Humble 的信号处理器可能已先执行 shutdown；重复调用会抛 RCLError。
        if rclpy.ok():
            rclpy.shutdown()
