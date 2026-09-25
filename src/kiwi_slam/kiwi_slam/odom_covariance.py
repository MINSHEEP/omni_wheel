"""给不发布协方差的 RF2O 里程计补上履带平台专属的不确定度。"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_delta(a: float, b: float) -> float:
    return math.remainder(a - b, 2.0 * math.pi)


@dataclass(frozen=True)
class StationaryFilterConfig:
    window_s: float
    max_scan_median_delta_m: float
    min_scan_overlap_ratio: float
    min_valid_scan_ratio: float
    max_translation_m: float
    max_yaw_rad: float
    max_gyro_rms_rad_s: float
    max_sensor_age_s: float

    def __post_init__(self) -> None:
        for name in ("window_s", "max_scan_median_delta_m", "max_translation_m",
                     "max_yaw_rad", "max_gyro_rms_rad_s", "max_sensor_age_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("min_scan_overlap_ratio", "min_valid_scan_ratio"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be in (0, 1]")


class StationaryOdomFilter:
    """Hold RF2O pose only with two seconds of matching scans, IMU and odom.

    The correction is an SE(2) offset. Updating it while held prevents the
    accumulated raw RF2O drift from reappearing as a jump when motion resumes.
    """

    def __init__(self, config: StationaryFilterConfig) -> None:
        self.config = config
        self.scans = deque()
        self.imu = deque()
        self.poses = deque()
        self.last_scan_time: float | None = None
        self.last_imu_time: float | None = None
        self.scan_stable = False
        self.offset = (0.0, 0.0, 0.0)
        self.held_pose: tuple[float, float, float] | None = None

    def _trim(self, history: deque, now: float) -> None:
        while len(history) > 1 and history[1][0] <= now - self.config.window_s:
            history.popleft()

    def observe_scan(self, message: LaserScan, now: float) -> None:
        if (not math.isfinite(now) or not math.isfinite(message.range_min)
                or not math.isfinite(message.range_max)
                or message.range_max <= message.range_min or not message.ranges):
            self.scan_stable = False
            self.scans.clear()
            return
        current = (now, tuple(message.ranges), message.range_min,
                   message.range_max, message.angle_min, message.angle_increment)
        self.scans.append(current)
        self._trim(self.scans, now)
        self.last_scan_time = now
        self.scan_stable = False
        if len(self.scans) < 2 or now - self.scans[0][0] < self.config.window_s:
            return
        old = self.scans[0]
        if (len(old[1]) != len(current[1]) or old[4:] != current[4:]):
            return

        def valid(value: float) -> bool:
            return math.isfinite(value) and current[2] <= value <= current[3]

        old_valid = sum(map(valid, old[1]))
        new_valid = sum(map(valid, current[1]))
        if min(old_valid, new_valid) < len(current[1]) * self.config.min_valid_scan_ratio:
            return
        paired = [abs(a - b) for a, b in zip(old[1], current[1])
                  if valid(a) and valid(b)]
        self.scan_stable = bool(
            paired
            and len(paired) >= min(old_valid, new_valid) * self.config.min_scan_overlap_ratio
            and statistics.median(paired) <= self.config.max_scan_median_delta_m
        )

    def observe_imu(self, message: Imu, now: float) -> None:
        rate = message.angular_velocity.z
        if not math.isfinite(now) or not math.isfinite(rate):
            self.imu.clear()
            self.last_imu_time = None
            return
        self.imu.append((now, rate))
        self._trim(self.imu, now)
        self.last_imu_time = now

    def _apply_offset(self, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, yaw = pose
        ox, oy, theta = self.offset
        return (ox + math.cos(theta) * x - math.sin(theta) * y,
                oy + math.sin(theta) * x + math.cos(theta) * y,
                math.remainder(theta + yaw, 2.0 * math.pi))

    def _anchor_offset(self, raw: tuple[float, float, float],
                       target: tuple[float, float, float]) -> None:
        theta = _angle_delta(target[2], raw[2])
        self.offset = (target[0] - math.cos(theta) * raw[0] + math.sin(theta) * raw[1],
                       target[1] - math.sin(theta) * raw[0] - math.cos(theta) * raw[1],
                       theta)

    def correct(self, message: Odometry, now: float) -> tuple[Odometry, bool]:
        raw = (message.pose.pose.position.x, message.pose.pose.position.y, _yaw(message))
        if not math.isfinite(now) or any(not math.isfinite(value) for value in raw):
            self.poses.clear()
            self.held_pose = None
            return message, False
        self.poses.append((now, raw))
        self._trim(self.poses, now)
        old = self.poses[0][1]
        displacement = math.hypot(raw[0] - old[0], raw[1] - old[1])
        yaw_change = abs(_angle_delta(raw[2], old[2]))
        imu_ready = bool(
            self.imu and self.last_imu_time is not None
            and 0.0 <= now - self.last_imu_time <= self.config.max_sensor_age_s
            and now - self.imu[0][0] >= self.config.window_s
            and math.sqrt(sum(rate * rate for _, rate in self.imu) / len(self.imu))
            <= self.config.max_gyro_rms_rad_s
        )
        stationary = bool(
            self.scan_stable and self.last_scan_time is not None
            and 0.0 <= now - self.last_scan_time <= self.config.max_sensor_age_s
            and now - self.poses[0][0] >= self.config.window_s
            and displacement <= self.config.max_translation_m
            and yaw_change <= self.config.max_yaw_rad and imu_ready
        )
        if stationary:
            if self.held_pose is None:
                self.held_pose = self._apply_offset(raw)
            corrected = self.held_pose
            self._anchor_offset(raw, corrected)
            message.twist.twist.linear.x = 0.0
            message.twist.twist.linear.y = 0.0
            message.twist.twist.angular.z = 0.0
        else:
            self.held_pose = None
            corrected = self._apply_offset(raw)
        message.pose.pose.position.x = corrected[0]
        message.pose.pose.position.y = corrected[1]
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(corrected[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(corrected[2] / 2.0)
        return message, stationary


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
        self.stationary_filter = None
        if self.declare_parameter("stationary_filter_enabled", False).value:
            self.stationary_filter = StationaryOdomFilter(StationaryFilterConfig(
                window_s=float(self.declare_parameter("stationary_window_s", 2.0).value),
                max_scan_median_delta_m=float(self.declare_parameter(
                    "stationary_max_scan_median_delta_m", 0.012).value),
                min_scan_overlap_ratio=float(self.declare_parameter(
                    "stationary_min_scan_overlap_ratio", 0.75).value),
                min_valid_scan_ratio=float(self.declare_parameter(
                    "stationary_min_valid_scan_ratio", 0.2).value),
                max_translation_m=float(self.declare_parameter(
                    "stationary_max_translation_m", 0.02).value),
                max_yaw_rad=float(self.declare_parameter(
                    "stationary_max_yaw_rad", 0.01).value),
                max_gyro_rms_rad_s=float(self.declare_parameter(
                    "stationary_max_gyro_rms_rad_s", 0.012).value),
                max_sensor_age_s=float(self.declare_parameter(
                    "stationary_max_sensor_age_s", 0.25).value),
            ))
            self.scan_sub = self.create_subscription(
                LaserScan, self._topic("scan_topic"), self._on_scan,
                qos_profile_sensor_data)
            self.imu_sub = self.create_subscription(
                Imu, self._topic("imu_topic"), self._on_imu,
                qos_profile_sensor_data)
        self.was_stationary = False
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
        if self.stationary_filter is not None:
            message, stationary = self.stationary_filter.correct(message, time.monotonic())
            if stationary != self.was_stationary:
                self.get_logger().info(
                    f"RF2O stationary hold {'active' if stationary else 'released'}")
                self.was_stationary = stationary
        self.publisher.publish(set_covariance(message, **self.variances))

    def _on_scan(self, message: LaserScan) -> None:
        self.stationary_filter.observe_scan(message, time.monotonic())

    def _on_imu(self, message: Imu) -> None:
        self.stationary_filter.observe_imu(message, time.monotonic())


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
