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
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import copy
from .laser_quality import ScanEvidence, PartialVelocityObservation, StationaryAccelerationVeto
from .reference_recovery import ReferenceRecovery, compose_pose, relative_pose


def _yaw(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_delta(a: float, b: float) -> float:
    return math.remainder(a - b, 2.0 * math.pi)


def preserve_covariance_floor(covariance, floor):
    """以 PSD 次序保留惯性桥累计不确定度，避免恢复后方差回落到固定值。"""
    covariance, floor = np.asarray(covariance), np.asarray(floor)
    difference = (floor-covariance + (floor-covariance).T)*.5
    eigen, vectors = np.linalg.eigh(difference)
    return covariance + vectors@np.diag(np.maximum(eigen, 0.))@vectors.T


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
    """Hold RF2O pose after two seconds of matching scans, IMU and odom.

    The correction is an SE(2) offset. Updating it while held prevents the
    accumulated raw RF2O drift from reappearing as a jump when motion resumes.
    Once held, RF2O's own displacement cannot end the hold: stationary tests
    show that RF2O can jump more than 20 cm while scans and IMU remain still.
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
            and (self.held_pose is not None or (
                displacement <= self.config.max_translation_m
                and yaw_change <= self.config.max_yaw_rad))
            and imu_ready
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
                    "stationary_max_scan_median_delta_m", 0.011).value),
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
        self.scan_frame = str(self.declare_parameter("scan_frame", "lidar_link").value)
        self.imu_frame = str(self.declare_parameter("imu_frame", "chassis_imu_link").value)
        self.quality_enabled = bool(self.declare_parameter("quality_enabled", False).value)
        self.acceleration_veto_enabled = bool(self.declare_parameter(
            'zero_velocity_acceleration_veto_enabled', False).value)
        self.acceleration_veto_received = None
        if self.acceleration_veto_enabled:
            if not self.quality_enabled:
                raise ValueError('acceleration veto requires laser quality checking')
            self.acceleration_veto = StationaryAccelerationVeto(
                float(self.declare_parameter('zero_velocity_acceleration_window_s', .2).value),
                float(self.declare_parameter('zero_velocity_acceleration_duration_s', .1).value),
                float(self.declare_parameter('zero_velocity_acceleration_holdoff_s', .3).value))
            self.acceleration_sub = self.create_subscription(Imu,
                self.declare_parameter('acceleration_topic', '/chassis/imu/acceleration').value,
                self._on_acceleration, qos_profile_sensor_data)
        self.calibrated = False
        self.calibrated_at = None
        self.quality_origin = False
        self.last_raw = None
        self.last_output = None
        self.last_match_pose = None
        self.held_reference = None
        self.last_scan = None
        self.last_scan_received = None
        self.scan_cache = {}
        self.pending_odometry = {}
        self.recovery_max_gap = float(self.declare_parameter("recovery_max_gap_s", 2.).value)
        self.inertial_recovery_enabled = bool(self.declare_parameter("inertial_recovery_enabled", False).value)
        self.inertial_max_age = float(self.declare_parameter("inertial_max_age_s", .15).value)
        if not 0 < self.inertial_max_age <= .25:
            raise ValueError('inertial_max_age_s must be in (0, .25]')
        self.inertial_history = deque(maxlen=500)
        self.inertial_valid = False
        self.inertial_valid_at = None
        self.recovery_covariance_floor = np.zeros((3, 3))
        self.evidence = ScanEvidence(float(self.declare_parameter("minimum_information_ratio", .015).value), self.recovery_max_gap)
        self.last_gyro = None
        self.gyro_history = deque()
        self.tracking_lost = False
        self.lidar_xy = list(self.declare_parameter("lidar_xy", [.24, 0.]).value)
        self.partial_enabled = bool(self.declare_parameter('partial_observation_enabled', False).value)
        self.partial = None
        self.last_partial_at = None
        if self.partial_enabled:
            if not self.quality_enabled:
                raise ValueError('partial observations require laser quality checking')
            self.partial = PartialVelocityObservation(self.lidar_xy,
                float(self.declare_parameter('partial_min_overlap', .8).value),
                float(self.declare_parameter('partial_max_residual_m', .02).value),
                float(self.declare_parameter('partial_confirmation_s', .2).value),
                float(self.declare_parameter('partial_max_gap_s', .3).value),
                [self.variances[name] for name in ('twist_vx_variance', 'twist_vy_variance', 'twist_wz_variance')])
        self.recovery = ReferenceRecovery(self.lidar_xy,
            self.recovery_max_gap,
            float(self.declare_parameter("recovery_duration_s", .2).value),
            float(self.declare_parameter("recovery_reference_max_gap_s", self.recovery_max_gap).value),
            float(self.declare_parameter("inertial_max_xy_variance", 4.).value),
            float(self.declare_parameter("inertial_max_yaw_variance", .15).value),
            float(self.declare_parameter("inertial_max_xy_growth_variance", .09).value),
            float(self.declare_parameter("inertial_max_yaw_growth_variance", .05).value))
        self.needs_recovery = False
        if len(self.lidar_xy) != 2 or not np.isfinite(self.lidar_xy).all():
            raise ValueError("invalid quality parameters")
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        heartbeat_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE,
                                   reliability=ReliabilityPolicy.RELIABLE)
        if self.partial_enabled:
            self.partial_pub = self.create_publisher(Odometry,
                self.declare_parameter('partial_output_topic', '/chassis/odom/laser_partial').value, 10)
            self.partial_ready_pub = self.create_publisher(Bool,
                self.declare_parameter('partial_ready_topic', '/chassis/odom/laser_partial_ready').value, state_qos)
        if self.quality_enabled and self.inertial_recovery_enabled:
            self.inertial_sub = self.create_subscription(Odometry,
                self.declare_parameter('inertial_topic', '/chassis/odom/inertial').value,
                self._on_inertial, qos_profile_sensor_data)
            self.inertial_valid_sub = self.create_subscription(Bool,
                self.declare_parameter('inertial_valid_topic', '/chassis/inertial/valid').value,
                self._on_inertial_valid, heartbeat_qos)
        self.stationary_pub = self.create_publisher(Bool, self.declare_parameter("stationary_topic", "/chassis/odom/calibration_stationary").value, state_qos)
        self.quality_pub = self.create_publisher(Bool, self.declare_parameter("quality_topic", "/chassis/odom/laser_ready").value, state_qos)
        self.zero_velocity_pub = self.create_publisher(Odometry, self.declare_parameter("zero_velocity_topic", "/chassis/odom/stationary").value, 10)
        self.validated_pub = self.create_publisher(LaserScan, self.declare_parameter("validated_scan_topic", "/scan_validated").value, qos_profile_sensor_data)
        self.calibration_sub = self.create_subscription(Bool, self.declare_parameter("imu_ready_topic", "/chassis/imu/calibration_ready").value, self._on_calibration,
            heartbeat_qos if self.quality_enabled else state_qos)
        self.last_published_at = None
        self.quality_timer = self.create_timer(.1, self._quality_watchdog)
        self.last_reason = None
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
        if self.quality_enabled:
            self._quality_odometry(message)
            return
        if self.stationary_filter is not None:
            message, stationary = self.stationary_filter.correct(message, time.monotonic())
            if stationary != self.was_stationary:
                self.get_logger().info(
                    f"RF2O stationary hold {'active' if stationary else 'released'}")
                self.was_stationary = stationary
        self.publisher.publish(set_covariance(message, **self.variances))

    def _on_calibration(self, message):
        self.calibrated = bool(message.data)
        self.calibrated_at = time.monotonic()

    def _on_inertial_valid(self, message):
        self.inertial_valid = bool(message.data)
        self.inertial_valid_at = time.monotonic()

    def _on_inertial(self, message):
        if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        pose = np.array([message.pose.pose.position.x, message.pose.pose.position.y, _yaw(message)])
        covariance = np.asarray(message.pose.covariance).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
        if not math.isfinite(stamp) or not np.isfinite(pose).all() or not np.isfinite(covariance).all():
            return
        if self.inertial_history and stamp <= self.inertial_history[-1]['stamp']:
            return
        self.inertial_history.append(dict(stamp=stamp, pose=pose, covariance=covariance, received=time.monotonic()))

    def _inertial_at(self, stamp):
        now = time.monotonic()
        if (not self.inertial_recovery_enabled or not self.inertial_valid
                or self.inertial_valid_at is None or now-self.inertial_valid_at > .3
                or not self.inertial_history
                or now-self.inertial_history[-1]['received'] > self.inertial_max_age
                or stamp is None):
            return None
        sample = min(self.inertial_history, key=lambda value: abs(value['stamp']-stamp))
        return sample if abs(sample['stamp']-stamp) <= self.inertial_max_age else None

    def _quality_watchdog(self):
        if self.quality_enabled:
            now = time.monotonic()
            fresh = self.last_published_at is not None and now-self.last_published_at < .25
            self.quality_pub.publish(Bool(data=bool(fresh and not self.needs_recovery and self.calibrated and self.calibrated_at is not None and now-self.calibrated_at < .5 and self.evidence.healthy and not self.tracking_lost)))
            if self.last_scan_received is None or now-self.last_scan_received > .25:
                self.stationary_pub.publish(Bool(data=False))
            if self.partial_enabled:
                self.partial_ready_pub.publish(Bool(data=bool(self.last_partial_at is not None
                    and now-self.last_partial_at < .25 and self.calibrated
                    and self.calibrated_at is not None and now-self.calibrated_at < .5)))

    def _publish_partial_observation(self, message, evidence, stamp):
        if not getattr(self, 'partial_enabled', False) or not self.quality_origin:
            return
        observation = self.partial.observe(evidence.local_match, stamp, evidence.local_matched_time)
        if observation is None:
            self.last_partial_at = None
            self.partial_ready_pub.publish(Bool(data=False))
            return
        velocity, planar_covariance = observation
        result = Odometry()
        result.header = copy.deepcopy(message.header)
        result.child_frame_id = self.base_frame
        result.pose.pose.orientation.w = 1.
        # pose 没有观测意义；EKF/INS 只能订阅下方有方向性协方差的 twist。
        result.pose.covariance = (np.eye(6)*1e6).ravel().tolist()
        result.twist.twist.linear.x, result.twist.twist.linear.y = map(float, velocity[:2])
        result.twist.twist.angular.z = float(velocity[2])
        covariance = np.eye(6)*1e6
        covariance[np.ix_([0, 1, 5], [0, 1, 5])] = planar_covariance
        result.twist.covariance = covariance.ravel().tolist()
        self.partial_pub.publish(result)
        self.last_partial_at = time.monotonic()
        self.partial_ready_pub.publish(Bool(data=True))

    def _clear_partial_observation(self):
        if getattr(self, 'partial_enabled', False):
            self.partial.reset()
            self.last_partial_at = None
            self.partial_ready_pub.publish(Bool(data=False))

    def _on_scan(self, message: LaserScan) -> None:
        now = time.monotonic()
        if self.quality_enabled:
            if message.header.frame_id != self.scan_frame:
                self.evidence.healthy = self.evidence.stationary = False
                self.quality_pub.publish(Bool(data=False))
                self.stationary_pub.publish(Bool(data=False))
                return
            if not self.quality_origin and self.evidence.time_discontinuous:
                self.evidence = ScanEvidence(self.evidence.information_ratio, self.recovery_max_gap)
            rate = math.nan
            if self.last_gyro is not None and now-self.last_gyro < .25 and self.gyro_history:
                rate = float(np.mean([w for _, w in self.gyro_history]))
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            # 连续性和匹配时间使用采样时间；回调排队延迟只影响新鲜度，
            # 不能把连续采样误判成永久传感器断流。
            self.evidence.observe(message, stamp, rate)
            self.last_scan = message
            self.last_scan_received = now
            self.scan_cache[stamp] = (message, copy.copy(self.evidence), now)
            for old in list(self.scan_cache):
                if len(self.scan_cache)>10:
                    self.scan_cache.pop(old)
            pending = self.pending_odometry.pop(stamp, None)
            if pending is not None:
                self._quality_odometry(pending)
            self.stationary_pub.publish(Bool(data=bool(self.evidence.healthy
                and OdometryCovarianceAdapter._zero_velocity_allowed(self, self.evidence, stamp))))
            if not self.evidence.healthy:
                self.quality_pub.publish(Bool(data=False))
            if self.evidence.reason != self.last_reason:
                self.get_logger().info("Laser quality: " + self.evidence.reason)
                self.last_reason = self.evidence.reason
        else:
            self.stationary_filter.observe_scan(message, now)

    def _on_acceleration(self, message):
        # 只接受已去重力、去偏且位于底盘系的观测；原始C板重力不可参与此判据。
        if not self.calibrated or message.header.frame_id != self.base_frame:
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        sensor_age = self.get_clock().now().nanoseconds*1e-9-stamp
        if not math.isfinite(sensor_age) or not -.05 <= sensor_age <= .25:
            return
        covariance = np.asarray(message.linear_acceleration_covariance).reshape(3, 3)[:2, :2]
        acceleration = [message.linear_acceleration.x, message.linear_acceleration.y]
        significant = self.acceleration_veto.push(stamp, acceleration, covariance)
        self.acceleration_veto_received = time.monotonic()
        if significant:
            self.stationary_pub.publish(Bool(data=False))

    def _zero_velocity_allowed(self, evidence, stamp):
        # 位姿暂时保持不等于仍有零速测量；模糊/起步确认阶段必须撤销强零速约束。
        if not evidence.stationary or evidence.reason != 'stationary':
            return False
        if not getattr(self, 'acceleration_veto_enabled', False):
            return True
        received = self.acceleration_veto_received
        return not (received is not None and time.monotonic()-received <= .25
                    and self.acceleration_veto.active(stamp))

    def _replay_scans_after_recovery(self, stamp):
        # RF2O通常晚于scan；缓存的未来扫描仍属于旧失败参考，必须按新参考重新计算，
        # 不能把旧healthy=false快照当成又一次断流，也不能未经匹配直接改成true。
        for future in sorted(key for key in self.scan_cache if key > stamp):
            scan, cached, received = self.scan_cache[future]
            self.evidence.observe(scan, future, cached.last_gyro_rate)
            self.scan_cache[future] = (scan, copy.copy(self.evidence), received)

    def _on_imu(self, message: Imu) -> None:
        now = time.monotonic()
        rate = message.angular_velocity.z
        if self.quality_enabled and message.header.frame_id != self.imu_frame:
            self.last_gyro = None
            self.gyro_history.clear()
            return
        if math.isfinite(rate):
            self.last_gyro = now
            self.gyro_history.append((now, rate))
            while self.gyro_history and now-self.gyro_history[0][0] > .15:
                self.gyro_history.popleft()
        self.stationary_filter.observe_imu(message, now)

    def _quality_odometry(self, message):
        now = time.monotonic()
        ev = self.evidence
        # IMU 暂不可用时保留可信参考，恢复后核对累计运动。
        if not self.calibrated or self.calibrated_at is None or now-self.calibrated_at > .5:
            self.needs_recovery = self.quality_origin
            self.quality_pub.publish(Bool(data=False))
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        pair = self.scan_cache.get(stamp)
        if pair is None:
            self.pending_odometry[stamp] = message
            for old in list(self.pending_odometry):
                if len(self.pending_odometry)>10:
                    self.pending_odometry.pop(old)
            return
        scan, ev, received = pair
        if now-received > .25 or (self.last_output is not None and stamp <= self.last_output[0]):
            return
        raw = (message.pose.pose.position.x, message.pose.pose.position.y, _yaw(message))
        if not np.isfinite(raw).all():
            self.needs_recovery = self.quality_origin
            self.quality_pub.publish(Bool(data=False))
            if getattr(self, 'partial_enabled', False):
                self._publish_partial_observation(message, ev, stamp)
            return
        if not ev.healthy:
            self.needs_recovery = self.quality_origin
            if not self.needs_recovery:
                self.quality_pub.publish(Bool(data=False))
                return
        if self.tracking_lost:
            self.quality_pub.publish(Bool(data=False))
            self.get_logger().error("Laser reference expired; verified relocalization or restart required", throttle_duration_sec=5.)
            if getattr(self, 'partial_enabled', False):
                self._publish_partial_observation(message, ev, stamp)
            return
        if self.last_output is not None and stamp-self.last_output[0] > .25:
            self.needs_recovery = True
        f = self.stationary_filter
        if self.needs_recovery:
            # 当前相邻扫描本身缺少一个自由度时，全位姿恢复没有几何依据。
            # 保留强方向速度观测，不反复执行多组完整位姿 ICP。
            if (getattr(self, 'partial_enabled', False) and ev.local_match is not None
                    and ev.local_match.matched and ev.local_match.observable_rank < 3):
                self._publish_partial_observation(message, ev, stamp)
                self.quality_pub.publish(Bool(data=False))
                return
            if not self.recovery.check(scan, stamp, raw, ev.stationary, self.evidence.information_ratio,
                    inertial=self._inertial_at(stamp),
                    local_match=ev.local_match, local_stamp=ev.local_matched_time,
                    local_inertial=self._inertial_at(ev.local_matched_time),
                    local_healthy=ev.local_healthy):
                if self.recovery.reason == 'reference_expired_restart_required':
                    self.tracking_lost = True
                self.quality_pub.publish(Bool(data=False))
                self.get_logger().warning('Laser recovery: '+self.recovery.reason, throttle_duration_sec=2.)
                if getattr(self, 'partial_enabled', False):
                    self._publish_partial_observation(message, ev, stamp)
                return
            if not self.evidence.accept_recovered_reference(scan, stamp):
                self.quality_pub.publish(Bool(data=False))
                return
            ev = copy.copy(self.evidence)
            OdometryCovarianceAdapter._replay_scans_after_recovery(self, stamp)
            f._anchor_offset(raw, self.recovery.recovered_pose)
            f.held_pose = None
            self.held_reference = None
            self.recovery_covariance_floor = preserve_covariance_floor(
                self.recovery.recovered_covariance, self.recovery_covariance_floor)
            self.last_match_pose = None
            self.needs_recovery = False
            self.get_logger().info('Laser '+self.recovery.reason+'; continuing original odom frame')
        if not self.quality_origin:
            if not ev.stationary:
                return
            f.held_pose = (0.,0.,0.)
            f._anchor_offset(raw, f.held_pose)
            self.quality_origin = True
        local = ev.last_match.pose.copy()
        c0,s0 = math.cos(local[2]),math.sin(local[2])
        rotation0 = np.array([[c0,-s0],[s0,c0]])
        local[:2] += np.asarray(self.lidar_xy)-rotation0@np.asarray(self.lidar_xy)
        if ev.stationary:
            if f.held_pose is None:
                f.held_pose = f._apply_offset(raw)
            corrected = f.held_pose
            if getattr(self, 'held_reference', None) is None:
                self.held_reference = (ev.matched_reference_time, local.copy(), np.asarray(corrected).copy())
            f._anchor_offset(raw, corrected)
            message.twist.twist.linear.x = message.twist.twist.linear.y = message.twist.twist.angular.z = 0.
        else:
            held_reference = getattr(self, 'held_reference', None)
            if f.held_pose is not None and held_reference is not None and held_reference[0] == ev.matched_reference_time:
                # 起步确认期间累计的真实位移要补回，不能被停车 offset 吞掉。
                corrected_release = compose_pose(held_reference[2], relative_pose(held_reference[1], local))
                f._anchor_offset(raw, corrected_release)
                self.last_match_pose = None
            f.held_pose = None
            self.held_reference = None
            corrected = f._apply_offset(raw)
            if self.last_output is not None:
                dt = stamp-self.last_output[0]
                distance = math.hypot(corrected[0]-self.last_output[1][0], corrected[1]-self.last_output[1][1])
                angle = abs(_angle_delta(corrected[2], self.last_output[1][2]))
                # 硬物理界限之外的 RF2O 突跳不可因协方差大就通过；不悄悄重置参考。
                if dt <= 0 or distance > .7*dt+.03 or angle > 1.5*dt+.03:
                    self.needs_recovery = True
                    self.quality_pub.publish(Bool(data=False))
                    self.get_logger().error(
                        f"RF2O physical increment rejected: stamp={stamp:.9f}, "
                        f"dt={dt:.4f}s, distance={distance:.4f}m, angle={angle:.4f}rad; verifying retained reference")
                    if getattr(self, 'partial_enabled', False):
                        self._publish_partial_observation(message, ev, stamp)
                    return
        if not ev.stationary and self.last_match_pose is not None and self.last_output is not None:
            ref, previous = self.last_match_pose
            if ref == ev.matched_reference_time:
                angle0 = self.last_output[1][2]-previous[2]
                c0,s0 = math.cos(angle0),math.sin(angle0)
                rotation0 = np.array([[c0,-s0],[s0,c0]])
                predicted_xy = np.asarray(self.last_output[1][:2])+rotation0@(local[:2]-previous[:2])
                disagreement = np.linalg.norm(np.asarray(corrected[:2])-predicted_xy)
                disagreement_yaw = abs(_angle_delta(corrected[2]-self.last_output[1][2], local[2]-previous[2]))
                if disagreement > .06 or disagreement_yaw > .08:
                    self.needs_recovery = True
                    self.quality_pub.publish(Bool(data=False))
                    self.get_logger().warning(
                        f"RF2O increment disagrees with background alignment; verifying retained reference: "
                        f"stamp={stamp:.9f}, dt={stamp-self.last_output[0]:.4f}s, "
                        f"translation_error={disagreement:.4f}m (limit=0.06), "
                        f"yaw_error={disagreement_yaw:.4f}rad (limit=0.08), "
                        f"overlap={ev.last_match.overlap:.3f}, residual={ev.last_match.residual:.4f}m, "
                        f"information_ratio={ev.last_match.condition:.5f}, state={ev.reason}")
                    if getattr(self, 'partial_enabled', False):
                        self._publish_partial_observation(message, ev, stamp)
                    return
        self.last_match_pose = (ev.matched_reference_time, local)
        message.pose.pose.position.x, message.pose.pose.position.y = corrected[:2]
        message.pose.pose.orientation.x = message.pose.pose.orientation.y = 0.
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(corrected[2]/2), math.cos(corrected[2]/2)
        if not ev.stationary and self.last_output is not None:
            dt = stamp-self.last_output[0]
            delta = np.asarray(corrected[:2])-np.asarray(self.last_output[1][:2])
            c, s = math.cos(corrected[2]), math.sin(corrected[2])
            velocity = np.array([[c, s], [-s, c]])@delta/dt
            message.twist.twist.linear.x, message.twist.twist.linear.y = map(float, velocity)
            message.twist.twist.angular.z = _angle_delta(corrected[2], self.last_output[1][2])/dt
        set_covariance(message, **self.variances)
        # 归一化几何方向 -> 雷达系平移协方差 -> 机体系 -> odom 系。
        match = ev.last_match
        rx,ry = self.lidar_xy
        lever_jacobian = np.array([[1.,0.,ry],[0.,1.,-rx]])
        geometry = lever_jacobian @ match.covariance @ lever_jacobian.T
        eigen, vectors = np.linalg.eigh(geometry)
        shape = vectors @ np.diag(np.clip(eigen/max(float(eigen.min()), 1e-8), 1., 100.)) @ vectors.T
        inflation = max(1., (match.residual/.01)**2) / max(match.overlap, .1)
        c,s = math.cos(corrected[2]), math.sin(corrected[2])
        rotation = np.array([[c,-s],[s,c]])
        xy = rotation @ shape @ rotation.T * self.variances['pose_xy_variance'] * inflation
        message.pose.covariance[0], message.pose.covariance[1] = float(xy[0,0]), float(xy[0,1])
        message.pose.covariance[6], message.pose.covariance[7] = float(xy[1,0]), float(xy[1,1])
        for index in (0,7,35):
            message.twist.covariance[index] *= inflation
        message.twist.covariance[0] *= float(shape[0,0])
        message.twist.covariance[7] = max(message.twist.covariance[7], .1 if not ev.stationary else .01)
        message.pose.covariance[35] *= inflation
        planar_indices = [0, 1, 5]
        covariance = np.asarray(message.pose.covariance).reshape(6, 6)
        planar_covariance = preserve_covariance_floor(
            covariance[np.ix_(planar_indices, planar_indices)], self.recovery_covariance_floor)
        covariance[np.ix_(planar_indices, planar_indices)] = planar_covariance
        message.pose.covariance = covariance.ravel().tolist()
        # 瞬时扫描几何噪声不属于原点累计误差；否则一次近退化测量会永久锁住大方差。
        # 真正由 INS 跨失效段带入的累计不确定度仍通过 floor 保留。
        self.recovery.accept(scan, stamp, raw, corrected, self.recovery_covariance_floor, planar_covariance)
        self.last_output = (stamp,corrected)
        self.last_published_at = now
        self.publisher.publish(message)
        if OdometryCovarianceAdapter._zero_velocity_allowed(self, ev, stamp):
            # 条件零速度约束；仅在背景匹配确认停车时发布，不把 RF2O vy=0 当常态测量。
            zero = Odometry()
            zero.header = message.header
            zero.child_frame_id = message.child_frame_id
            zero.pose.covariance = [0.] * 36
            zero.twist.covariance = [0.] * 36
            for index in (0,7,14,21,28,35):
                zero.twist.covariance[index] = .0004 if index in (0,7) else 1e6
            self.zero_velocity_pub.publish(zero)
        self.quality_pub.publish(Bool(data=True))
        self.validated_pub.publish(scan)
        if getattr(self, 'partial_enabled', False):
            self._clear_partial_observation()


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
