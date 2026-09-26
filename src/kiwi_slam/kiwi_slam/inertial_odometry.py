"""有可信激光锚点的短时平面惯性里程计；不发布 TF，不提供绝对定位。

输入为 planar_imu 已校正的车体系加速度和角速度，不能再次去重力、旋转
或缩放。恒速运动与静止都有零加速度，因此此处从不凭 IMU 判定停车。
纯数值核心不依赖 ROS，延迟到达的激光按采样时间重放 IMU，避免时钟错位。
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class InertialConfig:
    max_coast_s: float = 2.0
    max_imu_gap_s: float = 0.2
    anchor_window_s: float = 0.45
    anchor_min_span_s: float = 0.25
    anchor_min_samples: int = 4
    anchor_max_gap_s: float = 0.25
    anchor_fit_residual_m: float = 0.02
    max_speed_m_s: float = 0.8
    max_yaw_rate_rad_s: float = 2.0
    max_acceleration_m_s2: float = 1.5
    velocity_variance_floor: float = 0.01
    acceleration_bias_std: float = 0.05
    gyro_bias_std: float = 0.01
    max_position_variance: float = 4.0
    max_position_growth_variance: float = 0.09
    max_velocity_variance: float = 0.5
    max_yaw_variance: float = 0.5
    max_yaw_growth_variance: float = 0.05
    partial_max_velocity_variance: float = 0.25
    partial_velocity_innovation_limit_m_s: float = 0.3

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid inertial parameter: {name}')
        if not 0.05 <= self.max_imu_gap_s < self.max_coast_s <= 5.0:
            raise ValueError('inertial prediction must be bounded to at most 5 seconds')
        if not self.anchor_min_span_s < self.anchor_window_s <= 1.0:
            raise ValueError('invalid anchor velocity window')
        if self.anchor_min_samples < 4 or int(self.anchor_min_samples) != self.anchor_min_samples:
            raise ValueError('at least four laser anchors are required for velocity')
        if self.velocity_variance_floor >= self.max_velocity_variance:
            raise ValueError('velocity uncertainty floor exceeds bound')


@dataclass
class InertialEstimate:
    stamp: float
    state: np.ndarray  # odom x, odom y, yaw, odom vx, odom vy
    covariance: np.ndarray
    angular_velocity: float
    angular_variance: float
    anchor_stamp: float
    valid: bool
    reason: str


def _covariance(value, size):
    covariance = np.asarray(value, dtype=float)
    if covariance.shape != (size, size) or not np.isfinite(covariance).all():
        raise ValueError('invalid covariance dimensions or entries')
    if not np.allclose(covariance, covariance.T, atol=1e-8):
        raise ValueError('covariance must be symmetric')
    if np.linalg.eigvalsh(covariance).min() < -1e-9:
        raise ValueError('covariance must be positive semidefinite')
    return covariance.copy()


class InertialOdometry:
    """受限航位推算。可信锚点只能来自外部核验的激光，不能用本输出回灌。"""
    def __init__(self, config=None):
        self.config = config or InertialConfig()
        self.samples = deque()
        self.anchors = deque()
        self.pending_anchor = None
        self.state = self.covariance = None
        self.anchor_position_covariance = None
        self.anchor_yaw_variance = None
        self.stamp = self.anchor_stamp = None
        self.last_sample = None
        self.reason = 'waiting_trusted_anchor_velocity'
        self.last_anchor_rejection = None
        self.last_partial_stamp = None
        self.last_partial_rejection = None
        self.last_imu_rejection_detail = None

    def invalidate(self, reason):
        self.state = self.covariance = None
        self.anchor_position_covariance = None
        self.anchor_yaw_variance = None
        self.stamp = self.anchor_stamp = None
        self.pending_anchor = None
        self.anchors.clear()
        self.samples.clear()
        self.last_sample = None
        self.last_partial_stamp = None
        self.last_partial_rejection = None
        self.reason = str(reason)

    def add_anchor(self, stamp, pose, covariance):
        """接受激光位姿；从多帧回归速度，位置校正跳变不转成瞬时速度。

调用者必须先核验 laser_ready、frame 和消息新鲜度。已有状态允许短时
拒绝坏锚点，但拒绝的锚点不会刷新 coast 时限。失效后必须重新积累窗口。
"""
        pose = np.asarray(pose, dtype=float)
        if not math.isfinite(stamp) or pose.shape != (3,) or not np.isfinite(pose).all():
            self.last_anchor_rejection = 'anchor_nonfinite'
            return False
        try:
            covariance = _covariance(covariance, 3)
        except ValueError:
            self.last_anchor_rejection = 'anchor_covariance_invalid'
            return False
        if self.anchor_stamp is not None and stamp <= self.anchor_stamp:
            self.last_anchor_rejection = 'anchor_out_of_order'
            return False
        if self.anchors:
            old_stamp, old_pose, _ = self.anchors[-1]
            dt = stamp - old_stamp
            if dt <= 0:
                self.last_anchor_rejection = 'anchor_out_of_order'
                return False
            if dt > self.config.anchor_max_gap_s:
                self.anchors.clear()
            elif (np.linalg.norm(pose[:2] - old_pose[:2]) >
                  self.config.max_speed_m_s * dt + self.config.anchor_fit_residual_m or
                  abs(math.remainder(pose[2] - old_pose[2], 2*math.pi)) >
                  self.config.max_yaw_rate_rad_s * dt + 0.03):
                # 清除回归窗口；即使新的位置纠正是合法的，也须重新观察速度。
                self.anchors.clear()
                self.anchors.append((stamp, pose.copy(), covariance))
                self.last_anchor_rejection = 'anchor_jump_velocity_unverified'
                return False
        self.anchors.append((stamp, pose.copy(), covariance))
        while len(self.anchors) > 1 and stamp - self.anchors[0][0] > self.config.anchor_window_s:
            self.anchors.popleft()
        if (len(self.anchors) < self.config.anchor_min_samples or
                stamp - self.anchors[0][0] < self.config.anchor_min_span_s - 1e-8):
            self.last_anchor_rejection = 'anchor_velocity_window_incomplete'
            return False
        # 激光可先于对应 IMU 到达；等整个回归窗口有 IMU 后才计算末端速度。
        self.pending_anchor = (stamp, pose.copy(), covariance, tuple(self.anchors))
        return self._replay_anchor()

    def _anchor_velocity(self, stamp, pose, covariance, anchors):
        times = np.array([row[0] - stamp for row in anchors])
        positions = np.array([row[1][:2] for row in anchors])
        centered = times - times.mean()
        spread = float(centered @ centered)
        raw_velocity = centered @ (positions - positions.mean(axis=0)) / spread
        raw_residual = positions - positions.mean(axis=0) - np.outer(centered, raw_velocity)
        samples = list(self.samples)
        knots = sorted(set([row[0] for row in anchors] +
                           [row[0] for row in samples if anchors[0][0] < row[0] < stamp]))
        window = []
        index = 0
        for knot in knots:
            while index + 1 < len(samples) and samples[index + 1][0] <= knot:
                index += 1
            if abs(samples[index][0] - knot) <= 1e-8:
                window.append((knot, *samples[index][1:]))
            elif index + 1 < len(samples):
                window.append(self._interpolate(samples[index], samples[index + 1], knot))
            else:
                self.last_anchor_rejection = 'anchor_velocity_imu_history_incomplete'
                return None
        # 用锚点 yaw 回推窗口初始姿态，再按样本时间积分车体系加速度。
        turn = sum((b[2] + a[2]) * .5 * (b[0] - a[0])
                   for a, b in zip(window, window[1:]))
        yaw = pose[2] - turn
        delta_velocity = np.zeros(2)
        delta_position = np.zeros(2)
        displacement = {window[0][0]: delta_position.copy()}
        for first, second in zip(window, window[1:]):
            dt = second[0] - first[0]
            rate = (first[2] + second[2]) * .5
            angle = yaw + rate * dt * .5
            c, s = math.cos(angle), math.sin(angle)
            acceleration = np.array([[c, -s], [s, c]]) @ ((first[1] + second[1]) * .5)
            delta_position += delta_velocity * dt + acceleration * dt * dt * .5
            delta_velocity += acceleration * dt
            yaw += rate * dt
            displacement[second[0]] = delta_position.copy()
        inertial_positions = np.array([displacement[row[0]] for row in anchors])
        inertial_slope = centered @ (inertial_positions - inertial_positions.mean(axis=0)) / spread
        # 回归得到的是窗口平均速度。仅以同窗 IMU 补偿到最新锚点时刻，
        # 不把加速造成的正常曲率误当噪声，也不引入高噪声二阶位置差分。
        velocity = raw_velocity + delta_velocity - inertial_slope
        corrected = positions - inertial_positions
        residual = corrected - corrected.mean(axis=0) - np.outer(centered, raw_velocity - inertial_slope)
        max_residual = float(np.linalg.norm(residual, axis=1).max())
        if max_residual > self.config.anchor_fit_residual_m:
            self.last_anchor_rejection = 'anchor_velocity_fit_unstable'
            return None
        if np.linalg.norm(velocity) > self.config.max_speed_m_s:
            self.last_anchor_rejection = 'anchor_velocity_implausible'
            return None
        # 激光位姿误差时间相关，不假设 N 帧独立后把协方差压到近零。
        velocity_covariance = raw_residual.T @ raw_residual / max(1, len(times)-2) / spread
        velocity_covariance += np.eye(2) * self.config.velocity_variance_floor
        elapsed = times - times[0]
        effective_lag = elapsed[-1] - float(centered @ (.5 * elapsed**2) / spread)
        # 补偿噪声与偏置可在窗口内相关，按最坏方向方差计入，不能按 N 稀释。
        acceleration_variance = max(float(np.linalg.eigvalsh(row[3]).max()) for row in window)
        acceleration_peak = max(float(np.linalg.norm(row[1])) for row in window)
        yaw_variance = covariance[2, 2] + elapsed[-1]**2 * (
            max(row[4] for row in window) + self.config.gyro_bias_std**2)
        compensation_variance = effective_lag**2 * (acceleration_variance +
            self.config.acceleration_bias_std**2 + acceleration_peak**2 * yaw_variance)
        velocity_covariance += np.eye(2) * compensation_variance
        state = np.r_[pose, velocity]
        matrix = np.zeros((5, 5))
        matrix[:3, :3] = covariance
        matrix[3:, 3:] = velocity_covariance
        return state, matrix

    @staticmethod
    def _interpolate(first, second, stamp):
        if second[0] <= first[0]:
            return first
        fraction = (stamp-first[0])/(second[0]-first[0])
        return (stamp, *(first[i] + fraction*(second[i]-first[i]) for i in range(1, 5)))

    def _replay_anchor(self):
        if self.pending_anchor is None or not self.samples:
            return False
        stamp, pose, covariance, anchors = self.pending_anchor
        latest = self.samples[-1][0]
        if stamp > latest + 1e-8:
            return False  # 激光先到，等待该采样时刻的 IMU。
        if anchors[0][0] < self.samples[0][0] - 1e-8:
            self.last_anchor_rejection = 'anchor_velocity_imu_history_incomplete'
            self.pending_anchor = None
            return False
        fitted = self._anchor_velocity(stamp, pose, covariance, anchors)
        if fitted is None:
            self.pending_anchor = None
            return False
        state, covariance = fitted
        self.last_anchor_rejection = None
        before = self.samples[0]
        replay = []
        for sample in self.samples:
            if sample[0] <= stamp + 1e-8:
                before = sample
            else:
                if not replay:
                    before = self._interpolate(before, sample, stamp)
                replay.append(sample)
        self.state, self.covariance = state.copy(), covariance.copy()
        self.anchor_position_covariance = covariance[:2, :2].copy()
        self.anchor_yaw_variance = float(covariance[2, 2])
        self.stamp, self.anchor_stamp = stamp, stamp
        self.last_sample = before
        self.pending_anchor = None
        self.reason = 'anchored'
        for sample in replay:
            if not self._advance(sample):
                return False
        return True

    def add_partial_velocity(self, stamp, body_velocity, covariance):
        """仅吸收部分可观测的速度方向，不重锚位置、不刷新全观测时限。

调用者须先核验 partial_ready。激光增量与原锚点可能相关，因此仅小比例
调整速度均值，保留已有协方差并额外计入观测噪声；不宣称独立信息收敛。
"""
        velocity = np.asarray(body_velocity, dtype=float)
        if (self.state is None or not math.isfinite(stamp) or velocity.shape != (2,)
                or not np.isfinite(velocity).all()):
            self.last_partial_rejection = 'partial_invalid_or_unanchored'
            return False
        try:
            covariance = _covariance(covariance, 2)
        except ValueError:
            self.last_partial_rejection = 'partial_covariance_invalid'
            return False
        if (stamp < self.anchor_stamp - 1e-8 or stamp > self.stamp + 1e-8 or
                self.stamp - stamp > self.config.max_imu_gap_s or
                (self.last_partial_stamp is not None and stamp <= self.last_partial_stamp) or
                not self.estimate(self.stamp).valid):
            self.last_partial_rejection = 'partial_timestamp_or_prediction_invalid'
            return False
        samples = [row for row in self.samples if row[0] <= self.stamp + 1e-8]
        before = None
        window = []
        for row in samples:
            if row[0] <= stamp:
                before = row
            else:
                if not window:
                    if before is None:
                        self.last_partial_rejection = 'partial_imu_history_incomplete'
                        return False
                    window.append(self._interpolate(before, row, stamp))
                window.append(row)
        if not window:
            window = [self.last_sample]
        turn = sum((a[2]+b[2])*.5*(b[0]-a[0]) for a, b in zip(window, window[1:]))
        yaw = self.state[2] - turn
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        delta_velocity = np.zeros(2)
        for first, second in zip(window, window[1:]):
            dt = second[0]-first[0]
            rate = (first[2]+second[2])*.5
            angle = yaw + rate*dt*.5
            c, s = math.cos(angle), math.sin(angle)
            delta_velocity += np.array([[c, -s], [s, c]]) @ ((first[1]+second[1])*.5) * dt
            yaw += rate*dt
        eigenvalues, directions = np.linalg.eigh(covariance)
        updates = []
        for variance, direction in zip(eigenvalues, directions.T):
            if not 0 < variance <= self.config.partial_max_velocity_variance:
                continue  # 1e6 等弱方向完全不提供观测，不形成伪零速约束。
            axis = rotation @ direction
            observation = float(direction @ velocity)
            predicted = float(axis @ (self.state[3:]-delta_velocity))
            innovation = observation-predicted
            if (abs(observation) > self.config.max_speed_m_s or
                    abs(innovation) > self.config.partial_velocity_innovation_limit_m_s):
                self.last_partial_rejection = 'partial_velocity_innovation_implausible'
                return False
            # body→odom 旋转也有 yaw 不确定性，不能只复制激光标量方差。
            derivative = float(np.array([-axis[1], axis[0]]) @ (self.state[3:]-delta_velocity))
            variance += derivative**2 * self.covariance[2, 2]
            prior = float(axis @ self.covariance[3:, 3:] @ axis)
            gain = min(.25, prior/(prior+variance))
            updates.append((axis, innovation, variance, gain))
        if not updates:
            self.last_partial_rejection = 'partial_no_observable_velocity_direction'
            return False
        correction = sum((axis*innovation*gain for axis, innovation, _, gain in updates), np.zeros(2))
        if np.linalg.norm(self.state[3:]+correction) > self.config.max_speed_m_s:
            self.last_partial_rejection = 'partial_velocity_outside_validated_limit'
            return False
        self.state[3:] += correction
        age = self.stamp-stamp
        for axis, innovation, variance, gain in updates:
            self.covariance[3:, 3:] += np.outer(axis, axis)*(2*gain*gain*variance)
            # 延迟观测不让已发布位置跳变，遗漏的短时位置修正计入误差预算。
            self.covariance[:2, :2] += np.outer(axis, axis)*(age*gain*innovation)**2
        self.last_partial_stamp = stamp
        self.last_partial_rejection = None
        return True

    def push_imu(self, stamp, acceleration, angular_velocity, acceleration_covariance, angular_variance):
        self.last_imu_rejection_detail = None
        acceleration = np.asarray(acceleration, dtype=float)
        if (not math.isfinite(stamp) or acceleration.shape != (2,) or
                not np.isfinite(acceleration).all() or not math.isfinite(angular_velocity) or
                not math.isfinite(angular_variance) or angular_variance <= 0):
            self.invalidate('imu_nonfinite_or_invalid')
            return False
        try:
            covariance = _covariance(acceleration_covariance, 2)
        except ValueError:
            self.invalidate('imu_covariance_invalid')
            return False
        acceleration_norm = float(np.linalg.norm(acceleration))
        acceleration_exceeded = acceleration_norm > self.config.max_acceleration_m_s2
        yaw_rate_exceeded = abs(angular_velocity) > self.config.max_yaw_rate_rad_s
        if acceleration_exceeded or yaw_rate_exceeded:
            self.invalidate('imu_motion_outside_validated_limits')
            self.last_imu_rejection_detail = {
                'stamp': float(stamp),
                'ax_m_s2': float(acceleration[0]),
                'ay_m_s2': float(acceleration[1]),
                'acceleration_norm_m_s2': acceleration_norm,
                'max_acceleration_m_s2': self.config.max_acceleration_m_s2,
                'wz_rad_s': float(angular_velocity),
                'max_yaw_rate_rad_s': self.config.max_yaw_rate_rad_s,
                'trigger': ','.join(name for name, exceeded in (
                    ('acceleration_norm', acceleration_exceeded),
                    ('yaw_rate', yaw_rate_exceeded)) if exceeded),
            }
            return False
        sample = (float(stamp), acceleration.copy(), float(angular_velocity),
                  covariance, float(angular_variance))
        if self.samples:
            dt = stamp-self.samples[-1][0]
            if dt <= 0:
                if dt < -self.config.max_imu_gap_s:
                    self.invalidate('imu_clock_reversed')
                return False
            if dt > self.config.max_imu_gap_s:
                self.invalidate('imu_gap_requires_new_anchor')
        self.samples.append(sample)
        while len(self.samples) > 2 and stamp-self.samples[1][0] > 2.0:
            self.samples.popleft()
        if self.pending_anchor is not None and self._replay_anchor():
            return True
        if self.state is None:
            return False
        return self._advance(sample)

    def _advance(self, sample):
        if sample[0]-self.anchor_stamp > self.config.max_coast_s + 1e-8:
            self.reason = 'coast_timeout'
            return False
        dt = sample[0]-self.stamp
        if dt <= 1e-8:
            return dt >= -1e-8
        if dt > self.config.max_imu_gap_s + 1e-8:
            self.invalidate('imu_replay_gap_requires_new_anchor')
            return False
        previous = self.last_sample
        acceleration = (previous[1]+sample[1])*.5
        rate = (previous[2]+sample[2])*.5
        angle = self.state[2] + rate*dt*.5
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        global_acceleration = rotation @ acceleration
        derivative = np.array([-global_acceleration[1], global_acceleration[0]])
        transition = np.eye(5)
        transition[:2, 3:] = np.eye(2)*dt
        transition[:2, 2] = derivative*dt*dt*.5
        transition[3:, 2] = derivative*dt
        influence = np.zeros((5, 3))
        influence[:2, :2] = rotation*dt*dt*.5
        influence[3:, :2] = rotation*dt
        influence[2, 2] = dt
        influence[:2, 2] = derivative*dt**3*.25
        influence[3:, 2] = derivative*dt*dt*.5
        noise = np.zeros((3, 3))
        noise[:2, :2] = (previous[3]+sample[3])*.5
        noise[2, 2] = (previous[4]+sample[4])*.5
        self.covariance = transition @ self.covariance @ transition.T + influence @ noise @ influence.T
        self.covariance = (self.covariance+self.covariance.T)*.5
        self.state[:2] += self.state[3:]*dt + global_acceleration*dt*dt*.5
        self.state[3:] += global_acceleration*dt
        self.state[2] = math.remainder(self.state[2]+rate*dt, 2*math.pi)
        self.stamp, self.last_sample = sample[0], sample
        if np.linalg.norm(self.state[3:]) > self.config.max_speed_m_s:
            self.invalidate('inertial_velocity_outside_validated_limit')
            return False
        self.reason = 'coasting'
        return True

    def estimate(self, now):
        if self.state is None:
            return None
        covariance = self.covariance.copy()
        elapsed = max(0., self.stamp-self.anchor_stamp)
        # 残余偏置在窗口内相关，不能按独立白噪声被采样数稀释。
        bias = self.config.acceleration_bias_std**2
        covariance[:2, :2] += np.eye(2)*bias*elapsed**4*.25
        covariance[3:, 3:] += np.eye(2)*bias*elapsed**2
        covariance[:2, 3:] += np.eye(2)*bias*elapsed**3*.5
        covariance[3:, :2] += np.eye(2)*bias*elapsed**3*.5
        covariance[2, 2] += self.config.gyro_bias_std**2*elapsed**2
        reason = 'anchored' if elapsed <= self.config.anchor_max_gap_s else 'coasting'
        valid = True
        if not math.isfinite(now) or now < self.stamp-.05:
            valid, reason = False, 'clock_invalid'
        elif now-self.anchor_stamp > self.config.max_coast_s:
            valid, reason = False, 'coast_timeout'
        elif now-self.stamp > self.config.max_imu_gap_s:
            valid, reason = False, 'imu_stale'
        elif np.linalg.eigvalsh(covariance[:2, :2]).max() > (
                self.config.max_position_variance if
                np.linalg.eigvalsh(self.anchor_position_covariance).max() <= self.config.max_position_variance
                else np.linalg.eigvalsh(self.anchor_position_covariance).max() +
                self.config.max_position_growth_variance):
            valid, reason = False, 'position_uncertainty_limit'
        elif np.linalg.eigvalsh(covariance[:2, :2]-self.anchor_position_covariance).max() > self.config.max_position_growth_variance:
            # 保留激光自带的不确定性；单独限制本次无观测传播新增的误差。
            valid, reason = False, 'position_uncertainty_growth_limit'
        elif np.linalg.eigvalsh(covariance[3:, 3:]).max() > self.config.max_velocity_variance:
            valid, reason = False, 'velocity_uncertainty_limit'
        elif covariance[2, 2] > (self.config.max_yaw_variance if
                self.anchor_yaw_variance <= self.config.max_yaw_variance else
                self.anchor_yaw_variance+self.config.max_yaw_growth_variance):
            valid, reason = False, 'yaw_uncertainty_limit'
        elif covariance[2, 2]-self.anchor_yaw_variance > self.config.max_yaw_growth_variance:
            valid, reason = False, 'yaw_uncertainty_growth_limit'
        return InertialEstimate(self.stamp, self.state.copy(), covariance,
                                self.last_sample[2], self.last_sample[4],
                                self.anchor_stamp, valid, reason)


def gyro_at(samples, stamp, tolerance):
    """按采样时刻插值角速度；不能将最新回调角速度嫁接到旧加速度上。"""
    before = None
    for row in samples:
        if row[0] <= stamp:
            before = row
        elif before is not None:
            if stamp-before[0] > tolerance or row[0]-stamp > tolerance:
                return None
            ratio = (stamp-before[0])/(row[0]-before[0])
            return (before[1]+ratio*(row[1]-before[1]),
                    before[2]+ratio*(row[2]-before[2]))
        elif row[0]-stamp <= tolerance:
            return row[1], row[2]
        else:
            return None
    if before is not None and stamp-before[0] <= tolerance:
        return before[1], before[2]
    return None


def main(args=None):
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool, String

    class InertialNode(Node):
        def __init__(self):
            super().__init__('tracked_inertial_odometry')
            def param(name, default):
                return self.declare_parameter(name, default).value
            defaults = InertialConfig()
            self.core = InertialOdometry(InertialConfig(**{
                name: param(name, value) for name, value in vars(defaults).items()}))
            self.odom_frame = param('odom_frame', 'odom')
            self.base_frame = param('base_frame', 'base_footprint')
            self.max_measurement_age = float(param('max_measurement_age_s', .35))
            self.gyro_tolerance = float(param('gyro_sync_tolerance_s', .05))
            if (not math.isfinite(self.max_measurement_age) or
                    not .05 <= self.max_measurement_age <= .5 or
                    not .001 <= self.gyro_tolerance <= .1):
                raise ValueError('invalid inertial source timestamp limits')
            topics = {
                'gyro': param('gyro_topic', '/chassis/imu/gyro'),
                'acceleration': param('acceleration_topic', '/chassis/imu/acceleration'),
                'anchor': param('anchor_topic', '/chassis/odom/laser'),
                'partial': param('partial_topic', '/chassis/odom/laser_partial'),
                'output': param('output_topic', '/chassis/odom/inertial'),
                'valid': param('valid_topic', '/chassis/inertial/valid'),
                'state': param('state_topic', '/chassis/inertial/state'),
                'reason': param('reason_topic', '/chassis/inertial/reason'),
                'laser_ready': param('laser_ready_topic', '/chassis/odom/laser_ready'),
                'partial_ready': param('partial_ready_topic', '/chassis/odom/laser_partial_ready'),
                'imu_ready': param('imu_ready_topic', '/chassis/imu/data_ready'),
            }
            if (not all(topics.values()) or len(set(topics.values())) != len(topics) or
                    not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame):
                raise ValueError('inertial topics must be distinct and frames valid')
            if any(topics[key] in ('/odometry/filtered', '/chassis/odometry/filtered_unchecked')
                   for key in ('anchor', 'partial')):
                raise ValueError('inertial anchor must not use its own fused output feedback')
            self.gyros = deque()
            self.pending_acceleration = deque()
            self.pending_partial = deque(maxlen=20)
            self.flags = {}
            self.last_published = None
            self.last_state = None
            self.last_received = None
            status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            # 内部就绪判断只接收新心跳；对外状态仍保留最近一条供诊断读取。
            heartbeat_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE,
                                       reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(Odometry, topics['output'], 10)
            self.valid_pub = self.create_publisher(Bool, topics['valid'], status_qos)
            self.state_pub = self.create_publisher(String, topics['state'], status_qos)
            self.reason_pub = self.create_publisher(String, topics['reason'], status_qos)
            self.subs = [
                self.create_subscription(Imu, topics['gyro'], self.on_gyro, qos_profile_sensor_data),
                self.create_subscription(Imu, topics['acceleration'], self.on_acceleration, qos_profile_sensor_data),
                self.create_subscription(Odometry, topics['anchor'], self.on_anchor, 10),
                self.create_subscription(Odometry, topics['partial'], self.on_partial, 10),
            ]
            for key in ('laser_ready', 'partial_ready', 'imu_ready'):
                self.subs.append(self.create_subscription(Bool, topics[key],
                    lambda msg, name=key: self.on_flag(name, msg), heartbeat_qos))
            self.timer = self.create_timer(.05, self.publish)
            self.get_logger().info(
                f'Independent planar inertial odometry, max coast {self.core.config.max_coast_s:.2f}s; '
                'trusted laser anchors only; no TF; no absolute-position claim')

        def now_seconds(self):
            return self.get_clock().now().nanoseconds*1e-9

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec*1e-9

        def fresh(self, message):
            age = self.now_seconds()-self.stamp(message)
            return -.05 <= age <= self.max_measurement_age

        def flag(self, name):
            value = self.flags.get(name)
            return value is not None and value[0] and time.monotonic()-value[1] < .6

        def on_flag(self, name, message):
            self.flags[name] = (bool(message.data), time.monotonic())
            if name == 'imu_ready' and not message.data:
                self.core.invalidate('imu_data_not_ready_requires_new_anchor')
                self.gyros.clear()
                self.pending_acceleration.clear()
                self.pending_partial.clear()

        def on_gyro(self, message):
            if not self.flag('imu_ready'):
                return
            if message.header.frame_id != self.base_frame or not self.fresh(message):
                return
            stamp = self.stamp(message)
            rate = float(message.angular_velocity.z)
            variance = float(message.angular_velocity_covariance[8])
            if not math.isfinite(rate) or not math.isfinite(variance) or variance <= 0:
                self.core.invalidate('gyro_invalid')
                return
            if self.gyros and stamp <= self.gyros[-1][0]:
                if stamp < self.gyros[-1][0] - self.core.config.max_imu_gap_s:
                    self.core.invalidate('gyro_clock_reversed')
                    self.gyros.clear()
                    self.pending_acceleration.clear()
                return
            self.gyros.append((stamp, rate, variance))
            while len(self.gyros) > 2 and stamp-self.gyros[1][0] > 2.0:
                self.gyros.popleft()
            self.drain_acceleration()

        def on_acceleration(self, message):
            if not self.flag('imu_ready'):
                return
            if message.header.frame_id != self.base_frame or not self.fresh(message):
                return
            stamp = self.stamp(message)
            if self.pending_acceleration and stamp <= self.pending_acceleration[-1][0]:
                return
            self.pending_acceleration.append((stamp, message))
            self.drain_acceleration()

        def drain_acceleration(self):
            while self.pending_acceleration:
                stamp, message = self.pending_acceleration[0]
                if self.now_seconds()-stamp > self.max_measurement_age:
                    self.pending_acceleration.popleft()
                    self.core.invalidate('acceleration_waited_too_long_for_gyro')
                    continue
                angular = gyro_at(self.gyros, stamp, self.gyro_tolerance)
                if angular is None:
                    # 先到的加速度等待同一采样时间的 gyro，不能偷用晚到新值。
                    if self.gyros and self.gyros[-1][0] > stamp+self.gyro_tolerance:
                        self.pending_acceleration.popleft()
                        self.core.invalidate('gyro_acceleration_unsynchronized')
                        continue
                    break
                self.pending_acceleration.popleft()
                a = message.linear_acceleration
                covariance = np.array(message.linear_acceleration_covariance).reshape(3, 3)[:2, :2]
                self.core.push_imu(stamp, [a.x, a.y], angular[0], covariance, angular[1])
                detail = self.core.last_imu_rejection_detail
                if detail is not None:
                    self.get_logger().warning(
                        'IMU rejected: imu_motion_outside_validated_limits '
                        f"trigger={detail['trigger']} stamp={detail['stamp']:.9f} "
                        f"ax={detail['ax_m_s2']:.6f} ay={detail['ay_m_s2']:.6f} "
                        f"acceleration_norm={detail['acceleration_norm_m_s2']:.6f} m/s^2 "
                        f"max_acceleration={detail['max_acceleration_m_s2']:.6f} m/s^2 "
                        f"wz={detail['wz_rad_s']:.6f} rad/s "
                        f"max_yaw_rate={detail['max_yaw_rate_rad_s']:.6f} rad/s",
                        throttle_duration_sec=2.0)
                self.last_received = time.monotonic()
            self.drain_partial()
            self.publish()

        def on_partial(self, message):
            if (not self.flag('partial_ready') or not self.flag('imu_ready') or
                    self.flag('laser_ready') or not self.fresh(message)):
                return
            if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
                return
            stamp = self.stamp(message)
            if self.pending_partial and stamp <= self.pending_partial[-1][0]:
                return
            velocity = message.twist.twist.linear
            covariance = np.array(message.twist.covariance).reshape(6, 6)[:2, :2]
            self.pending_partial.append((stamp, [velocity.x, velocity.y], covariance))
            self.drain_partial()

        def drain_partial(self):
            if not self.flag('partial_ready') or self.flag('laser_ready'):
                self.pending_partial.clear()
                return
            while self.pending_partial:
                stamp, velocity, covariance = self.pending_partial[0]
                if self.now_seconds()-stamp > self.max_measurement_age:
                    self.pending_partial.popleft()
                    continue
                if self.core.stamp is None or stamp > self.core.stamp + 1e-8:
                    break
                self.pending_partial.popleft()
                self.core.add_partial_velocity(stamp, velocity, covariance)

        def on_anchor(self, message):
            if not self.flag('laser_ready') or not self.flag('imu_ready') or not self.fresh(message):
                return
            if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
                return
            p, q = message.pose.pose.position, message.pose.pose.orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.isfinite(quaternion).all() or not .99 < np.linalg.norm(quaternion) < 1.01:
                return
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            covariance = np.array(message.pose.covariance).reshape(6, 6)[np.ix_([0,1,5], [0,1,5])]
            self.core.add_anchor(self.stamp(message), [p.x, p.y, angle], covariance)
            self.publish()

        def publish(self):
            estimate = self.core.estimate(self.now_seconds())
            valid = estimate is not None and estimate.valid and self.flag('imu_ready')
            reason = self.core.reason if estimate is None else estimate.reason
            if not self.flag('imu_ready'):
                valid, reason = False, 'imu_data_not_ready_or_stale'
            if self.last_received is None or time.monotonic()-self.last_received > self.core.config.max_imu_gap_s:
                valid, reason = False, 'imu_callback_stale'
            state = ('ANCHORED' if reason == 'anchored' else 'COASTING') if valid else 'INVALID'
            self.valid_pub.publish(Bool(data=bool(valid)))
            self.state_pub.publish(String(data=state))
            self.reason_pub.publish(String(data=reason))
            if (state, reason) != self.last_state:
                self.get_logger().info(f'Inertial state: {state} ({reason})')
                self.last_state = (state, reason)
            if not valid or (self.last_published is not None and estimate.stamp <= self.last_published):
                return
            output = Odometry()
            nanos = int(round(estimate.stamp*1e9))
            output.header.stamp.sec, output.header.stamp.nanosec = divmod(nanos, 1000000000)
            output.header.frame_id, output.child_frame_id = self.odom_frame, self.base_frame
            x, y, angle, vx, vy = estimate.state
            output.pose.pose.position.x, output.pose.pose.position.y = float(x), float(y)
            output.pose.pose.orientation.z = math.sin(angle*.5)
            output.pose.pose.orientation.w = math.cos(angle*.5)
            c, s = math.cos(angle), math.sin(angle)
            inverse = np.array([[c, s], [-s, c]])
            velocity = inverse @ [vx, vy]
            output.twist.twist.linear.x, output.twist.twist.linear.y = map(float, velocity)
            output.twist.twist.angular.z = float(estimate.angular_velocity)
            pose_covariance = np.eye(6)*1e6
            pose_covariance[np.ix_([0,1,5], [0,1,5])] = estimate.covariance[:3, :3]
            velocity_jacobian = np.zeros((2, 5))
            velocity_jacobian[:, 3:] = inverse
            velocity_jacobian[:, 2] = [velocity[1], -velocity[0]]
            twist_covariance = np.eye(6)*1e6
            twist_covariance[:2, :2] = velocity_jacobian @ estimate.covariance @ velocity_jacobian.T
            twist_covariance[5, 5] = estimate.angular_variance + self.core.config.gyro_bias_std**2
            output.pose.covariance = pose_covariance.ravel().tolist()
            output.twist.covariance = twist_covariance.ravel().tolist()
            self.pub.publish(output)
            self.last_published = estimate.stamp

    rclpy.init(args=args)
    node = InertialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
