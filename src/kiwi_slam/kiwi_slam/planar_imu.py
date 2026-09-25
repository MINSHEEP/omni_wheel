"""平放履带的加速度校正。gyro_z 已由驱动转换，绝不再次旋转或缩放。

标定的是固定姿态下的静止合量和传感器到车体的旋转；不是完整六面内参
标定，也不是可用于坡地的三维 INS。由现有 EKF 积分加速度，本站不发布 TF。
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} 必须包含 {size} 个有限数值")
    return result


@dataclass
class PlanarCalibration:
    rotation: np.ndarray
    rest: np.ndarray
    variance: np.ndarray

    def __post_init__(self):
        self.rotation = vector(np.asarray(self.rotation).reshape(-1), 9, "rotation").reshape(3, 3)
        self.rest = vector(self.rest, 3, "rest_acceleration")
        self.variance = vector(self.variance, 3, "acceleration_variance")
        if not np.allclose(self.rotation @ self.rotation.T, np.eye(3), atol=1e-5) or not math.isclose(
                np.linalg.det(self.rotation), 1.0, abs_tol=1e-5):
            raise ValueError("加速度旋转矩阵必须正交且行列式为 +1")
        if not 8.0 < np.linalg.norm(self.rest) < 11.0 or np.any(self.variance <= 0):
            raise ValueError("静止加速度模长或方差无效")
        gravity = self.rotation @ self.rest
        if np.linalg.norm(gravity[:2]) > 0.1 or gravity[2] < 8.0:
            raise ValueError("加速度旋转矩阵与静止重力方向不一致")

    def correct(self, raw, wz=0.0, alpha=0.0, lever=(0.08, 0.0)):
        raw = vector(raw, 3, "acceleration")
        if not all(math.isfinite(v) for v in (wz, alpha, *lever)):
            raise ValueError("角速度/角加速度/安装位置无效")
        result = self.rotation @ (raw - self.rest)
        # a_sensor = a_origin + alpha × r + omega × (omega × r)
        rx, ry = lever
        result[:2] -= [-alpha * ry - wz * wz * rx, alpha * rx - wz * wz * ry]
        return result


def fit_calibration(static_samples, motion_acceleration, reference_acceleration):
    """静止三轴样本 + 特征充分区域的直线启停样本，确定重力轴和前进轴。

reference_acceleration 为短窗口 RF2O vx 差分，仅用于方向交叉检查；
不把 RF2O 当绝对标定真值，不按它拟合加速度比例系数。
"""
    samples = np.asarray(static_samples, dtype=float)
    motion = np.asarray(motion_acceleration, dtype=float)
    reference = np.asarray(reference_acceleration, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) < 1000:
        raise ValueError("至少需要 1000 个静止加速度样本")
    if motion.shape != (len(reference), 3) or len(reference) < 12:
        raise ValueError("至少需要 12 个有效直线加减速窗口")
    if not all(np.isfinite(x).all() for x in (samples, motion, reference)):
        raise ValueError("标定样本含非有限值")
    rest = samples.mean(axis=0)
    if np.max(samples.std(axis=0)) > 0.15:
        raise ValueError("静止样本振动过大，拒绝标定")
    z_axis = rest / np.linalg.norm(rest)
    centered = reference - reference.mean()
    if np.std(reference) < 0.04:
        raise ValueError("前进方向激励不足")
    if np.count_nonzero(reference > 0.03) < 3 or np.count_nonzero(reference < -0.03) < 3:
        raise ValueError("必须同时包含足够的直线加速和减速样本")
    response = centered @ (motion - motion.mean(axis=0)) / (centered @ centered)
    horizontal = response - z_axis * np.dot(response, z_axis)
    gain = float(np.linalg.norm(horizontal))
    if not 0.5 < gain < 1.8:
        raise ValueError("加速度与直线参考响应不符，检查单位/轴向/参考里程计")
    x_axis = horizontal / gain
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.vstack((x_axis, y_axis, z_axis))
    longitudinal = (motion - rest) @ x_axis
    correlation = float(np.corrcoef(longitudinal, reference)[0, 1])
    if not math.isfinite(correlation) or correlation < 0.7:
        raise ValueError("前进方向相关性不足，不能由静止重力猜测水平轴向")
    variance = np.maximum((samples @ rotation.T).var(axis=0), 0.04)
    calibration = PlanarCalibration(rotation, rest, variance)
    return calibration, {"static_samples": len(samples), "motion_windows": len(reference),
                         "forward_correlation": correlation, "reference_response_gain": gain,
                         "static_std_raw": samples.std(axis=0).tolist(),
                         "scope": "fixed-level planar acceleration; no scale fit; laser reference is not ground truth"}


class AccelerationProcessor:
    """合并 USB 成批样本；用约 0.1 s 角速度窗口估计角加速度，避免微秒 dt 爆值。"""
    def __init__(self, calibration, lever=(0.08, 0.0), period=0.02, max_gap=0.2):
        self.calibration = calibration
        self.lever = vector(lever, 2, "imu_lever_xy")
        self.period, self.max_gap = period, max_gap
        if not 0 < period < max_gap:
            raise ValueError("输出周期必须大于零且小于最大数据间隔")
        self.pending = []
        self.rates = deque()
        self.last = None

    def reset(self):
        self.pending.clear()
        self.rates.clear()
        self.last = None

    def push(self, timestamp, raw, wz):
        if not math.isfinite(timestamp) or not math.isfinite(wz):
            self.reset()
            raise ValueError("IMU 时间/角速度无效")
        try:
            raw = vector(raw, 3, "acceleration")
        except ValueError:
            self.reset()
            raise
        if self.last is not None and (timestamp <= self.last or timestamp - self.last > self.max_gap):
            self.reset()
        self.last = timestamp
        self.pending.append((timestamp, raw, wz))
        if timestamp - self.pending[0][0] < self.period:
            return None
        stamp = self.pending[-1][0]
        acceleration = np.mean([row[1] for row in self.pending], axis=0)
        rate = float(np.mean([row[2] for row in self.pending]))
        self.pending.clear()
        self.rates.append((stamp, rate))
        while len(self.rates) > 2 and stamp - self.rates[1][0] >= 0.1:
            self.rates.popleft()
        span = stamp - self.rates[0][0]
        # 开头或断流后先收集转向历史；不凭零角加速度假设发布加速度。
        if span < 0.08:
            return None
        alpha = (rate - self.rates[0][1]) / span
        corrected = self.calibration.correct(acceleration, rate, alpha, self.lever)
        return stamp, corrected


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class PlanarImuNode(Node):
        def __init__(self):
            super().__init__("tracked_imu_acceleration")
            def param(name, default):
                return self.declare_parameter(name, default).value
            calibration = PlanarCalibration(
                param("accel_rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                param("rest_acceleration", [0.0, 0.0, 9.80665]),
                param("acceleration_variance", [0.04, 0.04, 0.04]))
            self.enabled = param("enabled", False)
            self.source_frame = param("source_frame", "chassis_imu_link")
            self.output_frame = param("output_frame", "base_footprint")
            source = param("source_topic", "/chassis/imu/data_raw")
            output = param("output_topic", "/chassis/imu/acceleration")
            if source == output or not all((source, output, self.source_frame, self.output_frame)):
                raise ValueError("IMU 话题/坐标配置无效")
            self.processor = AccelerationProcessor(calibration, param("imu_lever_xy", [0.08, 0.0]))
            self.pub = self.create_publisher(Imu, output, qos_profile_sensor_data)
            self.sub = self.create_subscription(Imu, source, self.callback, qos_profile_sensor_data)
            self.get_logger().info(f"Planar acceleration enabled={self.enabled}; gyro_z unchanged; {source} -> {output}")

        def callback(self, message):
            if not self.enabled:
                return
            if message.header.frame_id != self.source_frame:
                self.processor.reset()
                self.get_logger().error("IMU source frame mismatch", throttle_duration_sec=5.0)
                return
            t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            a = message.linear_acceleration
            try:
                result = self.processor.push(t, [a.x, a.y, a.z], message.angular_velocity.z)
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5.0)
                return
            if result is None:
                return
            stamp, acceleration = result
            output = Imu()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.output_frame
            output.orientation_covariance[0] = -1.0
            output.angular_velocity_covariance[0] = -1.0
            output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = map(float, acceleration)
            for index, variance in zip((0, 4, 8), self.processor.calibration.variance):
                output.linear_acceleration_covariance[index] = float(variance)
            self.pub.publish(output)

    rclpy.init(args=args)
    node = PlanarImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
