"""把 FAST-LIO2 的 IMU 位姿转换成底盘平面里程计和唯一公共 odom TF。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def _quaternion(values):
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("invalid quaternion")
    length = math.sqrt(sum(value * value for value in values))
    if length < 1e-6:
        raise ValueError("zero quaternion")
    return tuple(value / length for value in values)


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _rotate(q, point):
    x, y, z, w = q
    vx, vy, vz = point
    # q * (v, 0) * q^-1, expanded to avoid a dependency on a transform library.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def _rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class AdaptedOdom:
    stamp_s: float
    pose: PlanarPose
    vx: float
    vy: float
    wz: float


def base_pose_from_body(body_xyz, body_xyzw, base_to_body_xyz, base_to_body_rpy):
    """由 T_odom_body 与已标定 T_base_body 求 T_odom_base 的平面投影。"""
    if (len(body_xyz) != 3 or len(base_to_body_xyz) != 3 or
            len(base_to_body_rpy) != 3 or
            not all(math.isfinite(value) for value in (*body_xyz, *base_to_body_xyz,
                                                        *base_to_body_rpy))):
        raise ValueError("invalid pose or extrinsic")
    q_ob = _quaternion(body_xyzw)
    q_bb = _quaternion(_rpy_quaternion(*base_to_body_rpy))
    q_bb_inverse = (-q_bb[0], -q_bb[1], -q_bb[2], q_bb[3])
    q_obase = _quaternion(_multiply(q_ob, q_bb_inverse))
    lever_in_odom = _rotate(q_obase, base_to_body_xyz)
    base_xyz = tuple(body_xyz[index] - lever_in_odom[index] for index in range(3))
    yaw = math.atan2(2.0 * (q_obase[3] * q_obase[2] + q_obase[0] * q_obase[1]),
                     1.0 - 2.0 * (q_obase[1] ** 2 + q_obase[2] ** 2))
    return PlanarPose(base_xyz[0], base_xyz[1], yaw)


class LioOdomProjector:
    """验证来源与时间；只在两帧连续有效位姿后输出可融合速度。"""

    def __init__(self, source_frame, source_child_frame, base_to_body_xyz,
                 base_to_body_rpy, max_sample_gap_s=1.0):
        if not source_frame or not source_child_frame:
            raise ValueError("source frames must be nonempty")
        if not math.isfinite(max_sample_gap_s) or max_sample_gap_s <= 0.0:
            raise ValueError("max_sample_gap_s must be positive")
        # Validate the extrinsic before connecting to ROS topics.
        base_pose_from_body((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                            base_to_body_xyz, base_to_body_rpy)
        self.source_frame = source_frame
        self.source_child_frame = source_child_frame
        self.base_to_body_xyz = tuple(base_to_body_xyz)
        self.base_to_body_rpy = tuple(base_to_body_rpy)
        self.max_sample_gap_s = max_sample_gap_s
        self.previous: tuple[float, PlanarPose] | None = None

    def push(self, stamp_s, frame_id, child_frame_id, position, orientation):
        if frame_id != self.source_frame or child_frame_id != self.source_child_frame:
            raise ValueError("unexpected source frame")
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            raise ValueError("invalid source time")
        if self.previous is not None and stamp_s <= self.previous[0]:
            raise ValueError("non-increasing source time")
        pose = base_pose_from_body(position, orientation, self.base_to_body_xyz,
                                   self.base_to_body_rpy)
        previous = self.previous
        self.previous = (stamp_s, pose)
        if previous is None or stamp_s - previous[0] > self.max_sample_gap_s:
            return None
        dt = stamp_s - previous[0]
        dx, dy = pose.x - previous[1].x, pose.y - previous[1].y
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        return AdaptedOdom(stamp_s, pose, (c * dx + s * dy) / dt,
                           (-s * dx + c * dy) / dt,
                           math.remainder(pose.yaw - previous[1].yaw, 2.0 * math.pi) / dt)


class LioOdomAdapter(Node):
    def __init__(self):
        super().__init__("lio_odom_adapter")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/fastlio/odom_base")
        self.declare_parameter("source_frame", "camera_init")
        self.declare_parameter("source_child_frame", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("extrinsic_calibrated", False)
        self.declare_parameter("base_to_body_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("base_to_body_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("max_sample_gap_s", 1.0)
        self.declare_parameter("pose_variance", [0.04, 0.04, 100.0, 100.0, 100.0, 0.01])
        self.declare_parameter("twist_variance", [0.25, 0.25, 100.0, 100.0, 100.0, 0.04])

        def parameter(name):
            return self.get_parameter(name).value

        if parameter("extrinsic_calibrated") is not True:
            raise ValueError("base_to_body extrinsic must be explicitly calibrated")
        self.odom_frame = parameter("odom_frame")
        self.base_frame = parameter("base_frame")
        if not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame:
            raise ValueError("invalid output frames")
        self.projector = LioOdomProjector(
            parameter("source_frame"), parameter("source_child_frame"),
            parameter("base_to_body_xyz"), parameter("base_to_body_rpy"),
            parameter("max_sample_gap_s"))
        self.pose_covariance = self._covariance(parameter("pose_variance"))
        self.twist_covariance = self._covariance(parameter("twist_variance"))
        self.publisher = self.create_publisher(Odometry, parameter("output_topic"), 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, parameter("input_topic"), self._on_odometry,
            qos_profile_sensor_data)

    @staticmethod
    def _covariance(diagonal):
        if (len(diagonal) != 6 or not all(isinstance(value, (int, float)) and
                math.isfinite(value) and value > 0.0 for value in diagonal)):
            raise ValueError("covariance diagonal must contain six positive variances")
        covariance = [0.0] * 36
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = float(value)
        return covariance

    def _on_odometry(self, source):
        stamp = source.header.stamp
        position, orientation = source.pose.pose.position, source.pose.pose.orientation
        try:
            adapted = self.projector.push(
                stamp.sec + stamp.nanosec * 1e-9, source.header.frame_id,
                source.child_frame_id, (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w))
        except ValueError as error:
            self.get_logger().warning(f"丢弃 FAST-LIO2 里程计：{error}")
            return
        if adapted is None:
            return
        output = Odometry()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = adapted.pose.x
        output.pose.pose.position.y = adapted.pose.y
        output.pose.pose.orientation.z = math.sin(adapted.pose.yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(adapted.pose.yaw / 2.0)
        output.pose.covariance = self.pose_covariance
        output.twist.twist.linear.x = adapted.vx
        output.twist.twist.linear.y = adapted.vy
        output.twist.twist.angular.z = adapted.wz
        output.twist.covariance = self.twist_covariance
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = adapted.pose.x
        transform.transform.translation.y = adapted.pose.y
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = LioOdomAdapter()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
