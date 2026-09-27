"""FAST-LIO2 位姿适配的外参、速度与输入护栏。"""

import math
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

from kiwi_slam.lio_odom_adapter import (
    LioOdomAdapter, LioOdomProjector, base_pose_from_body,
)


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_full_extrinsic_is_inverted_before_planar_projection():
    # 机体向左转 90 度后，机体 x 正向一米的 IMU 位于 odom 的 y=1。
    pose = base_pose_from_body((0.0, 1.0, 0.4), yaw_quaternion(math.pi / 2),
                               (1.0, 0.0, 0.4), (0.0, 0.0, 0.0))
    assert pose.x == pytest.approx(0.0, abs=1e-12)
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    assert pose.yaw == pytest.approx(math.pi / 2)

    pose = base_pose_from_body((2.0, 0.0, 0.0), yaw_quaternion(math.pi / 2),
                               (0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))
    assert pose.yaw == pytest.approx(0.0, abs=1e-12)

    # 带俯仰角的安装也先用完整三维外参还原，之后才投影到水平面。
    pitch = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    pose = base_pose_from_body((1.0, 0.0, 0.0), pitch,
                               (1.0, 0.0, 0.0), (0.0, math.pi / 2, 0.0))
    assert pose.x == pytest.approx(0.0, abs=1e-12)
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    assert pose.yaw == pytest.approx(0.0, abs=1e-12)


def test_twist_is_derived_in_base_frame_not_copied_from_fastlio():
    projector = LioOdomProjector("camera_init", "body", (1.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0))
    assert projector.push(1.0, "camera_init", "body", (0.0, 1.0, 0.0),
                          yaw_quaternion(math.pi / 2)) is None
    result = projector.push(1.5, "camera_init", "body", (1.0, 1.0, 0.0),
                            yaw_quaternion(math.pi / 2))
    assert result.pose.x == pytest.approx(1.0)
    assert result.pose.y == pytest.approx(0.0)
    assert result.vx == pytest.approx(0.0, abs=1e-12)
    assert result.vy == pytest.approx(-2.0)
    assert result.wz == pytest.approx(0.0)


def test_yaw_rate_wraps_across_pi():
    projector = LioOdomProjector("camera_init", "body", (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0))
    projector.push(1.0, "camera_init", "body", (0.0, 0.0, 0.0),
                   yaw_quaternion(math.pi - 0.05))
    result = projector.push(1.1, "camera_init", "body", (0.0, 0.0, 0.0),
                            yaw_quaternion(-math.pi + 0.05))
    assert result.wz == pytest.approx(1.0)


def test_wrong_frame_or_nonincreasing_time_does_not_change_anchor():
    projector = LioOdomProjector("camera_init", "body", (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0))
    projector.push(1.0, "camera_init", "body", (0.0, 0.0, 0.0),
                   yaw_quaternion(0.0))
    with pytest.raises(ValueError, match="frame"):
        projector.push(1.1, "map", "body", (100.0, 0.0, 0.0), yaw_quaternion(0.0))
    with pytest.raises(ValueError, match="time"):
        projector.push(1.0, "camera_init", "body", (100.0, 0.0, 0.0),
                       yaw_quaternion(0.0))
    with pytest.raises(ValueError, match="quaternion"):
        projector.push(1.1, "camera_init", "body", (100.0, 0.0, 0.0),
                       (0.0, 0.0, 0.0, 0.0))
    result = projector.push(1.2, "camera_init", "body", (0.2, 0.0, 0.0),
                            yaw_quaternion(0.0))
    assert result.vx == pytest.approx(1.0)


def test_gap_reanchors_without_emitting_false_velocity():
    projector = LioOdomProjector("camera_init", "body", (0.0, 0.0, 0.0),
                                 (0.0, 0.0, 0.0), max_sample_gap_s=0.5)
    projector.push(1.0, "camera_init", "body", (0.0, 0.0, 0.0),
                   yaw_quaternion(0.0))
    assert projector.push(2.0, "camera_init", "body", (10.0, 0.0, 0.0),
                          yaw_quaternion(0.0)) is None
    result = projector.push(2.1, "camera_init", "body", (10.1, 0.0, 0.0),
                            yaw_quaternion(0.0))
    assert result.vx == pytest.approx(1.0)


def test_covariance_is_conservative_and_never_all_zero():
    covariance = LioOdomAdapter._covariance([0.04, 0.04, 100.0, 100.0, 100.0, 0.01])
    assert len(covariance) == 36
    assert all(covariance[index * 6 + index] > 0.0 for index in range(6))
    with pytest.raises(ValueError, match="covariance"):
        LioOdomAdapter._covariance([0.0] * 6)


def test_public_tf_matches_accepted_odom_and_rejects_bad_source():
    class Collector:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

        def sendTransform(self, message):
            self.messages.append(message)

    odom_output, tf_output = Collector(), Collector()
    rejected = []
    adapter = SimpleNamespace(
        projector=LioOdomProjector("camera_init", "body", (1.0, 0.0, 0.0),
                                    (0.0, 0.0, 0.0)),
        odom_frame="odom", base_frame="base_footprint",
        pose_covariance=LioOdomAdapter._covariance([0.04] * 6),
        twist_covariance=LioOdomAdapter._covariance([0.25] * 6),
        publisher=odom_output, tf_broadcaster=tf_output,
        get_logger=lambda: SimpleNamespace(warning=rejected.append),
    )

    def sample(nanosec, x, frame="camera_init"):
        message = Odometry()
        message.header.stamp.sec = 10
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = frame
        message.child_frame_id = "body"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        return message

    # First sample only anchors the derivative, so neither output may be emitted.
    LioOdomAdapter._on_odometry(adapter, sample(0, 1.0))
    assert not odom_output.messages and not tf_output.messages
    LioOdomAdapter._on_odometry(adapter, sample(100_000_000, 1.1))
    assert len(odom_output.messages) == len(tf_output.messages) == 1
    odom, transform = odom_output.messages[0], tf_output.messages[0]
    assert odom.header == transform.header
    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == transform.child_frame_id == "base_footprint"
    assert odom.pose.pose.position.x == pytest.approx(transform.transform.translation.x)
    assert odom.pose.pose.position.y == pytest.approx(transform.transform.translation.y)
    assert odom.pose.pose.orientation == transform.transform.rotation
    assert odom.twist.twist.linear.x == pytest.approx(1.0)

    LioOdomAdapter._on_odometry(adapter, sample(200_000_000, 1.2, "map"))
    LioOdomAdapter._on_odometry(adapter, sample(100_000_000, 1.2))
    assert len(rejected) == 2
    assert len(odom_output.messages) == len(tf_output.messages) == 1
