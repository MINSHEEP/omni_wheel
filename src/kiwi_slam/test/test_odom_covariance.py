"""确认适配器保留真实运动，同时压住静止 RF2O 伪运动。"""

import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from kiwi_slam.odom_covariance import (
    StationaryFilterConfig,
    StationaryOdomFilter,
    set_covariance,
)


def stationary_filter():
    return StationaryOdomFilter(StationaryFilterConfig(
        window_s=2.0,
        max_scan_median_delta_m=0.011,
        min_scan_overlap_ratio=0.75,
        min_valid_scan_ratio=0.2,
        max_translation_m=0.02,
        max_yaw_rad=0.01,
        max_gyro_rms_rad_s=0.012,
        max_sensor_age_s=0.25,
    ))


def sample(filter_, time_s, raw_x, raw_y, raw_yaw, scan_shift=0.0, gyro=0.003,
           scan_partial_shift=0.0):
    scan = LaserScan()
    scan.range_min = 0.2
    scan.range_max = 12.0
    scan.angle_min = -1.0
    scan.angle_increment = 0.01
    scan.ranges = [4.0 + scan_shift + (index % 5) * 0.01
                   + (scan_partial_shift if index < 40 else 0.0)
                   for index in range(200)]
    imu = Imu()
    imu.angular_velocity.z = gyro
    message = Odometry()
    message.pose.pose.position.x = raw_x
    message.pose.pose.position.y = raw_y
    message.pose.pose.orientation.z = math.sin(raw_yaw / 2)
    message.pose.pose.orientation.w = math.cos(raw_yaw / 2)
    message.twist.twist.linear.x = 0.01
    filter_.observe_scan(scan, time_s)
    filter_.observe_imu(imu, time_s)
    return filter_.correct(message, time_s)


def test_rf2o_covariance_adapter_keeps_pose_twist_and_timestamp():
    message = Odometry()
    message.header.stamp.sec = 123
    message.header.frame_id = "odom"
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = 2.5
    message.twist.twist.linear.x = -0.3
    adapted = set_covariance(
        message,
        pose_xy_variance=0.25,
        pose_yaw_variance=0.01,
        twist_vx_variance=0.04,
        twist_vy_variance=0.01,
        twist_wz_variance=0.04,
    )
    assert adapted.header.stamp.sec == 123
    assert adapted.header.frame_id == "odom"
    assert adapted.child_frame_id == "base_footprint"
    assert adapted.pose.pose.position.x == 2.5
    assert adapted.twist.twist.linear.x == -0.3
    assert [adapted.pose.covariance[i] for i in (0, 7, 35)] == [0.25, 0.25, 0.01]
    assert [adapted.twist.covariance[i] for i in (0, 7, 35)] == [0.04, 0.01, 0.04]
    assert all(adapted.pose.covariance[i] == 1e6 for i in (14, 21, 28))


def test_stationary_rf2o_drift_is_held_and_motion_resumes_without_pose_jump():
    filter_ = stationary_filter()
    stationary = []
    for index in range(1201):
        t = index * 0.1
        raw_x = 0.001 * t
        raw_y = -0.0005 * t
        raw_yaw = 0.0004 * t
        output, held = sample(filter_, t, raw_x, raw_y, raw_yaw)
        stationary.append((output.pose.pose.position.x, output.pose.pose.position.y, held))
    assert stationary[-1][2]
    assert abs(stationary[-1][0]) < 0.01
    assert abs(stationary[-1][1]) < 0.01

    previous = stationary[-1][0]
    released = False
    for step in range(1, 101):
        t = 120.0 + step * 0.1
        output, held = sample(filter_, t, 0.12 + step * 0.01, -0.06,
                              0.048, scan_shift=step * 0.01)
        if not held:
            released = True
        assert abs(output.pose.pose.position.x - previous) < 0.04
        previous = output.pose.pose.position.x
    assert released
    assert output.pose.pose.position.x > 0.9


def test_imu_rotation_vetoes_stationary_hold():
    filter_ = stationary_filter()
    for index in range(31):
        output, held = sample(filter_, index * 0.1, 0.0, 0.0, 0.0)
    assert held
    for index in range(31, 61):
        output, held = sample(filter_, index * 0.1, 0.0, 0.0, 0.0,
                              gyro=0.04)
    assert not held


def test_stationary_rf2o_jump_and_partial_scan_noise_do_not_release_hold():
    filter_ = stationary_filter()
    for index in range(31):
        output, held = sample(filter_, index * 0.1, 0.0, 0.0, 0.0)
    assert held
    for index in range(31, 61):
        output, held = sample(filter_, index * 0.1, 0.25, -0.12, 0.03)
        assert held
        assert abs(output.pose.pose.position.x) < 0.001
        assert abs(output.pose.pose.position.y) < 0.001
    for index in range(61, 91):
        output, held = sample(filter_, index * 0.1, 0.25, -0.12, 0.03,
                              scan_partial_shift=0.1)
        assert held
