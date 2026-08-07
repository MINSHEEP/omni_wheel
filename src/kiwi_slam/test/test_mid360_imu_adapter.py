"""Mid-360S IMU 的 EKF 适配单元测试。"""

import pytest
from sensor_msgs.msg import Imu

from kiwi_slam.mid360_imu_adapter import adapt_imu


def test_adapt_imu_keeps_raw_measurements_and_adds_covariance():
    raw = Imu()
    raw.header.frame_id = "livox_frame"
    raw.angular_velocity.x = 0.01
    raw.angular_velocity.y = -0.03
    raw.angular_velocity.z = 0.20
    raw.linear_acceleration.x = 1.0
    raw.linear_acceleration.y = 2.0
    raw.linear_acceleration.z = 3.0

    result = adapt_imu(
        raw,
        (0.1, 0.2, 0.3),
        (1.1, 1.2, 1.3),
        (0.02, 0.02, 0.02),
    )

    assert result.header.frame_id == "livox_frame"
    assert result.orientation.w == 1.0
    assert result.orientation_covariance[0] == -1.0
    assert result.angular_velocity.x == 0.0
    assert result.angular_velocity.y == pytest.approx(-0.03)
    assert result.angular_velocity.z == pytest.approx(0.20)
    assert list(result.angular_velocity_covariance)[::4] == pytest.approx([0.1, 0.2, 0.3])
    assert result.linear_acceleration.x == 1.0
    assert result.linear_acceleration.y == 2.0
    assert result.linear_acceleration.z == 3.0
    assert list(result.linear_acceleration_covariance)[::4] == pytest.approx([1.1, 1.2, 1.3])
