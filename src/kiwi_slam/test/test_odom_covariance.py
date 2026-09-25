"""确认适配器保留 RF2O 运动结果，并给 EKF 使用的轴设置正方差。"""

from nav_msgs.msg import Odometry

from kiwi_slam.odom_covariance import set_covariance


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
