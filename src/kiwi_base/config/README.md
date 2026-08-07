# kiwi_base 参数说明

本目录只有两个运行参数入口，职责不能混用：

```text
kiwi_base.yaml    底盘硬件、ROS 话题、测量噪声、键盘和模型结构
ekf_chassis.yaml  轮速 + C 板 IMU 的 robot_localization 传感器选择
```

`kiwi_base.yaml` 的关键参数：

```text
device                              USB CDC 固定设备名
publish_rate                        USB 收发周期 Hz
feedback_motor_scale_to_rad_s       反馈值转换为电机轴 rad/s 的倍率
motor_reduction_ratio               三个电机的减速比
wheel_radius_m                      轮子有效半径
wheel_center_distance_m             底盘中心到轮心距离
max_linear_speed_m_s                前后速度绝对值上限
max_lateral_speed_m_s               左右速度绝对值上限
max_angular_speed_rad_s              旋转速度绝对值上限
wheel_twist_deadband                vx、vy、wz 静止死区
imu_angular_velocity_sign           C 板角速度到 ROS 右手系的符号
imu_angular_velocity_deadband_rad_s C 板三轴角速度静止死区
wheel_*_variance                    轮式里程计协方差对角线
imu_*_variance                      C 板 IMU 协方差对角线
```

固定话题：

```text
cmd_vel_topic        /cmd_vel_safe
wheel_odom_topic     /chassis/odom/raw
imu_topic            /chassis/imu/data_raw
feedback_debug_topic /chassis/feedback/raw
joint_states_topic   /joint_states
```

`ekf_chassis.yaml` 只供 Pace 备用路线和独立底盘测试使用：融合轮式 `vx/vy/wz`、C 板
`yaw` 和 `gyro_z`。Mid-360S 主路线使用 `kiwi_slam/config/ekf_mid360.yaml`；切换路线时
由 launch 选择 EKF YAML，不修改底盘硬件参数。

传感器相对底盘的 `xyz/rpy` 不是节点参数，统一维护在
`kiwi_slam/urdf/kiwi.urdf.xacro`。Livox 点云到内置 IMU 的厂内标定属于 FAST-LIO2
算法内参，单独维护在 `kiwi_slam/config/fast_lio_mid360s.yaml`。
