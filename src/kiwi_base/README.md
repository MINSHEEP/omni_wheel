# kiwi_base

`kiwi_base` 是 Kiwi 三轮全向底盘的 ROS 2 Humble 驱动包。它负责 C 板 USB CDC 通信、
Kiwi 正逆运动学、轮式原始里程计、C 板 IMU、轮关节状态和前台键盘遥操作。

## 节点和接口

```text
kiwi_base_node
  输入：/cmd_vel
  输出：/chassis/odom/raw
        /chassis/imu/data_raw
        /chassis/feedback/raw
        /joint_states

ekf_filter_node
  输入：/chassis/odom/raw、/chassis/imu/data_raw
  输出：/odometry/filtered、odom -> base_footprint

kiwi_teleop_keyboard
  输出：/cmd_vel

kiwi_odom_test
  只读打印：/odometry/filtered
```

`/cmd_vel` 是唯一底盘速度入口。底盘节点对 `linear.x`、`linear.y` 和 `angular.z` 做最终
限幅。键盘的 `speed_m_s`、`yaw_rad_s` 是手动命令速度，不能代替底盘安全上限。

坐标约定为 `+X` 前进、`+Y` 左移、`+Z` 向上、`+wz` 逆时针。键盘 `W/S/A/D/Q/E` 分别对应
前后、左右横移和逆/顺时针旋转，空格或 `X` 停止。

## 唯一参数文件

```text
config/kiwi_base.yaml
```

文件内的 `kiwi_base_node` 和 `ekf_filter_node` 由公共底盘 launch 同时读取；
`kiwi_teleop_keyboard` 和 `kiwi_odom_test` 也从同一文件读取自己的参数。机器人结构尺寸
`kiwi_robot_model` 由 `kiwi_slam/urdf/kiwi.urdf.xacro` 读取。

重点参数：

```text
device                         C 板设备，默认 /dev/robomaster_c
wheel_radius_m                 轮半径
wheel_center_distance_m        中心到轮心距离
max_linear_speed_m_s           vx 安全上限
max_lateral_speed_m_s          vy 安全上限
max_angular_speed_rad_s        wz 安全上限
speed_m_s / yaw_rad_s          键盘手动速度
deadman_timeout_ms             无新按键后的自动停车时间
```

## 构建和启动

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source install/setup.bash
```

仅启动底盘：

```bash
ros2 launch kiwi_base kiwi_base.launch.py
```

启动底盘和 EKF：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

键盘必须在有交互输入的前台终端运行，不能作为没有 TTY 的普通后台节点启动：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

自主导航运行时不要同时启动键盘，否则两个速度源会竞争 `/cmd_vel`。

## 调试

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /chassis/feedback/raw
ros2 topic echo /chassis/odom/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

静止时原始和滤波速度应接近零；运动时应验证前进使 `x` 增加、左移使 `y` 增加、逆时针使
yaw 增加。方向错误应检查电机编号和反馈符号，不要在 TF 中额外旋转 180 度。

## 测试

```bash
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_base --event-handlers console_cohesion+ \
  --ctest-args --output-on-failure
colcon test-result --all --verbose
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```
