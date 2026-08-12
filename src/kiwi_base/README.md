# kiwi_base

`kiwi_base` 是 ROS 2 Humble 下的三轮全向底盘硬件包，负责 RoboMaster C 板 USB CDC
通信、Kiwi 正逆运动学、轮式里程计、C 板 IMU、关节状态和键盘控制。

## 运行链路

```text
/cmd_vel
  -> kiwi_base_node 三轴限幅
  -> Kiwi 逆运动学
  -> 电机转子目标速度
  -> USB 0x3A

USB 0x3B
  -> /chassis/odom/raw
  -> /chassis/imu/data_raw
  -> /chassis/feedback/raw
  -> /joint_states

/chassis/odom/raw + /chassis/imu/data_raw
  -> robot_localization EKF
  -> /odometry/filtered + odom -> base_footprint
```

键盘、Nav2 和测试脚本统一发布到 `/cmd_vel`。底盘节点内部负责有限值处理和最终速度
限幅，不再使用 `/cmd_vel_safe` 或额外速度中继。

## 参数

所有底盘相关参数都在 `config/kiwi_base.yaml`：

```text
kiwi_base_node        串口、运动学、限幅、反馈处理、话题和 frame
ekf_filter_node       C 板 IMU + 轮式里程计的平面 EKF
kiwi_teleop_keyboard  键盘速度和按键超时
kiwi_odom_test        终端里程计调试话题
kiwi_robot_model      URDF 读取的结构尺寸
```

键盘速度由 `speed_m_s` 和 `yaw_rad_s` 决定；底盘上限由
`max_linear_speed_m_s`、`max_lateral_speed_m_s` 和 `max_angular_speed_rad_s` 决定。

## 构建与启动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source install/setup.bash
```

只启动底盘：

```bash
ros2 launch kiwi_base kiwi_base.launch.py
```

启动底盘和 EKF：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

键盘控制：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

按键为 `W/S` 前后、`A/D` 左右、`Q/E` 旋转、空格或 `X` 停止。松键超过
`deadman_timeout_ms` 后自动发布零速。不要让键盘和 Nav2 同时控制 `/cmd_vel`。

## 调试

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /chassis/feedback/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic echo /chassis/odom/raw
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 测试

```bash
colcon test --packages-select kiwi_base --event-handlers console_direct+
colcon test-result --verbose
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```
