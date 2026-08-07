# kiwi_base 参数速查

`config/kiwi_base.yaml` 是唯一需要加载的参数文件。节点源码也有同样的默认值；YAML
中的同名参数会覆盖源码默认值。相对路径启动时从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kiwi_base kiwi_base.launch.py
```

节点参数统一从 `params_file` 指定的 YAML 读取；launch 不再用 `device` 或日志默认值覆盖 YAML。

## 运行链路

```text
/cmd_vel_safe (Twist)
  -> vx / vy / wz 三轴限幅
  -> KiwiKinematics::inverse()
  -> 三个轮输出轴速度
  -> motor_reduction_ratio（默认 19）
  -> 0x33/0x3A USB 控制帧
  -> C 板 C620 + M3508
  -> 0x3B 反馈帧
  -> /wheel/odom_raw、/joint_states、/imu/data_raw
  -> robot_localization EKF -> /odometry/filtered + odom→base_footprint
```

## 三轮公式

输入遵循 REP-103：`linear.x>0` 前进、`linear.y>0` 左移、
`wz=angular.z>0` 逆时针。轮序固定为实车电机 1、2、3：

```text
w1 = ( 0.8660254 * vx + 0.5 * vy - L*wz) / R
w2 = (                   -vy          - L*wz) / R
w3 = (-0.8660254 * vx + 0.5 * vy - L*wz) / R
```

公式位于 `src/kiwi_kinematics.cpp` 的 `KiwiKinematics::inverse()`。
`R=wheel_radius_m`，`L=wheel_center_distance_m`；默认分别为 `0.1015 m` 和
`0.32256 m`。`19` 是 M3508 套装减速比，在运动学计算后单独应用。

## 参数在哪里改

| 参数 | 作用 |
| --- | --- |
| `device` | USB CDC 设备路径；只在 YAML 中修改 |
| `publish_rate` | 发送和串口读取周期，Hz |
| `log_serial_frames` | 是否打印 CMD/TX 帧和已校验的反馈解码日志；默认 `false` |
| `motor_reduction_ratio` | 三路轮输出轴速度到电机轴速度的乘数 |
| `feedback_motor_scale_to_rad_s` | C 板反馈数值的单位倍率 |
| `wheel_radius_m` | 轮子有效半径，默认 `0.1015 m` |
| `wheel_center_distance_m` | 底盘中心到轮心距离，默认 `0.32256 m` |
| `max_linear_speed_m_s` | `linear.x` 前后速度绝对值上限，默认 `0.6 m/s` |
| `max_lateral_speed_m_s` | `linear.y` 左右速度绝对值上限，默认 `0.6 m/s` |
| `max_angular_speed_rad_s` | `angular.z` 旋转速度绝对值上限，默认 `1.5 rad/s` |
| `wheel_pose_variance` / `wheel_twist_variance` | 原始轮里程计三个平面轴的方差初值 |
| `imu_*_variance` | IMU 姿态、角速度和加速度三个轴的方差初值 |
| `imu_angular_velocity_sign` | IMU 三轴角速度符号；当前 Z 为 `-1`，转换为 ROS 逆时针正 |
| `imu_angular_velocity_deadband_rad_s` | IMU 三轴静止死区，默认均为 `0.015 rad/s` |
| `wheel_twist_deadband` | 轮式 `vx/vy/wz` 静止死区，默认均为 `0.015` |
| `cmd_vel_topic` | 底盘订阅的 `geometry_msgs/msg/Twist` 话题，默认 `/cmd_vel_safe` |
| `wheel_odom_topic` / `imu_topic` / `feedback_debug_topic` / `joint_states_topic` | 节点发布话题 |
| `odom_frame_id` / `base_frame_id` / `imu_frame_id` | 消息 frame_id |
| `wheel_names` | 三个 `JointState.name`，顺序必须和电机 1/2/3 一致 |

速度默认值在 `src/kiwi_base/src/kiwi_teleop_keyboard.cpp`：
`speed_m_s=3.0`、`yaw_rad_s=1.5`。运行时可覆盖：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  -p speed_m_s:=0.10 -p yaw_rad_s:=0.30 -p deadman_timeout_ms:=250
```

键盘默认 `3.0 m/s` 会在底盘入口被 YAML 默认值 `0.6 m/s` 限制。首次实车联调仍应
进一步降低三项上限并架空车轮。

## USB 帧格式

发送控制帧固定 18 字节，小端序：

```text
33 | 0c | 3a | CRC8 | float32 motor1 | float32 motor2 | float32 motor3 | CRC16(2)
```

`33` 是 SOF/协议版本，`0c` 是 12 字节 payload 长度，`3a` 是控制 ID。头部 CRC8
覆盖 `33 0c 3a`，CRC16 覆盖头部、CRC8 和 payload，尾部低字节在前。组帧代码位于
`src/robomaster_protocol.cpp:encode_command()`。

C 板当前确认的组合反馈为 `0x3B`、payload 48 字节（12 个 little-endian `float32`）：

```text
motor1, motor2, motor3,
roll, pitch, yaw,
gyro_x, gyro_y, gyro_z,
accel_x, accel_y, accel_z
```

解码位于 `decode_feedback()`；`FrameParser` 负责 SOF、长度、CRC8、CRC16 和半帧/粘包。
当前只接受 `0x3B/48`，因此反馈字段和 C 板实际发送内容始终一一对应。

## ROS 输出格式

- `wheel_odom_topic`：默认 `/wheel/odom_raw`，只发布原始消息，不发布 TF。
- `imu_topic`：`sensor_msgs/msg/Imu`，0x3B 的欧拉角转四元数；角速度经过 YAML 符号和
  静止死区修正，加速度保持原始单位发布。
- `feedback_debug_topic`：`kiwi_base/msg/FeedbackDebug`，默认 `/kiwi_base/feedback_debug`；
  原样发布 0x3B 解码后的三路电机、`roll/pitch/yaw`、三轴原始角速度和三轴原始加速度，
  适合 Foxglove 绘图，不参与控制或 EKF。
- `joint_states_topic`：`sensor_msgs/msg/JointState`，位置为积分角度，速度为轮输出轴 rad/s。
- `/odometry/filtered`：EKF 融合输出，并发布唯一的 `odom -> base_footprint` TF。

EKF 使用 `/wheel/odom_raw` 的死区后 `vx/vy`，以及 `/imu/data_raw` 的相对 Euler yaw 和
修正后 `gyro_z`。不重复使用由同一轮速积分得到的位姿，Euler yaw 以启动时刻为零点。

```bash
sudo apt install ros-humble-robot-localization
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

该 launch 不再发布临时 IMU 静态变换。`base_link -> imu_link`、
`base_link -> lidar_link` 等固定关系统一写入上层 URDF/Xacro，并由
`robot_state_publisher` 发布。

消息类型、字段和话题默认值集中在 `src/kiwi_base/src/kiwi_base_node.cpp` 构造函数；
正常只需改 YAML 参数，不需改协议代码。

## 键盘按键

```text
w / s : linear.x=+/-speed，前进 / 后退
a / d : linear.y=+/-speed，左移 / 右移
q / e : +wz / -wz，逆时针 / 顺时针
空格或 x：清零
```

键盘直接发布 REP-103 坐标，因此 `w` 产生 `linear.x>0`，`a` 产生 `linear.y>0`。
底盘节点不再执行额外换轴。
终端没有按键释放事件，键盘节点在 `deadman_timeout_ms`（默认 250 ms）内没有收到新的
运动按键时自动发布零速；SSH 环境下可根据终端自动重复延迟把该值调大。
