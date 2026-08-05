# kiwi_base 参数速查

`config/kiwi_base.yaml` 是唯一需要加载的参数文件。节点源码也有同样的默认值；YAML
中的同名参数会覆盖源码默认值。相对路径启动时从工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kiwi_base kiwi_base.launch.py \
  params_file:=src/kiwi_base/config/kiwi_base.yaml
```

## 运行链路

```text
/cmd_vel_safe (Twist)
  -> KiwiKinematics::inverse()
  -> 三个轮输出轴速度
  -> motor_reduction_ratio（默认 19）
  -> 0x33/0x3A USB 控制帧
  -> C 板 C620 + M3508
  -> 0x3B 反馈帧
  -> /wheel/odom、/joint_states、/imu/data_raw
```

## 三轮公式

当前输入使用已经点动验证的底盘控制坐标：前进为 `linear.y<0`，左移为
`linear.x<0`，`wz=angular.z` 逆时针为正。轮序固定为 1、2、3：

```text
w1 = 3 * ( 0.8660254 * vx + 0.5 * vy - wz)
w2 = 3 * (-vy                         - wz)
w3 = 3 * (-0.8660254 * vx + 0.5 * vy - wz)
```

公式位于 `src/kiwi_kinematics.cpp` 的 `KiwiKinematics::inverse()`。代码不读取轮半径、
轮心距或 `wheel_sign`；`3.0` 是当前实车联调标定比例，`19` 才是 M3508 套装减速比。

## 参数在哪里改

| 参数 | 作用 |
| --- | --- |
| `device` | USB CDC 设备路径 |
| `publish_rate` | 发送和串口读取周期，Hz |
| `motor_reduction_ratio` | 三路轮输出轴速度到电机轴速度的乘数 |
| `feedback_motor_scale_to_rad_s` | C 板反馈数值的单位倍率 |
| `cmd_vel_topic` | 底盘订阅的 `geometry_msgs/msg/Twist` 话题 |
| `wheel_odom_topic` / `imu_topic` / `joint_states_topic` | 节点发布话题 |
| `odom_frame_id` / `base_frame_id` / `imu_frame_id` | 消息 frame_id |
| `wheel_names` | 三个 `JointState.name`，顺序必须和电机 1/2/3 一致 |

速度默认值在 `src/kiwi_base/src/kiwi_teleop_keyboard.cpp`：
`speed_m_s=0.25`、`yaw_rad_s=0.8`。运行时可覆盖：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  -p speed_m_s:=0.10 -p yaw_rad_s:=0.30
```

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

- `wheel_odom_topic`：`nav_msgs/msg/Odometry`，只发布消息，不发布 `odom -> base` TF。
- `imu_topic`：`sensor_msgs/msg/Imu`，0x3B 的欧拉角转四元数，角速度和加速度原样发布。
- `joint_states_topic`：`sensor_msgs/msg/JointState`，位置为积分角度，速度为轮输出轴 rad/s。

消息类型、字段和话题默认值集中在 `src/kiwi_base/src/kiwi_base_node.cpp` 构造函数；
正常只需改 YAML 参数，不需改协议代码。

## 键盘按键

```text
w / s : linear.y=-/+speed，前进 / 后退
a / d : linear.x=-/+speed，左移 / 右移
q / e : +wz / -wz，逆时针 / 顺时针
空格或 x：清零
```

键盘直接发布这套控制坐标，因此 `w` 产生 `linear.y<0`，`a` 产生 `linear.x<0`。
`control_velocity={-t.vy,-t.vx,t.wz}` 已经代数展开到固定公式，不再在运行代码重复换轴。
