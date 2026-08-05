# kiwi_base

ROS 2 Humble / Ubuntu 22.04 下的三轮 Kiwi 全向底盘最小驱动包。

当前链路：

```text
/cmd_vel (Twist) -> 固定逆运动学 -> 三轮速度 * 19 -> USB 0x3A
-> RoboMaster C 板/C620/M3508 -> USB 0x3B -> ROS 反馈话题
```

本阶段暂不启用速度限幅、加速度斜坡、命令超时、反馈联锁、软件急停和 Jacobian
几何检查。USB 帧 CRC8/CRC16 仍保留，用于识别完整帧；物理急停和 MCU watchdog
由下位机负责。

## 键盘映射与默认速度

`src/kiwi_teleop_keyboard.cpp` 直接发布 ROS 机体坐标，不做隐藏换轴：

```text
w : linear.y = -speed_m_s   前进
s : linear.y = +speed_m_s   后退
a : linear.x = -speed_m_s   左移
d : linear.x = +speed_m_s   右移
q : angular.z = +yaw_rad_s  逆时针
e : angular.z = -yaw_rad_s  顺时针
空格 / x : 三个量清零
```

每次方向键只设置一个轴，并清零另外两个轴；按下新的方向键会替换上一条速度。

默认值在 `src/kiwi_base/src/kiwi_teleop_keyboard.cpp` 构造函数：

```cpp
speed_m_s = 0.25;
yaw_rad_s = 0.8;
```

运行时覆盖默认值：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  -p speed_m_s:=0.10 -p yaw_rad_s:=0.30
```

当前输入是项目已经点动验证的控制坐标，不是标准 REP-103 的直觉轴：`w` 产生
`linear.y<0`，`a` 产生 `linear.x<0`。如果实车方向相反，只修改固定公式中的轮序/符号
并同步测试，不在键盘节点增加第二层换轴。

## 目标速度到三个电机

输入为当前底盘控制坐标：`vx=linear.x`、`vy=linear.y`、`wz=angular.z`。前进是
`vy<0`，左移是 `vx<0`，逆时针 `wz>0`。`src/kiwi_base/src/kiwi_kinematics.cpp` 直接使用：

```text
w1 = 3 * ( 0.8660254 * vx + 0.5 * vy - wz)
w2 = 3 * (-vy                         - wz)
w3 = 3 * (-0.8660254 * vx + 0.5 * vy - wz)
```

`w1/w2/w3` 是轮输出轴联调速度；`kiwi_base_node.cpp:update()` 再乘
`motor_reduction_ratio`（默认 `[19,19,19]`），得到发给 C 板的三个电机轴 `rad/s`。
C 板不做逆运动学，只做每个 M3508 的速度闭环。

默认键盘速度的对应轮速：

```text
w（前进，0.25 m/s）：[-0.375000,  0.750000, -0.375000]
a（左移，0.25 m/s）：[-0.649519,  0.000000,  0.649519]
q（旋转，0.8 rad/s）：[-2.400000, -2.400000, -2.400000]
```

比例 3.0 是联调标定值，不是轮半径；轮直径和 322.56 mm 轮心距暂不参与本版公式。

## USB 帧格式

组帧代码：`src/kiwi_base/src/robomaster_protocol.cpp:encode_command()`。

发送控制帧固定 18 字节，小端序：

```text
33 | 0c | 3a | CRC8 | motor1(float32) | motor2(float32) | motor3(float32) | CRC16(低,高)
```

- `0x33`：SOF/协议版本；`0x0c`：12 字节 payload；`0x3a`：电机速度命令 ID。
- CRC8 覆盖前 3 字节；CRC16 覆盖 `header + CRC8 + payload`，低字节先传。
- 线上没有 `enable` 字段；停止时发送三个 `0.0f`。

C 板组合反馈为 `0x3b`，payload 是 12 个 little-endian `float32`：

```text
motor1_actual, motor2_actual, motor3_actual,
roll, pitch, yaw,
gyro_x, gyro_y, gyro_z,
accel_x, accel_y, accel_z
```

解码入口：`FrameParser::feed()` -> `KiwiBaseNode::on_feedback()` ->
`decode_feedback()`。当前不再兼容其它反馈 ID，避免把旧数据结构误当作实机数据。

## ROS 话题和消息格式

话题名和 frame id 都在 `config/kiwi_base.yaml` 中修改：

```text
cmd_vel_topic       geometry_msgs/msg/Twist        默认 /cmd_vel_safe（订阅）
wheel_odom_topic    nav_msgs/msg/Odometry         默认 /wheel/odom（发布）
imu_topic           sensor_msgs/msg/Imu           默认 /imu/data_raw（发布）
joint_states_topic  sensor_msgs/msg/JointState    默认 /joint_states（发布）
```

`/wheel/odom` 只发布消息，不发布 TF；后续由 `robot_localization` EKF 发布
`odom -> base_footprint`，SLAM Toolbox 或 AMCL 负责上层地图定位。Nav2 的标准
`Twist` 接入前，需要转换为本项目的 `linear.x=-nav_vy`、`linear.y=-nav_vx`。

## 启动和联调

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source install/setup.bash
ros2 launch kiwi_base kiwi_base.launch.py \
  params_file:=src/kiwi_base/config/kiwi_base.yaml \
  device:=/dev/robomaster_c \
  log_serial_frames:=true
```

另一个终端启动键盘：

```bash
source /opt/ros/humble/setup.bash
source /home/ym/ros2_ws/install/setup.bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

无硬件验证：

```bash
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
python3 src/kiwi_base/tools/kiwi_serial_test.py \
  --vy -0.25 --enable --dry-run --print-hex
```

只读取下位机反馈：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py \
  --port /dev/robomaster_c --print-hex
```

启动 `log_serial_frames:=true` 后，节点逐帧显示目标映射、`TX[18] ...` 和 `RX[54] ...`：

```text
键盘命令 vx=0.0000 vy=-0.2500 wz=0.0000 -> 轮子转速=[-0.3750 0.7500 -0.3750] 电机转速=[-7.1250 14.2500 -7.1250]
TX[18] 33 0c 3a 65 ...
RX[54] 33 30 3b 5b ...
```

同一串口只能由一个进程打开。

## 文件职责

```text
src/kiwi_teleop_keyboard.cpp  键盘 -> Twist
src/kiwi_kinematics.cpp       Twist -> w1/w2/w3 固定公式
src/robomaster_protocol.cpp   0x3A 组帧、0x3B 解码、CRC
src/kiwi_base_node.cpp        USB 读写、反馈转 ROS 消息、轮速积分
config/kiwi_base.yaml         唯一参数入口
test/test_kinematics.cpp      固定公式测试
test/test_protocol.cpp        CRC、长度、半帧和粘包测试
```
