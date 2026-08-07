# kiwi_base

ROS 2 Humble / Ubuntu 22.04 下的三轮 Kiwi 全向底盘最小驱动包。

当前链路：

```text
/cmd_vel_safe (Twist) -> 三轴速度限幅 -> 物理逆运动学 -> 三轮速度 * 19 -> USB 0x3A
-> RoboMaster C 板/C620/M3508 -> USB 0x3B -> ROS 反馈话题
```

底盘节点会按 YAML 分别限制前后、左右和旋转速度；加速度斜坡、命令超时、反馈联锁、
软件急停和 Jacobian 几何检查暂不启用。USB 帧 CRC8/CRC16 仍保留；物理急停和 MCU
watchdog 由下位机负责。

## 键盘映射与默认速度

`src/kiwi_teleop_keyboard.cpp` 直接发布 REP-103 机体坐标，不做隐藏换轴：

```text
w : linear.x = +speed_m_s   前进
s : linear.x = -speed_m_s   后退
a : linear.y = +speed_m_s   左移
d : linear.y = -speed_m_s   右移
q : angular.z = +yaw_rad_s  逆时针
e : angular.z = -yaw_rad_s  顺时针
空格 / x : 三个量清零
```

每次方向键只设置一个轴，并清零另外两个轴；按下新的方向键会替换上一条速度。

默认值在 `src/kiwi_base/src/kiwi_teleop_keyboard.cpp` 构造函数：

```cpp
speed_m_s = 3.0;
yaw_rad_s = 1.5;
```

运行时覆盖默认值：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  -p speed_m_s:=0.10 -p yaw_rad_s:=0.30 -p deadman_timeout_ms:=250
```

键盘默认 `3.0 m/s` 高于底盘 YAML 默认的 `0.6 m/s`，实际送入运动学的是限幅后的
`0.6 m/s`。首次实车联调仍建议在 YAML 中进一步降低三项上限并架空车轮。

键盘终端无法提供按键释放事件，节点默认在 250 ms 没有收到新的运动按键后自动发布零速；
持续按住时依赖终端自动重复字符刷新计时。该值可按终端按键重复延迟调整。

当前输入遵循 REP-103：`w` 产生 `linear.x>0`，`a` 产生 `linear.y>0`。
如果实车方向相反，只修改运动学中的轮序/电机符号并同步测试，不在键盘节点增加第二层换轴。

## 目标速度到三个电机

输入为 ROS 机体坐标：`vx=linear.x` 向前、`vy=linear.y` 向左、
`wz=angular.z` 逆时针。轮半径 `R=0.1015 m`，中心到轮心距离 `L=0.32256 m`，
`src/kiwi_base/src/kiwi_kinematics.cpp` 使用：

```text
w1 = ( 0.8660254 * vx + 0.5 * vy - L * wz) / R
w2 = (                   -vy          - L * wz) / R
w3 = (-0.8660254 * vx + 0.5 * vy - L * wz) / R
```

`w1/w2/w3` 是轮输出轴角速度；`kiwi_base_node.cpp:update()` 再乘
`motor_reduction_ratio`（默认 `[19,19,19]`），得到发给 C 板的三个电机轴 `rad/s`。
C 板不做逆运动学，只做每个 M3508 的速度闭环。

三个单位基运动的轮输出轴速度：

```text
vx=1 m/s： [ 8.532270,  0.000000, -8.532270] rad/s
vy=1 m/s： [ 4.926108, -9.852217,  4.926108] rad/s
wz=1 rad/s：[-3.177931, -3.177931, -3.177931] rad/s
```

当前公式不使用临时放大比例；`R`、`L` 在 `config/kiwi_base.yaml` 中配置。
正解使用同一公式的解析逆变换，输出 `m/s、m/s、rad/s` 供轮里程计积分。

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
wheel_odom_topic    nav_msgs/msg/Odometry         默认 /wheel/odom_raw（发布）
imu_topic           sensor_msgs/msg/Imu           默认 /imu/data_raw（发布）
feedback_debug_topic kiwi_base/msg/FeedbackDebug 默认 /kiwi_base/feedback_debug（发布）
joint_states_topic  sensor_msgs/msg/JointState    默认 /joint_states（发布）
```

`/wheel/odom_raw` 是轮速正解和积分得到的原始里程计，只发布消息、不发布 TF。
`robot_localization` 融合死区处理后的轮速 `vx/vy`、相对 Euler yaw 与修正后的 IMU
`gyro_z`，发布 `/odometry/filtered` 和唯一的 `odom -> base_footprint`。Euler yaw 以
节点启动时刻为零点，不把下位机的任意绝对初始航向带入 `odom`。

`/kiwi_base/feedback_debug` 使用自定义消息 `kiwi_base/msg/FeedbackDebug`，字段固定为：

```text
motor_actual_rad_s[0..2]：电机 1/2/3 原始反馈
euler_rad[0..2]：roll/pitch/yaw，单位 rad
gyro_raw_rad_s[0..2]：下位机原始角速度，单位 rad/s
accel_m_s2[0..2]：下位机原始加速度，单位 m/s²
```

该话题仅用于 Foxglove 等工具的图形化调试，不被 EKF 订阅；ROS 方向修正后的角速度
仍只在 `/imu/data_raw` 中发布。

完整运行链路：

```text
C 板 0x3B
  -> kiwi_base_node
     -> /wheel/odom_raw：轮速正解得到 vx、vy、wz，并积分原始 x、y、yaw
     -> /imu/data_raw：四元数、角速度、加速度
     -> /kiwi_base/feedback_debug：0x3B 解码后的四组原始数组
     -> /joint_states：三个轮子的累计角度和角速度

/wheel/odom_raw(vx、vy) + /imu/data_raw(relative yaw、gyro_z)
  -> robot_localization EKF
  -> /odometry/filtered + odom→base_footprint
```

协方差由 `kiwi_base_node` 写入原始消息，EKF 从消息中读取；EKF 配置并不是对
`x/y` 做旋转，而是选择轮里程计的 `vx/vy`、IMU 相对 yaw 和 `gyro_z` 作为校正量，
其他状态由运动模型预测。三轴符号和静止死区均由 `kiwi_base.yaml` 配置。SLAM Toolbox
在建图模式发布 `/map` 和 `map→odom`，不发布
`odom→base_footprint`。

## TF 与后续 URDF

```text
map ──SLAM Toolbox（建图）或 AMCL（定位，二选一）──> odom
odom ──EKF────────────────────────────────────────> base_footprint
base_footprint ──robot_state_publisher 固定关节───> base_link
base_link ──固定关节──────────────────────────────> imu_link / lidar_link / motor_link_1..3
motor_link_1..3 ──连续轮关节 + /joint_states──────> wheel_link_1..3
```

- `base_footprint` 是机器人中心在地面的二维投影，不包含惯量和几何模型，供 Nav2、
  SLAM 和 EKF 使用。
- `base_link` 是实际机体参考坐标，通常位于底盘几何中心或旋转中心，承载碰撞、视觉和
  惯量模型；平地二维模型中它与 `base_footprint` 同向，只相差固定高度。
- IMU、雷达和电机外壳是固定安装，使用静态 TF；轮子会转动，必须使用连续/旋转关节，
  由 `/joint_states` 驱动动态 TF，不能把轮子本体写成静态 TF。
- 建图时由 SLAM Toolbox 发布 `map→odom`；加载地图导航时关闭 SLAM Toolbox，由 AMCL
  发布同一条 TF，二者不能同时运行。

## EKF 融合里程计

首次安装依赖：

```bash
sudo apt install ros-humble-robot-localization
```

启动底盘和 EKF（两者读取同一份 YAML）：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

该 launch 只启动 `kiwi_base_node` 和 `ekf_filter_node`，不再发布过渡期的零位姿
`base_footprint -> imu_link`。IMU、雷达和底盘机体之间的固定 TF 必须由上层 URDF/Xacro
通过 `robot_state_publisher` 统一发布，避免重复 TF。

```bash
ros2 topic hz /wheel/odom_raw /imu/data_raw /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

实时查看 EKF 输出的 Pose 和 Twist：

```bash
ros2 run kiwi_base kiwi_odom_test
```

默认订阅 `/odometry/filtered`；需要监听其他里程计话题时使用：

```bash
ros2 run kiwi_base kiwi_odom_test --ros-args -p odom_topic:=/其他里程计话题
```

YAML 中的协方差只是保守初值，不是最终标定结果。静止采样和纯 `x/y/yaw` 实测后，
应以各字段样本方差更新参数；EKF 只能降低随机噪声，不能修复错误轮序、单位和 IMU 安装方向。

## 启动和联调

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source install/setup.bash
ros2 launch kiwi_base kiwi_base.launch.py
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
  --vx 0.25 --enable --dry-run --print-hex
```

只读取下位机反馈：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py \
  --port /dev/robomaster_c --print-hex
```

YAML 中 `log_serial_frames: true` 时，节点显示目标映射、`TX[18] ...` 原始发送帧，
以及通过协议校验后的 0x3B 反馈解码结果：

```text
键盘命令 vx=0.0000 vy=-0.2500 wz=0.0000 -> 轮子转速=[-1.2315 2.4631 -1.2315] 电机转速=[-23.3990 46.7980 -23.3990]
TX[18] 33 0c 3a 65 ...
反馈电机转速=[...] 欧拉角(rpy rad)=[...] IMU原始角速度(rad/s)=[...] ROS角速度=[...]
IMU原始加速度(m/s^2)=[...]
```

同一串口只能由一个进程打开。

## 文件职责

```text
src/kiwi_teleop_keyboard.cpp  键盘 -> Twist
src/kiwi_kinematics.cpp       Twist -> w1/w2/w3 固定公式
src/measurement_processing.cpp  传感器符号修正与静止死区
src/robomaster_protocol.cpp   0x3A 组帧、0x3B 解码、CRC
src/kiwi_base_node.cpp        USB 读写、反馈转 ROS 消息、轮速积分
config/kiwi_base.yaml         唯一参数入口
launch/kiwi_base_ekf.launch.py  底盘 + EKF；传感器 TF 由上层 URDF 负责
test/test_kinematics.cpp      固定公式测试
test/test_protocol.cpp        CRC、长度、半帧和粘包测试
test/test_ekf_config.py       EKF 话题、融合维度和协方差配置测试
```
