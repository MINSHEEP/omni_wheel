# kiwi_base

本包包含 Kiwi 三轮全向的 `kiwi_base_node` 和两电机履带的 `tracked_base_node`。`kiwi_base.launch.py`、`kiwi_base_ekf.launch.py` 以及前台 `ros2 run kiwi_base kiwi_teleop_keyboard` 都从 `kiwi_platform/config/active_platform.yaml` 选择相同的底盘 YAML。

`kiwi_platform` 是配置选择器，并不取代本包的驱动；`config/kiwi_base.yaml` 和 `config/tracked_pwm.yaml` 分别是两个底盘正在使用的实际节点参数。

以下反馈轮式里程计参数说明专指 Kiwi。`kiwi_base` 是 Kiwi 三轮全向底盘的 ROS 2 Humble 驱动包。C 板通过 USB CDC 接收
速度命令并异步上报电机/IMU 遥测；本包负责正逆运动学、反馈轮式里程计、轮关节状态和键盘遥操作。

## 节点和接口

```text
kiwi_base_node
  输入：/cmd_vel
  输出：/chassis/odom/raw
        /chassis/imu/data_raw      （异步 IMU 遥测）
        /chassis/feedback/raw      （异步原始字段调试）
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
限幅，并在 0.5 秒未收到新指令时主动归零。键盘的 `speed_m_s`、`yaw_rad_s` 是手动命令
速度，不能代替底盘安全上限。

坐标约定为 `+X` 前进、`+Y` 左移、`+Z` 向上、`+wz` 逆时针。键盘 `W/S/A/D/Q/E` 分别对应
前后、左右横移和逆/顺时针旋转，空格或 `X` 停止。

## Kiwi 参数文件

```text
config/kiwi_base.yaml
```

文件内的 `kiwi_base_node` 和 `ekf_filter_node` 由公共底盘 launch 同时读取；
`kiwi_teleop_keyboard` 和 `kiwi_odom_test` 也从同一文件读取自己的参数。机器人结构尺寸
`kiwi_robot_model` 由 `kiwi_slam/urdf/kiwi.urdf.xacro` 读取。

重点参数：

```text
device                         C 板设备，默认 /dev/robomaster_c
feedback_framed                按 0x3B CRC 帧解析异步遥测
motor_feedback_lowpass_cutoff_hz
                               电机实测速度一阶低通截止频率
wheel_radius_m                 轮半径
wheel_center_distance_m        中心到轮心距离
max_wheel_speed_rad_s          轮输出轴物理上限，合成超限时等比例缩放
max_linear_speed_m_s           vx 安全上限
max_lateral_speed_m_s          vy 安全上限
max_angular_speed_rad_s        wz 安全上限
max_*_acceleration_*           Nav2 与底盘共用的三轴物理加速度上限
cmd_vel_timeout_s              速度指令看门狗超时
speed_m_s / yaw_rad_s          键盘手动速度
deadman_timeout_ms             无新按键后的自动停车时间
```

## 构建和启动

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_base
source install/setup.bash
```

仅启动底盘：

```bash
ros2 launch kiwi_base kiwi_base.launch.py
```

启动底盘和 EKF（Kiwi 可在仅构建底盘后使用；履带还需构建 `kiwi_slam`、`rf2o_laser_odometry` 和雷达驱动）：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

键盘必须在有交互输入的前台终端运行，不能作为没有 TTY 的普通后台节点启动：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

自主导航运行时不要同时启动键盘，否则两个速度源会竞争 `/cmd_vel`。

## 履带底盘

选择 `tracked_pwm` 后，同一底盘 launch 会启动 `tracked_base_node`，同一键盘命令读取 `config/tracked_pwm.yaml`。履带无编码器，不发布 `/chassis/odom/raw`；`kiwi_base_ekf.launch.py` 会启动 M10P、RF2O 和 EKF。履带不支持 `linear.y`，键盘 `A/D` 会被驱动拒绝并停车；当前 `keyboard_trial_enabled=true` 允许临时键盘开环试车，每路最多 ±5 rad/s；`hardware_calibrated=false`、`enable_motion=false` 保持正常运动和 Nav2 锁定，此时键盘的 m/s 参数只作为控制刻度。机身 0.70 × 0.59 m、履带中心距 0.50 m 已记录，其余待测项见[平台架构](../../documents/platform_architecture.md)。

## 调试

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /chassis/odom/raw
ros2 topic echo /joint_states --once
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

0x3A 命令没有 ACK，节点每周期发送最新目标且不等待回包；同一非阻塞串口上的异步 0x3B
是独立测量流。电机实测值依次经过倍率换算、一阶低通、静止死区和减速比换算，再用于
`/chassis/odom/raw` 与 `/joint_states`；IMU 字段发布到 `/chassis/imu/data_raw`。
没有有效 0x3B 时不会用 `/cmd_vel` 伪造里程计。

静止时原始和滤波速度应接近零；运动时应验证前进使 `x` 增加、左移使 `y` 增加、逆时针使
yaw 增加。该里程计无法观测打滑或堵转，只可作为导航预测；方向错误应检查电机编号和
机械安装，不要在 TF 中额外旋转 180 度。

EKF 使用电机反馈正解的 `vx/vy/wz` 与底盘绝对 `yaw/gyro_z`：yaw 负责压住航向漂移，静止时以
去线性趋势的非重叠窗口自动更新 yaw/gyro_z 方差。加速度安装轴尚未标定，因此只发布原始值、
不参与 EKF。该组合不能观测轮胎打滑造成的 x/y 位移误差；室外点云特征不足时仍需
RTK/GNSS、视觉惯导或其他绝对平移观测。

## 测试

```bash
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_base --event-handlers console_cohesion+ \
  --ctest-args --output-on-failure
colcon test-result --all --verbose
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
```
## 遥控器 USB 上报接入（2026-09-10）

`kiwi_base_node` 在同一串口解析 `0x3B` 电机/IMU 与 `0x3C` 遥控帧，
后者发布为 `kiwi_base/msg/RemoteControl`，默认话题 `/chassis/remote_control`。
当前接入仅发布状态，不生成 `/cmd_vel` 或躯干轨迹。

在 Ubuntu 22.04 / ROS 2 Humble 的工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source tools/activate_workspace.sh
colcon test --packages-select kiwi_base --event-handlers console_direct+
colcon test-result --verbose
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

另一个加载相同工作区的终端查看：

```bash
ros2 topic echo /chassis/remote_control
```

已经启动 SLAM/导航时，直接查看话题，不要再启动第二个底盘节点。
必须使用 `feedback_framed: true`；历史裸 48 字节模式不能解析混合流。
自定义底盘 YAML 还需添加 `remote_control_topic: /chassis/remote_control`
和 `remote_frame_timeout_s: 0.20`（仓库 YAML 已配置）。

消息包含四个摇杆通道、左右拨杆和 MCU 打包时间。按节点控制周期发布状态；
`header.stamp` 保留最后一帧 CRC 合格遥控数据的 ROS 接收时间，
`frame_fresh` 仅在最近 0.20 秒内收到此类帧时为 true（主机单调时钟）。
超时保留旧值用于诊断；启动无数据或串口关闭后清空值、时间戳和状态。
`controls_valid` 仅检查通道范围 [-700,700]、拨杆范围 [1,3]；
它不会因为 USB 帧超时自动变 false，因此消费者必须分别检查两个标志。
CRC 合格但数值越界的帧仍发布原值，并立即设置 `controls_valid=false`。
坏 CRC、0x3B 或其他字节不会刷新遥控帧计时器。

注意：`ros2 topic hz` 测得的是状态发布频率，不能证明 MCU 正在上报；应查看
`frame_fresh` 和接收时间。当前协议既没有滚轮 `ch[4]`，也没有接收机在线标志；
即使两个标志均为 true，也可能是 MCU 反复打包已经失效的遥控值。
后续连接躯干控制前，固件必须提供接收机有效状态/真实接收新鲜度；
ROS 消费者还必须独立检测本话题断流，处理停止保持和控制权切换。
不能将这四个通道重新映射到底盘 `/cmd_vel` 与固件本地遥控叠加。
