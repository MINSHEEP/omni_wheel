# kiwi_base

`kiwi_base` 是 ROS 2 Humble 下的三轮全向底盘硬件包，只负责速度指令、Kiwi 正逆运动学、
RoboMaster C 板 USB CDC 协议和原始底盘测量发布。建图、雷达驱动和传感器选择不进入本包。

## 1. 运行链路

```text
/cmd_vel_safe (geometry_msgs/Twist)
  -> 三轴速度限幅
  -> Kiwi 逆运动学
  -> 轮输出轴 rad/s × 19
  -> USB 0x3A（三个 float32 电机转子轴目标速度）
  -> RoboMaster C 板速度闭环

USB 0x3B（12 个 float32）
  -> CRC/长度/有限值检查
  -> 电机转速正运动学
  -> /chassis/odom/raw
  -> /chassis/imu/data_raw
  -> /chassis/feedback/raw
  -> /joint_states
```

节点不发布 `odom -> base_footprint`。该动态 TF 由 `robot_localization` EKF 独占。

## 2. 文件职责

```text
config/kiwi_base.yaml          底盘硬件、话题、协方差、死区、键盘和结构尺寸
config/ekf_chassis.yaml        C 板 IMU + 轮速的备用 EKF 配置
launch/kiwi_base.launch.py     只启动底盘节点
launch/kiwi_base_ekf.launch.py 启动底盘和 C 板 IMU EKF
src/kiwi_base_node.cpp         ROS/USB 适配和四类底盘测量发布
src/kiwi_kinematics.cpp        三行逆解和对应解析正解
src/robomaster_protocol.cpp    0x3A 编码、0x3B 解码、CRC8/CRC16
src/measurement_processing.cpp 符号修正、死区和速度限幅基础函数
msg/FeedbackDebug.msg          下位机 12 字段可视化消息
```

所有运行参数都从 YAML 读取。C++ 不保存硬件默认值，缺少参数时节点直接报错退出。

## 3. 话题和消息结构

```text
话题                         类型                              内容
/cmd_vel_safe                geometry_msgs/msg/Twist           x 前、y 左、z 逆时针目标速度
/chassis/feedback/raw        kiwi_base/msg/FeedbackDebug       C 板 0x3B 的 12 个原始标量
/chassis/imu/data_raw        sensor_msgs/msg/Imu               C 板姿态、修正角速度、原始加速度
/chassis/odom/raw            nav_msgs/msg/Odometry             轮速正解和原始积分里程计
/joint_states                sensor_msgs/msg/JointState        三轮累计角度和轮输出轴角速度
/odometry/filtered           nav_msgs/msg/Odometry             EKF 输出（本包只提供测试订阅器）
```

`FeedbackDebug` 的字段顺序与 0x3B payload 一致：

```text
字段                          单位       含义
motor_1/2/3_actual_rad_s      rad/s      三个电机转子轴实际转速
roll/pitch/yaw_rad            rad        C 板解算欧拉角，yaw 逆时针为正
gyro_x/y/z_raw_rad_s          rad/s      下位机原始角速度，尚未做 ROS 符号修正
accel_x/y/z_m_s2              m/s^2      下位机原始三轴线加速度
```

`/chassis/imu/data_raw` 使用 `sensor_msgs/msg/Imu`：

```text
header              接收时刻和 chassis_imu_link
orientation         C 板 roll/pitch/yaw 转换的四元数
angular_velocity    按 ROS 右手系修正并经过 YAML 死区的角速度
linear_acceleration C 板原始加速度
*_covariance        YAML 配置的三轴测量方差
```

`/chassis/odom/raw` 使用 `nav_msgs/msg/Odometry`：`pose` 是轮速原始积分结果，`twist`
是 EKF 实际使用的 `vx/vy/wz`。两部分都带 YAML 配置的协方差；本节点不广播 TF。

查看自定义结构：

```bash
ros2 interface show kiwi_base/msg/FeedbackDebug
```

## 4. 运动学和参数入口

坐标遵循 REP-103：`+x` 前进、`+y` 左移、`+wz` 逆时针。轮半径 `R`、轮心距 `L`：

```text
w1 = ( sin60*vx + 0.5*vy - L*wz) / R
w2 = (            -vy - L*wz) / R
w3 = (-sin60*vx + 0.5*vy - L*wz) / R
```

输出是轮输出轴 `rad/s`，发送前再乘 `motor_reduction_ratio=19` 得到电机转子轴目标值。
正解在 `KiwiKinematics::forward()` 中使用同一组公式的解析逆变换。修改轮径、轮心距、
减速比、速度限幅、静止死区和键盘速度时只改 `config/kiwi_base.yaml`。

历史 Jacobian、二次轮速限幅和重复有限值检查仍以 `//` 注释保留，注释旁写明停用原因；
它们不参与当前编译和运行链路。

## 5. 构建和启动

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

启动底盘和 C 板 IMU EKF：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

键盘控制（`W/S` 前后、`A/D` 左右、`Q/E` 旋转、空格停止）：

```bash
PARAMS=$(ros2 pkg prefix kiwi_base)/share/kiwi_base/config/kiwi_base.yaml
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args --params-file "$PARAMS"
```

松键超过 `deadman_timeout_ms` 后键盘节点自动发布零速；默认速度只改 YAML 的
`speed_m_s` 和 `yaw_rad_s`。

## 6. 调试接口

```bash
# 检查设备固定名和权限
ls -l /dev/robomaster_c /dev/ttyACM*
udevadm info -q property -n /dev/ttyACM0

# 查看四类底盘原始数据
ros2 topic hz /chassis/feedback/raw
ros2 topic echo /chassis/feedback/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic echo /chassis/odom/raw
ros2 topic echo /joint_states

# 查看 EKF 输出和 TF
ros2 run kiwi_base kiwi_odom_test --ros-args --params-file "$PARAMS"
ros2 run tf2_ros tf2_echo odom base_footprint

# 临时显示结构化 TX/RX 日志：先把 kiwi_base.yaml 的 log_serial_frames 改为 true
ros2 launch kiwi_base kiwi_base.launch.py params_file:="$PARAMS"
```

`log_serial_frames` 在节点启动时从 YAML 读取，运行中修改参数不会更新已缓存的成员值；
调试结束后改回 `false` 并重启，避免 50 Hz 日志影响实时性。

## 7. 测试

```bash
cd ~/ros2_ws
colcon test --packages-select kiwi_base --event-handlers console_direct+
colcon test-result --verbose

python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```

串口协议已经跑通，但上位机接收时刻仍代替 MCU 采样时间；高精度融合前应在协议中增加
MCU 时间戳和序号。安全联锁、急停和反馈超时保护暂以 `//` 注释停用，恢复时必须补测试。
