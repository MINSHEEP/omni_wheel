# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 工作区概览

这是 Kiwi 三轮全向底盘机器人的 ROS 2 Humble 工作区。各包按依赖顺序组成完整运行链路：

```
base (遗留接口) → lslidar_msgs / lslidar_driver (镭神 M10P 雷达驱动)
                → kiwi_base (底盘驱动)
                → kiwi_slam (建图)
                → kiwi_navigation (导航)
```

`base` 包仅提供 `base/srv/Control.srv`，不参与当前运行链路。

更细的参考资料：各包自带 README、`src/kiwi_base/USB_CDC_PROTOCOL.md`（通信协议）、`documents/`（实施方案与硬件手册 PDF）。工作区文档路径统一使用 `/home/ym/ros2_ws`。

## 构建命令

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash

# 构建完整栈
colcon build --symlink-install --packages-up-to kiwi_navigation

# 只构建单个包
colcon build --symlink-install --packages-select kiwi_base

# 构建后必须 source
source install/setup.bash
```

## 测试命令

```bash
# 运行 kiwi_base 测试（含 C++ gtest 和 pytest）
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_base --event-handlers console_cohesion+ \
  --ctest-args --output-on-failure
colcon test-result --all --verbose

# 运行 kiwi_slam / kiwi_navigation 测试同理，换 --packages-select 即可

# 不启动 ROS 节点的串口硬件联调
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
```

运行单个测试：

```bash
# 单个 ctest 目标（测试名：test_kinematics / test_protocol / test_measurement_processing / test_ekf_config）
colcon test --packages-select kiwi_base --ctest-args -R test_protocol --output-on-failure

# 直接跑 gtest 二进制（最快，支持 --gtest_filter）
./build/kiwi_base/test_protocol --gtest_filter='Protocol.DetectsFeedbackCrcError'

# 单个 pytest 文件/用例。契约测试只读 src/ 下的 YAML/源码，无需构建即可运行
#（例外：test_compatibility.py 的 ROS 包可用性检查需要 ROS 环境）
python3 -m pytest src/kiwi_base/test/test_ekf_config.py -k keyboard_speeds -v
```

测试覆盖：

- `kiwi_base`：C++ gtest 覆盖 `kiwi_kinematics`、`robomaster_protocol`、`measurement_processing`；pytest 验证 EKF/底盘 YAML 接口契约。
- `kiwi_slam`：pytest 验证 M10P 参数、SLAM 参数、URDF 与 YAML 一致性、诊断接口和兼容性预检。
- `kiwi_navigation`：pytest 验证 Nav2 YAML 接口契约（`test/test_config.py`）。

## Launch 文件层级

| 用途 | 命令 |
|------|------|
| 仅底盘 | `ros2 launch kiwi_base kiwi_base.launch.py` |
| 底盘 + EKF | `ros2 launch kiwi_base kiwi_base_ekf.launch.py` |
| 底盘 + EKF + RobotModel/TF | `ros2 launch kiwi_slam kiwi_robot.launch.py` |
| 完整建图（M10P + SLAM Toolbox） | `ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true` |
| 组合建图导航（推荐建图入口） | `ros2 launch kiwi_navigation mapping_navigation.launch.py` |
| AMCL 导航（已有地图） | `ros2 launch kiwi_navigation localization_navigation.launch.py map:=/home/ym/ros2_ws/maps/warehouse.yaml` |

各 launch 提供细粒度开关：`start_base` / `start_ekf` / `start_lidar` / `start_slam` / `start_monitor` / `start_description`，便于单独排查某一环节。`rviz_software_rendering` 默认 true（规避工控机 GLSL sampler 报错）。`mapping_navigation.launch.py` 与 `localization_navigation.launch.py` 的 `use_rviz` 默认已是 true。

实车启动前可先跑预检：

```bash
ros2 run kiwi_slam kiwi_slam_compatibility --require-hardware
```

保存地图：

```bash
mkdir -p /home/ym/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /home/ym/ros2_ws/maps/warehouse
```

`maps/` 目录在工作区根目录，不属于任何 ROS 包，不应提交到版本控制。

## 架构

### 数据链路

```
C 板 (USB CDC：0x3A 命令无 ACK；0x3B 电机/IMU 异步遥测)
  ← kiwi_base_node ← /cmd_vel
  → kiwi_base_node → /chassis/odom/raw + /joint_states（按成功写入的命令开环推算）
  → kiwi_base_node → /chassis/imu/data_raw + /chassis/feedback/raw
  → ekf_filter_node → /odometry/filtered + TF: odom → base_footprint

M10P (UART /dev/wheeltec_lidar，500000 波特率由驱动按型号硬编码)
  → lslidar_driver_node → /scan
  → slam_toolbox → /map + TF: map → odom

robot_state_publisher → TF: base_footprint → base_link → lidar_link / chassis_imu_link / wheel_links
```

完整 TF 树：`map → odom → base_footprint → base_link → lidar_link / chassis_imu_link`

### kiwi_base 包内部结构

`kiwi_base_core` 静态库包含以下独立模块，其余可执行文件链接它：

- `robomaster_protocol`：0x3A 命令组帧和异步 0x3B 遥测解析；0x3B 不是命令 ACK。
- `kiwi_kinematics`：三轮全向正逆运动学及合成轮速等比例饱和。
- `measurement_processing`：IMU 符号/比例修正、陀螺零偏和低通滤波。
- `yaw_drift_estimator`：yaw 展开及只基于连续静止区间的线性漂移估计。
- `stationary_covariance_estimator`：自动标定 EKF 实际融合的 yaw/`gyro_z` 方差。

可执行文件：`kiwi_base_node`（主驱动）、`kiwi_teleop_keyboard`（前台键盘）、`kiwi_odom_test`（里程计调试打印）

`kiwi_slam` 另有两个 Python 可执行文件：`kiwi_slam_compatibility`（启动前平台/依赖/设备预检）、`kiwi_slam_monitor`（运行时诊断节点，提供 `/kiwi_slam_monitor/check` 服务）。

### 唯一参数文件

`src/kiwi_base/config/kiwi_base.yaml` 是底盘相关的**唯一配置入口**，`kiwi_base_node`、`ekf_filter_node`、`kiwi_teleop_keyboard`、`kiwi_odom_test` 全部从这一个文件读取参数。`kiwi_slam/urdf/kiwi.urdf.xacro` 也从该文件读取机器人结构尺寸。修改底盘参数只改这一个文件。

`src/kiwi_slam/config/ls_m10p_uart.yaml` 是 M10P 雷达的**唯一配置入口**，`lslidar_driver_node` 从中读取串口、帧 ID 和 `/scan` 参数。当前用 `angle_disable_min/max`（单位 0.01 度）屏蔽后方 135°~225° 扇区，雷达**不是 360° 全向可用**——评估建图与 costmap 覆盖时需考虑；该扇区基于「0° 朝前安装」假设，待实机校准。

### 坐标约定

+X 前进，+Y 左移，+Z 向上，+wz 逆时针。键盘 W/S/A/D/Q/E 分别对应前后、左右横移、逆/顺时针，空格或 X 停止。

### 重要约束

- **键盘与自主导航互斥**：`kiwi_teleop_keyboard` 必须在前台有 TTY 的终端运行，且不能与 Nav2 同时发布 `/cmd_vel`。
- **控制命令无 ACK，遥测异步**：0x3A 发送不等待应答；0x3B 只提供 IMU/调试数据，断流不得阻断控制。`/chassis/odom/raw` 和 `/joint_states` 仍按成功写入的命令推算，不能检测打滑、堵转或实际 x/y 位移误差。EKF 用绝对 yaw 与 `gyro_z` 修正航向，但没有绝对平移观测时不能消除线性里程计累计漂移。
- **速度限幅是三层链路**：`DWB (nav2.yaml) → velocity_smoother (nav2.yaml) → /cmd_vel → kiwi_base_node 最终限幅（kiwi_base.yaml 的 max_*）`。超出底盘上限的指令会被 `kiwi_base_node` **静默截断**，因此修改任何一层限速时必须同步核对另一层：改 `kiwi_base.yaml` 的 `max_*` 要检查 `src/kiwi_navigation/config/nav2.yaml` 的 DWB `max_vel_*` 与 `velocity_smoother.max_velocity` 不超过它。pytest `test_ekf_config.py::test_keyboard_speeds_do_not_exceed_base_limits` 强制键盘速度 ≤ 底盘上限（不覆盖 nav2.yaml）。
- **硬件设备**：C 板默认 `/dev/robomaster_c`（udev 符号链接），M10P 默认 `/dev/wheeltec_lidar`（udev 符号链接，规则内容见 `src/kiwi_slam/README.md` 的「雷达串口 udev 映射」），修改设备名分别改对应 YAML。注意 M10P 的 WCH 串口芯片枚举为 `ttyACM*` 而非 `ttyUSB*`。

## 调试快速参考

```bash
ros2 topic hz /scan /odometry/filtered /map
ros2 topic echo /chassis/odom/raw
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

# 仅当 monitor 已启动时可用：kiwi_slam.launch.py 默认 start_monitor:=true，
# mapping_navigation.launch.py 默认 false，需显式传 start_monitor:=true
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
```

现场故障采集（录制隐藏话题 + 节点日志 + 运行时快照，输出 `bags/incident_<stamp>.tar.gz`）：

```bash
tools/record_incident.sh --launch-cmd 'ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=false' --duration 60
```

## 提交约定

提交信息格式为 `<type>: 中文摘要`，type 取 `fix` / `feat` / `chore` / `docs`。
