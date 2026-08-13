# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 工作区概览

这是 Kiwi 三轮全向底盘机器人的 ROS 2 Humble 工作区。五个包按依赖顺序组成完整运行链路：

```
base (遗留接口) → bluesea-ros2 (E110 雷达驱动) → kiwi_base (底盘驱动)
                                                  → kiwi_slam (建图)
                                                  → kiwi_navigation (导航)
```

`base` 包仅提供 `base/srv/Control.srv`，不参与当前运行链路。

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

# 运行 kiwi_slam 测试
colcon test --packages-select kiwi_slam --event-handlers console_cohesion+ \
  --ctest-args --output-on-failure

# 不启动 ROS 节点的串口硬件联调
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```

`kiwi_base` 测试：C++ gtest 覆盖 `kiwi_kinematics`、`robomaster_protocol`、
`measurement_processing`；pytest 验证 EKF/底盘 YAML 接口契约。

`kiwi_slam` 测试：pytest 验证 E110 参数、SLAM 参数、URDF 与 YAML 一致性和诊断接口。

## Launch 文件层级

| 用途 | 命令 |
|------|------|
| 仅底盘 | `ros2 launch kiwi_base kiwi_base.launch.py` |
| 底盘 + EKF | `ros2 launch kiwi_base kiwi_base_ekf.launch.py` |
| 底盘 + EKF + RobotModel/TF | `ros2 launch kiwi_slam kiwi_robot.launch.py` |
| 完整建图（E110 + SLAM Toolbox） | `ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true` |
| 组合建图导航（推荐建图入口） | `ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true` |
| AMCL 导航（已有地图） | `ros2 launch kiwi_navigation localization_navigation.launch.py map:=/home/ym/ros2_ws/maps/warehouse.yaml use_rviz:=true` |

保存地图：

```bash
mkdir -p /home/ym/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /home/ym/ros2_ws/maps/warehouse
```

`maps/` 目录在工作区根目录，不属于任何 ROS 包，不应提交到版本控制。

## 架构

### 数据链路

```
C 板 (USB CDC)
  → kiwi_base_node → /chassis/odom/raw
                   → /chassis/imu/data_raw
                   → /chassis/feedback/raw
                   → /joint_states
  → ekf_filter_node → /odometry/filtered + TF: odom → base_footprint

E110 (UART /dev/ttyUSB0)
  → bluesea_node → /scan
  → slam_toolbox → /map + TF: map → odom

robot_state_publisher → TF: base_footprint → base_link → lidar_link / wheel_links
```

完整 TF 树：`map → odom → base_footprint → base_link → lidar_link`

### kiwi_base 包内部结构

`kiwi_base_core` 静态库包含三个模块，其余可执行文件链接它：

- `robomaster_protocol`：USB 帧打包/解析（0x3A 控制帧 12B、0x3B 反馈帧 48B），CRC8 头 + CRC16 帧
- `kiwi_kinematics`：三轮全向正逆运动学（`/cmd_vel` → 三轮角速度，三轮角速度 → 里程计）
- `measurement_processing`：原始 IMU 和里程计数据滤波

可执行文件：`kiwi_base_node`（主驱动）、`kiwi_teleop_keyboard`（前台键盘）、`kiwi_odom_test`（里程计调试打印）

### 唯一参数文件

`src/kiwi_base/config/kiwi_base.yaml` 是底盘相关的**唯一配置入口**，`kiwi_base_node`、`ekf_filter_node`、`kiwi_teleop_keyboard`、`kiwi_odom_test` 全部从这一个文件读取参数。`kiwi_slam/urdf/kiwi.urdf.xacro` 也从该文件读取机器人结构尺寸。修改底盘参数只改这一个文件。

`src/kiwi_slam/config/lds_e110.yaml` 是 E110 雷达的**唯一配置入口**，`bluesea_node` 从中读取串口、帧 ID 和 `/scan` 参数。

### 坐标约定

+X 前进，+Y 左移，+Z 向上，+wz 逆时针。键盘 W/S/A/D/Q/E 分别对应前后、左右横移、逆/顺时针，空格或 X 停止。

### 重要约束

- **键盘与自主导航互斥**：`kiwi_teleop_keyboard` 必须在前台有 TTY 的终端运行，且不能与 Nav2 同时发布 `/cmd_vel`。
- **速度限幅在 `kiwi_base_node`**：无论来源是键盘还是 Nav2，`kiwi_base_node` 对 `linear.x/y` 和 `angular.z` 做最终限幅，不要在其他层重复限幅。
- **硬件设备**：C 板默认 `/dev/robomaster_c`（udev 符号链接），E110 默认 `/dev/ttyUSB0`，修改设备名分别改对应 YAML。

## 调试快速参考

```bash
ros2 topic hz /scan /odometry/filtered /map
ros2 topic echo /chassis/feedback/raw
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
```
