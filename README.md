# Kiwi ROS 2 移动机器人工作区

新增两电机履带底盘与底盘/雷达/惯导一键选择入口，参见 [可切换底盘平台架构](documents/platform_architecture.md)。本文件下方仍记录原 Kiwi 平台的运行方法。

这是 Kiwi 三轮全向底盘的 ROS 2 Humble 工作区。当前运行链路只使用：

- RoboMaster C 板 USB CDC 底盘驱动（0x3A 命令无 ACK，0x3B 电机/IMU 异步遥测）
- 镭神 LSLIDAR M10P 二维 UART 激光雷达
- `robot_localization` 平面 EKF
- `slam_toolbox` 二维在线建图
- Nav2 AMCL 已知地图定位和全向导航


## 1. 工作区结构

```text
ros2_ws/
├── src/
│   ├── kiwi_base/       底盘、运动学、反馈里程计、IMU、键盘
│   ├── kiwi_slam/       M10P 配置、SLAM Toolbox、URDF、链路诊断
│   ├── kiwi_navigation/ AMCL、Nav2、建图/定位导航 launch 和 RViz
│   ├── lslidar_driver/  M10P UART 驱动
│   └── lslidar_msgs/    镭神驱动消息接口
├── build/                colcon 构建目录
├── install/              colcon 安装目录
└── maps/                 用户保存的地图（需自行创建）
```

地图文件不放在源码包中。建议统一保存到：

```text
/home/ym/ros2_ws/maps/
```

每张地图至少包含同名的 `.yaml` 和 `.pgm` 文件，例如：

```text
maps/warehouse.yaml
maps/warehouse.pgm
```

导航启动参数使用 `.yaml` 文件：`map:=/home/ym/ros2_ws/maps/warehouse.yaml`。

## 2. 坐标、话题和速度接口

ROS 机体坐标遵循 REP-103：`+X` 向前，`+Y` 向左，`+Z` 向上，`+wz` 逆时针。
键盘中 `W` 发布 `vx+`，`A` 发布 `vy+`，`Q` 发布 `wz+`。`base_footprint` 和
`base_link` 的固定关节没有旋转，实车机械前方必须与红色 X 轴一致。

```text
/scan                    M10P LaserScan
/cmd_vel                 所有速度源的统一入口
/chassis/odom/raw        根据滤波后的电机实测速度推算的轮式里程计
/chassis/imu/data_raw    C 板异步 IMU 遥测（绝对 yaw、角速度、原始加速度）
/chassis/feedback/raw    C 板异步 0x3B 原始字段调试消息
/joint_states            根据电机实测速度换算的三个轮关节状态
/odometry/filtered       EKF 输出，Nav2 使用此话题
/map                     SLAM Toolbox 或 map_server 发布的地图
```

TF 链应为：

```text
建图：       map -> odom -> base_footprint -> base_link -> lidar_link
已知图导航： map -> odom (AMCL) -> base_footprint (EKF) -> base_link -> lidar_link
```

`kiwi_base.yaml` 是底盘和 EKF 的唯一参数入口。键盘的 `speed_m_s`、`yaw_rad_s` 是手动
速度，底盘的 `max_*` 是所有 `/cmd_vel` 输入的最终安全上限；两者不是重复配置。
Nav2 还会在 `kiwi_navigation/config/nav2.yaml` 中对自主导航速度进行单独限制。

## 3. 环境和硬件准备

支持环境：Ubuntu 22.04、ROS 2 Humble。连接硬件后确认设备节点：

```bash
ls -l /dev/robomaster_c   # C 板 USB CDC
ls -l /dev/wheeltec_lidar # M10P UART 的 udev 稳定名称
```

若设备名不同，修改以下两个参数后重新启动：

```text
src/kiwi_base/config/kiwi_base.yaml      device
src/kiwi_slam/config/ls_m10p_uart.yaml   serial_port
```

建议加入串口用户组并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

每次新终端先加载环境：

```bash
cd /home/ym/ros2_ws
source tools/activate_workspace.sh
```

不要在同一终端最后加载含旧 `kiwi_base` 的其他工作区。激活脚本会把当前工作区放到
overlay 最前面，并校验 `kiwi_base`、`kiwi_slam`、`kiwi_navigation` 的实际来源；这可避免
EKF 因误读旧 YAML 而静默回退到没有 `odom0/imu0` 的默认配置。

## 4. 构建和测试

首次或源码变更后构建当前链路：

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_navigation
source tools/activate_workspace.sh
```

只构建底盘：

```bash
colcon build --symlink-install --packages-select kiwi_base
```

运行测试：

```bash
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_base kiwi_slam kiwi_navigation \
  --event-handlers console_cohesion+ --ctest-args --output-on-failure
colcon test-result --all
```

构建后可先做平台、依赖和设备检查：

```bash
ros2 run kiwi_slam kiwi_slam_compatibility
ros2 run kiwi_slam kiwi_slam_compatibility --require-hardware
```

不带 `--require-hardware` 时，缺少串口只会给出警告，适合在无实车环境检查配置；实车启动前
应使用带该选项的检查。

## 5. 只启动底盘和 EKF

适合先确认 USB 命令、异步电机/IMU 反馈、轮式里程计和 TF，不启动雷达和 SLAM：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

检查输出：

```bash
ros2 topic hz /chassis/odom/raw
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /odometry/filtered
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

0x3A 命令没有 ACK，发送链不等待反馈；独立的 0x3B 遥测同时提供滤波后的电机实测速度
以及绝对 yaw/`gyro_z`。EKF 用后两者修正航向。该链路仍不能从 IMU 单独恢复打滑造成的 x/y 位移误差；
室外激光特征不足时，需要 RTK/GNSS、视觉惯导或其他绝对平移观测才能约束累计漂移。

## 6. 二维建图流程

### 6.1 启动建图系统

推荐使用组合启动文件，它会启动底盘、EKF、M10P、SLAM Toolbox 和 Nav2：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
```

如果只想建图、不启动 Nav2，可以使用：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

默认配置文件为：

```text
底盘/EKF：src/kiwi_base/config/kiwi_base.yaml
M10P：    src/kiwi_slam/config/ls_m10p_uart.yaml
SLAM：    src/kiwi_slam/config/slam_toolbox.yaml
Nav2：    src/kiwi_navigation/config/nav2.yaml
```

### 6.2 开始移动建图

在第二个终端运行键盘节点（必须使用前台可交互终端）：

```bash
cd /home/ym/ros2_ws
source tools/activate_workspace.sh
ros2 run kiwi_base kiwi_teleop_keyboard
```

按键：

```text
W/S  前进/后退       A/D  左/右横移
Q/E  逆时针/顺时针   空格或 X 立即停止
Ctrl-C              退出并发布零速度
```

建图时低速、匀速覆盖所有走廊和房间，避免快速旋转、急停和长时间无激光回波。确认：

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

### 6.3 保存地图

另开终端创建固定地图目录，然后保存：

```bash
mkdir -p /home/ym/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/ym/ros2_ws/maps/warehouse
```

命令成功后会生成：

```text
/home/ym/ros2_ws/maps/warehouse.yaml
/home/ym/ros2_ws/maps/warehouse.pgm
```

保存前应停止机器人并等待地图稳定。`.yaml` 是导航启动入口，`.pgm` 是地图图像，不能只保留
其中一个文件。保存后可检查：

```bash
ls -lh /home/ym/ros2_ws/maps/warehouse.*
```

保存完毕后在建图终端按 `Ctrl-C`，再停止键盘终端。

## 7. 已知地图定位和 Nav2 导航

### 7.1 选择地图

启动导航时必须传入要使用的地图 YAML，不能传 `.pgm`，也不能省略 `map`：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/a/ros2_ws/maps/ceshi3.yaml \
  use_rviz:=true
```

例如另一张地图：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/ym/ros2_ws/maps/lab.yaml
```

该 launch 会启动底盘、EKF、M10P、Nav2 的 AMCL、map_server、costmap、规划器、DWB 全向控制器
和 RViz；不会启动 SLAM Toolbox。导航时地图来源是 `map` 参数指向的 YAML 文件。

### 7.2 在 RViz 中定位和导航

1. 等待 AMCL、map_server 和 TF 正常启动。
2. 在 RViz 点击 `2D Pose Estimate`，在地图上点击机器人当前位置并拖出朝向。
3. 等待激光扫描与地图重合，必要时重新设置初始位姿。
4. 使用 `Nav2 Goal` 或 `2D Goal Pose` 设置目标点。
5. 观察规划路径、局部代价地图和 `/cmd_vel`，确认机器人按期望方向运动。

自主导航期间不要启动键盘节点，否则键盘和 Nav2 会同时向 `/cmd_vel` 发布命令。

## 8. 诊断命令

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
ros2 topic echo /chassis/odom/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic echo /odometry/filtered
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

启用 SLAM 诊断节点时：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py start_monitor:=true
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
ros2 topic echo /diagnostics
```

常见故障：

- 没有 `/scan`：检查 `/dev/wheeltec_lidar`、`ls_m10p_uart.yaml` 的 `serial_port` 和雷达供电。
- 没有 `/odometry/filtered`：检查 0x3B 遥测、底盘 `/chassis/odom/raw` 及 EKF 参数。
- 地图不更新：同时检查 `/scan`、`map -> odom`、`odom -> base_footprint` 和激光安装 TF。
- 键盘无效：确认键盘节点在前台 TTY 中运行，并且没有 Nav2 同时占用 `/cmd_vel`。
- 方向错误：ROS 约定是 `vx+` 前进、`vy+` 左移；应检查电机编号/符号和实车前方标定，不要重复旋转 TF。

## 9. 主要参数入口

| 功能 | 文件 |
| --- | --- |
| 底盘、运动学、反馈里程计、EKF、键盘 | `src/kiwi_base/config/kiwi_base.yaml` |
| M10P 串口和 `/scan` | `src/kiwi_slam/config/ls_m10p_uart.yaml` |
| SLAM Toolbox | `src/kiwi_slam/config/slam_toolbox.yaml` |
| SLAM 诊断 | `src/kiwi_slam/config/monitor.yaml` |
| Nav2、AMCL、DWB、代价地图 | `src/kiwi_navigation/config/nav2.yaml` |
| 机器人结构和传感器 TF | `src/kiwi_slam/urdf/kiwi.urdf.xacro` |
| 地图 | `/home/ym/ros2_ws/maps/*.yaml` 和对应 `.pgm` |

修改参数后重新构建相关包并重新 `source install/setup.bash`。地图文件不需要重新构建，直接在
启动导航时通过 `map:=绝对路径` 选择。
