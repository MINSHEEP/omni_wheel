# kiwi_slam

`kiwi_slam` 是 ROS 2 Humble / Ubuntu 22.04 下的三轮全向底盘在线建图集成包。
它不重复实现底盘和雷达驱动，而是组合以下现有模块：

- `kiwi_base`：RoboMaster C 板 USB 通信、三轮运动学、轮速里程计和 IMU 发布。
- `robot_localization`：融合死区后的轮速 `vx/vy`、相对 Euler yaw 与修正后 `gyro_z`，
  发布滤波里程计和动态 TF。
- `bluesea2`：读取 PaceCat LDS-E110-R 雷达并发布 `/scan`。
- `slam_toolbox`：激光建图、回环检测、`/map` 和 `map->odom`。
- `robot_state_publisher`：根据 URDF 发布底盘、IMU、雷达和轮子 TF。
- `kiwi_slam_monitor`：检查雷达、里程计、地图和 TF 链路。

## 系统链路

```text
RoboMaster C
  └─ USB CDC ─> kiwi_base_node
                 ├─ /wheel/odom_raw ─┐
                 ├─ /imu/data_raw ───┴─> ekf_filter_node
                 │                         ├─ /odometry/filtered
                 │                         └─ odom -> base_footprint
                 └─ /joint_states ─────> robot_state_publisher

LDS-E110-R
  └─ 230400 UART ─> bluesea_node ─> /scan ─> slam_toolbox
                                                   ├─ /map
                                                   └─ map -> odom

robot_state_publisher
  └─ base_footprint -> base_link -> imu_link/lidar_link/三个轮子
```

完整 TF 树应为：

```text
map                     SLAM Toolbox 发布
└── odom                EKF 的 world_frame
    └── base_footprint  EKF 发布动态 TF
        └── base_link   URDF 固定 TF
            ├── imu_link
            ├── lidar_link
            └── motor_link_1..3 -> wheel_link_1..3
```

职责边界：

```text
Xacro/URDF             定义 RobotModel、/robot_description 和机体静态 TF
robot_state_publisher  发布 URDF 中的 /tf_static 和轮关节 /tf
EKF                    发布 odom -> base_footprint
SLAM Toolbox           发布 /map 消息和 map -> odom
```

URDF/Xacro 本身不会发布 `/map`。没有 `/scan` 时可以正常显示 RobotModel，但不能生成
有效地图。`kiwi_slam.launch.py` 直接启动底盘与 EKF，不复用带过渡逻辑的底盘 launch；
IMU 和雷达 TF 只允许由 `urdf/kiwi.urdf.xacro` 维护。

## 雷达与底盘参数

LDS-E110-R V1.5 参数位于 `config/lds_e110.yaml`：

```text
接口            UART 3.3 V，230400，8N1
扫描范围        360°
扫描频率        约 7 Hz（420 rpm）
角分辨率        约 0.6°
有效量程        0.05～12 m
ROS 话题        /scan
ROS frame       lidar_link
默认设备        /dev/ttyUSB0
```

底盘 URDF 使用当前已知参数：轮径 `0.203 m`、圆心到轮心距离 `0.32256 m`。
雷达默认高度 `lidar_z=0.20 m`、IMU 默认位姿为零，仅用于首次启动，正式建图前必须实测。

## 安装依赖

```bash
sudo apt update
sudo apt install \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-rqt-graph
```

如果启动时报错 `No such file or directory: 'xacro'`，说明远端缺少
`ros-humble-xacro`，安装后不需要修改源码。

## 构建

首次构建或依赖发生变化时：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_slam
source install/setup.bash
```

如果底盘和雷达包已经构建，只更新了 `kiwi_slam`：

```bash
colcon build --symlink-install --packages-select kiwi_slam
source install/setup.bash
```

若 `base` 报错 `failed to create symbolic link ... Is a directory`，说明同一
`build` 目录混用了普通构建和 `--symlink-install`。保留旧产物后重建：

```bash
mv build/base build/base.non_symlink_backup
colcon build --symlink-install --packages-up-to kiwi_slam
```

不要删除 `src/base`，后续统一使用 `--symlink-install`。

## 硬件兼容性检查

该命令只检查环境和设备，执行完立即退出，不会启动建图：

```bash
ros2 run kiwi_slam kiwi_slam_compatibility \
  --lidar-port /dev/ttyUSB0 \
  --require-hardware
```

无硬件测试时去掉 `--require-hardware`，串口缺失只显示 `WARN`。

## 启动建图

不启动 RViz：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py
```

启动专用 RViz 配置：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

参数名必须是 `use_rviz`，不能写成 `use-rviz`。

专用 `config/kiwi_slam.rviz` 默认显示：

- `/map`
- `/scan`
- RobotModel
- TF
- Grid
- SLAM Toolbox 面板

RViz 固定坐标系默认为 `odom`。这样即使 SLAM 还没有生成 `map`，机器人模型
仍然可以显示；地图生成后会通过 `map->odom` 自动对齐。

不需要诊断服务、希望减少节点图业务连线时可关闭监视器：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true start_monitor:=false
```

TF2 内部的 `transform_listener_impl_*` 仍可能出现，这是 SLAM、EKF 或 RViz 的正常
TF 监听实现；应在 rqt_graph 勾选 `Hide tf nodes`，不能通过删除 TF 监听器解决。

覆盖传感器安装位姿：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true \
  lidar_x:=0.10 lidar_y:=0.00 lidar_z:=0.25 lidar_yaw:=0.00 \
  imu_x:=0.0115 imu_y:=-0.149 imu_z:=0.00 imu_yaw:=0.00
```

默认 IMU 平移已经按实测写入：相对 `base_link` 前方 `11.5 mm`、右侧 `149 mm`；坐标约定
为 x 向前、y 向左。IMU 高度和安装角仍保持零，测量后通过同一组 launch 参数更新。

## 诊断接口

持续查看完整诊断消息：

```bash
ros2 topic echo /kiwi_slam/diagnostics
```

监视节点会按诊断状态分别输出 INFO、WARN 或 ERROR；各级别使用独立日志调用位置，兼容
ROS 2 Humble 的 logger severity 缓存规则，状态恢复时不会因切换日志级别退出。

按需检查一次完整链路：

```bash
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
```

诊断内容包括：

```text
laser_scan       /scan 是否存在、frame、频率、有效点比例和超时
filtered_odom    /odometry/filtered 是否存在、frame、频率和四元数合法性
map              /map 是否存在、尺寸、分辨率和更新时间
base_to_laser_tf base_footprint -> lidar_link
odom_to_base_tf  odom -> base_footprint
map_to_odom_tf   map -> odom
```

## `/map` 不发布的排查顺序

SLAM Toolbox 必须收到能够变换到 `odom` 的首帧 `/scan`，才会产生有效地图。

`ros2 topic info /map --verbose` 显示 `Publisher count: 1` 只代表 SLAM Toolbox 已创建
发布器，不代表已经发布有效 OccupancyGrid。若诊断同时显示 `no scan received`，当前
唯一阻塞点仍是雷达数据，不应由 URDF 伪造空地图。

```bash
# 1. 雷达节点必须存在
ros2 node list | grep -E 'blue|lidar'

# 2. /scan 应有一个发布者，频率约 7 Hz，frame_id 应为 lidar_link
ros2 topic info /scan --verbose
ros2 topic hz /scan
ros2 topic echo /scan --once

# 3. EKF 必须持续输出滤波里程计
ros2 topic hz /odometry/filtered

# 4. SLAM 所需 TF 必须连续
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 run tf2_ros tf2_echo odom base_footprint

# 5. 检查 SLAM 和地图发布者
ros2 node info /slam_toolbox
ros2 topic info /map --verbose

# 6. 汇总判断
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
```

判断规则：

```text
现象                                原因或检查方向
/bluesea_node 不存在                雷达节点启动失败，查看主启动终端
/scan 没有发布者                    雷达端口、USB 转串口或驱动问题
/scan 有发布者但没有消息            波特率、协议、串口权限或硬件连接问题
/odometry/filtered 没有消息         C 板反馈、kiwi_base 或 EKF 问题
odom -> base_footprint 不存在       EKF 没有获得有效传感器输入
base_footprint -> lidar_link 不存在 robot_state_publisher/URDF 问题
/map 发布者为 0                     slam_toolbox 未运行或已经退出
/map 有发布者但没有有效消息         /scan 无法通过 TF 变换或首帧未处理
```

雷达设备与权限检查：

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id/*
grep -n "port:" \
  ~/ros2_ws/install/kiwi_slam/share/kiwi_slam/config/lds_e110.yaml
groups
```

用户应属于 `dialout` 组。修改组后需要重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## rqt_graph 节点图说明

启动节点图：

```bash
rqt_graph
```

推荐在界面中：

1. 图类型选择 `Nodes only`。
2. 勾选 `Hide tf nodes`。
3. 勾选 `Hide debug`。
4. 勾选 `Hide unreachable nodes`。

`/transform_listener_impl_<随机后缀>` 是 TF2 为内部坐标查询创建的监听节点：

- 主要订阅 `/tf` 和 `/tf_static`。
- 随机后缀用于防止节点重名。
- 不代表启动了多个底盘、雷达或 TF 发布器。
- 不会重复发布坐标变换，也不需要手动删除。

可检查其中任意一个：

```bash
ros2 node info /transform_listener_impl_XXXXXXXX
```

精简后，核心业务图应接近：

```text
bluesea_node --/scan--------------------> slam_toolbox --/map------> monitor/RViz
kiwi_base_node --/wheel/odom_raw--------> ekf_filter_node
kiwi_base_node --/imu/data_raw----------> ekf_filter_node
ekf_filter_node --/odometry/filtered----> monitor
kiwi_base_node --/joint_states----------> robot_state_publisher
```

如果节点图中没有 `bluesea_node` 和 `/scan`，建图一定不会开始。

## RobotModel 不显示

依次检查：

```bash
ros2 node info /robot_state_publisher
ros2 topic info /robot_description --verbose
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

固定关节由 `/tf_static` 发布；三个轮子的连续关节依赖 `kiwi_base_node` 的
`/joint_states`。没有电机反馈时，底盘和传感器固定部分仍应显示，轮子可能报告
缺少动态关节变换。

## 无硬件启动测试

仅启动 URDF 和诊断节点：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py \
  start_base:=false \
  start_lidar:=false \
  start_slam:=false \
  use_rviz:=false
```

此模式下 `no scan received`、`no odometry received` 和 `no map received` 属于预期结果。

## 构建与测试验证

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select kiwi_slam --event-handlers console_direct+
ctest --test-dir build/kiwi_slam --output-on-failure
```

当前测试覆盖 E110 参数、SLAM frame 与量程、URDF 关键链接、RViz 显示契约、
运行依赖和诊断计算。

## 保存地图

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/kiwi_map
```

保存前应低速完成一个闭环并回到起点，确认 RViz 中墙面没有双层、旋转或明显错位。
