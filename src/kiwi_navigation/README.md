# kiwi_navigation

`kiwi_navigation` 是 ROS 2 Humble 下的二维建图与 Nav2 集成层。它不修改底盘协议、
FAST-LIO 或雷达驱动，只组合标准雷达接口、SLAM Toolbox、AMCL 和 Nav2。

## 坐标系结论

```text
Mid-360 + FAST-LIO:  camera_init -> body，显示 /cloud_registered 和 /Laser_map
Pace + Nav2:         map -> odom -> base_footprint -> base_link -> lidar_link
```

`camera_init` 是 FAST-LIO 的算法世界坐标系，不是 `camera_link`。当前 FAST-LIO 原生 TF
被隔离到 `/fast_lio/tf`，因此三维 RViz 切到 `odom` 或 `map` 后没有完整变换链，点云会
消失。三维建图使用 `kiwi_lio.launch.py` 和 `camera_init`；二维建图导航使用本包和 `map`，
两条路线不要在同一个 RViz 中手动混切 Fixed Frame。

## 雷达解耦接口

建图和导航只订阅 `/scan`，不订阅 E110 或 Mid-360 的原始消息：

```text
E110:    bluesea2 -> /scan (sensor_msgs/msg/LaserScan)
Mid-360: livox_ros_driver2 -> /livox/lidar (CustomMsg)
         -> kiwi_livox_scan_adapter -> /scan (sensor_msgs/msg/LaserScan)
```

两种雷达使用同一套 SLAM、Nav2、costmap 和 RViz 配置。替换雷达只改启动参数：

```bash
# E110
ros2 launch kiwi_navigation mapping_navigation.launch.py lidar_type:=pace

# Mid-360：使用原始 Livox 点云投影成二维扫描，仍由 SLAM Toolbox 建图
ros2 launch kiwi_navigation mapping_navigation.launch.py lidar_type:=mid360
```

Mid-360 的 FAST-LIO 三维路线仍通过 `kiwi_slam kiwi_lio.launch.py` 独立运行；它发布的
`/Laser_map` 是三维点云，不能直接替代 Nav2 的二维 `/map`。如果以后更换其他雷达，只需
新增一个驱动适配 launch，使其发布 `/scan` 和正确的传感器 TF，不改 SLAM/Nav2 参数。

当前 Mid-360 二维适配按水平高度切片投影原始点云，未使用 FAST-LIO 的运动去畸变；应先
低速、静止验证地图质量。若高速运动产生重影，优先使用 E110 二维路线，或后续增加基于
FAST-LIO 配准点云的投影节点，算法层接口仍保持 `/scan` 不变。

## 数据链路

```text
/scan（由雷达适配层提供） + /odometry/filtered + TF
  -> SLAM Toolbox
  -> /map + map->odom
  -> Nav2 global/local costmap
  -> SmacPlanner2D + DWB Omni
  -> /cmd_vel_nav
  -> nav2_velocity_smoother
  -> /cmd_vel
  -> kiwi_nav_cmd_relay（限幅、非法值归零、0.30 s 超时清零）
  -> /cmd_vel_safe
  -> kiwi_base_node
```

第一阶段局部规划器使用 DWB，全向参数已显式开启 `vy` 正负采样；全局规划器使用
SmacPlanner2D。速度上限暂设为 `vx/vy=0.30 m/s`、`wz=0.80 rad/s`，实车验收后再只修改
`config/nav2.yaml` 和 `config/cmd_relay.yaml`。

## 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_navigation
source install/setup.bash
```

## 建图并启动 Nav2

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py lidar_type:=pace
```

该命令会启动底盘、EKF、Pace E110、SLAM Toolbox、Nav2、速度中继和 RViz。RViz 固定
坐标系为 `map`。不要同时启动 `kiwi_teleop_keyboard`，否则 `/cmd_vel_safe` 会有两个发布者。

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/kiwi_site
```

## 已知地图定位与导航

```bash
# E110
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/kiwi_site.yaml lidar_type:=pace

# Mid-360
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/kiwi_site.yaml lidar_type:=mid360
```

该模式关闭 SLAM Toolbox，由 AMCL 独占 `map -> odom`。AMCL 使用
`nav2_amcl::OmniMotionModel`，支持三轮全向底盘横移。

## 调试接口

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /odometry/filtered
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
ros2 topic info /cmd_vel_safe --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 action list | grep navigate
```

导航前必须满足：`/scan`、`/odometry/filtered` 连续，且
`map -> odom -> base_footprint -> lidar_link` 可查询。当前中继解决命令超时，不替代物理
急停、MCU watchdog 或后续的 `nav2_collision_monitor`。
