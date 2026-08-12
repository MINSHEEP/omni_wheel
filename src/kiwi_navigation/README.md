# kiwi_navigation

`kiwi_navigation` 集成 E110、SLAM Toolbox、AMCL 和 Nav2，为 Kiwi 三轮全向底盘提供二维
建图和已知地图导航。速度链路为：

```text
Nav2 DWB Omni -> velocity_smoother -> /cmd_vel -> kiwi_base_node
```

底盘节点会对所有速度源做最后限幅。导航使用 `base_footprint`、`odom` 和 `/odometry/filtered`。
AMCL 使用 `nav2_amcl::OmniMotionModel`，DWB 支持 `vy` 横向速度。

## 配置

```text
config/nav2.yaml           AMCL、map_server、costmap、DWB、速度平滑器
config/mapping_navigation  当前没有独立文件，使用 launch 默认参数
config/navigation.rviz     已知地图导航 RViz，Fixed Frame=map
```

底盘和 EKF 参数不复制到本包，统一读取：

```text
src/kiwi_base/config/kiwi_base.yaml
```

## 构建

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_navigation
source install/setup.bash
```

## 在线建图

组合建图导航启动文件会启动底盘、EKF、E110、SLAM Toolbox、Nav2 和 RViz：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
```

第二个终端启动键盘：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

建图完成后停止机器人，在第三个终端保存地图：

```bash
mkdir -p /home/a/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/a/ros2_ws/maps/warehouse
```

保存结果：

```text
/home/a/ros2_ws/maps/warehouse.yaml
/home/a/ros2_ws/maps/warehouse.pgm
```

地图目录是工作区根目录的 `maps/`，不会自动提交到任何源码功能包。每次保存一张新地图时
使用不同的前缀，例如 `lab`、`office`、`warehouse`。

## 已知地图 AMCL 导航

启动导航使用保存地图的 `.yaml` 文件：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/a/ros2_ws/maps/warehouse.yaml \
  use_rviz:=true
```

选择其他地图只修改 `map:=`，例如：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/a/ros2_ws/maps/lab.yaml
```

不要传 `.pgm`，也不要省略 `map`。该 launch 启动 AMCL、map_server 和 Nav2，但不会启动
SLAM Toolbox，因此不会修改地图。启动后在 RViz：

1. 使用 `2D Pose Estimate` 设置机器人在地图中的初始位置和朝向。
2. 等待激光与地图重合，必要时重新设置初始位姿。
3. 使用 `Nav2 Goal` 设置目标点。

自主导航时不要运行键盘节点，避免同时发布 `/cmd_vel`。

## 调试

```bash
ros2 node list
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

如果没有地图，检查 `map:=` 是否指向存在的 YAML；如果 AMCL 不收敛，重新设置初始位姿并
确认 `/scan` 的 `frame_id` 为 `lidar_link`。

