# kiwi_navigation

`kiwi_navigation` 组合 PaceCat LDS-E110-R、SLAM Toolbox、AMCL 和 Nav2，实现 Kiwi
三轮全向底盘的二维建图与导航。

## 数据链路

```text
E110 -> /scan
/odometry/filtered + TF
  -> SLAM Toolbox 或 AMCL
  -> Nav2 DWB Omni
  -> nav2_velocity_smoother
  -> /cmd_vel
  -> kiwi_base_node
```

Nav2 Humble 的 `velocity_smoother` 直接输出 `/cmd_vel`。底盘节点使用同一话题并执行最终
限幅，不再启动额外速度中继。

## 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_navigation
source install/setup.bash
```

## 建图导航

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/kiwi_site
```

已知地图定位导航：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/kiwi_site.yaml
```

不要同时启动 `kiwi_teleop_keyboard` 和 Nav2，否则两者会同时发布 `/cmd_vel`。

## 调试

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```
