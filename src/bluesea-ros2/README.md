# bluesea2

`bluesea2` 是当前 Kiwi 系统使用的 PaceCat LDS-E110-R 二维 UART 激光雷达驱动包。源码
目录名为 `bluesea-ros2`，ROS 包名为 `bluesea2`。

## 当前使用配置

Kiwi 建图和导航不直接使用本包默认的其他雷达参数，而是通过 launch 传入：

```text
/home/a/ros2_ws/src/kiwi_slam/config/lds_e110.yaml
```

当前接口：

```text
设备：       /dev/ttyUSB0
协议：       UART 230400 8N1
LaserScan：  /scan
frame_id：   lidar_link
范围：       0.05 - 12.0 m
```

若串口设备名不同，只改 `lds_e110.yaml` 的 `port`，不要在多个 launch 中复制参数。

## 单独启动雷达

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bluesea2 uart_lidar.launch \
  params_file:=/home/a/ros2_ws/src/kiwi_slam/config/lds_e110.yaml
```

检查数据：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

正常运行时 `LaserScan.header.frame_id` 应为 `lidar_link`，并且 TF 中存在
`base_link -> lidar_link`。

## 在 Kiwi 系统中的用法

一般不需要单独启动本包。以下组合启动会自动包含 UART 雷达：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/a/ros2_ws/maps/warehouse.yaml
```

本包只提供 E110 驱动，不负责底盘控制、建图、地图保存或 AMCL 定位。
