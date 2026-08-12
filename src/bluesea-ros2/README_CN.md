# bluesea2 中文说明

本包在 Kiwi 工程中只用于 PaceCat LDS-E110-R 二维 UART 激光雷达。当前统一配置文件为：

```text
/home/a/ros2_ws/src/kiwi_slam/config/lds_e110.yaml
```

默认硬件和 ROS 接口：

```text
串口：      /dev/ttyUSB0
波特率：    230400
LaserScan： /scan
坐标系：    lidar_link
测距范围：  0.05 - 12.0 m
```

单独测试雷达：

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bluesea2 uart_lidar.launch \
  params_file:=/home/a/ros2_ws/src/kiwi_slam/config/lds_e110.yaml
ros2 topic hz /scan
```

正常运行时 `/scan` 的 `header.frame_id` 应为 `lidar_link`，并且 TF 中应存在
`base_link -> lidar_link`。完整建图、地图保存和 AMCL 导航流程请查看工作区根目录
`/home/a/ros2_ws/README.md`。

本包不负责底盘控制、SLAM、地图保存或导航规划。Kiwi 系统通常直接使用：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=/home/a/ros2_ws/maps/warehouse.yaml
```
