# kiwi_slam

`kiwi_slam` 提供 Kiwi 底盘的公共 RobotModel/TF、PaceCat LDS-E110-R UART 激光雷达配置、
SLAM Toolbox 二维建图和链路诊断节点。当前不包含 Livox、Mid360 或 FAST-LIO 适配器。

## 数据链路

```text
C 板 -> kiwi_base_node -> /chassis/odom/raw + /chassis/imu/data_raw
                           -> robot_localization
                           -> /odometry/filtered + odom -> base_footprint

E110 -> bluesea2 -> /scan -> slam_toolbox -> /map + map -> odom

robot_state_publisher -> base_footprint -> base_link
                       -> chassis_imu_link / lidar_link / wheel links
```

完整 TF 链为：

```text
map -> odom -> base_footprint -> base_link -> lidar_link
```

## 配置文件

```text
config/lds_e110.yaml       E110 串口、frame_id 和 /scan 参数
config/slam_toolbox.yaml   在线同步建图参数
config/monitor.yaml        /scan、里程计、地图和 TF 诊断参数
../kiwi_base/config/kiwi_base.yaml
                            底盘、EKF 和机器人模型唯一入口
urdf/kiwi.urdf.xacro       结构尺寸读取 YAML，传感器安装外参在此标定
```

E110 默认使用 `/dev/ttyUSB0`、230400 波特率、`lidar_link` 和 `/scan`。设备名不同先修改
`config/lds_e110.yaml` 的 `port`。

## 构建和启动

```bash
cd /home/a/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_slam
source install/setup.bash
```

只启动公共底盘、EKF 和 RobotModel：

```bash
ros2 launch kiwi_slam kiwi_robot.launch.py
```

启动 E110 和 SLAM Toolbox 建图：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

需要单独控制启动项时可使用：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py \
  start_base:=true start_ekf:=true start_lidar:=true \
  start_slam:=true start_monitor:=true use_rviz:=true
```

`kiwi_slam.launch.py` 只负责建图链路，不负责加载已保存地图，也不启动 AMCL；已知地图定位
请使用 `kiwi_navigation/localization_navigation.launch.py`。

## 建图和保存地图

建图时在另一终端运行前台键盘：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

确认 `/scan`、`/odometry/filtered` 和 `/map` 正常后低速移动。保存地图建议统一放到：

```bash
mkdir -p /home/a/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/a/ros2_ws/maps/warehouse
```

最终文件是 `/home/a/ros2_ws/maps/warehouse.yaml` 和 `warehouse.pgm`。启动导航时传入前者。

## 诊断

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
ros2 topic echo /diagnostics
```

诊断节点默认在 `kiwi_slam.launch.py` 中启动，在组合建图导航 launch 中默认关闭，可通过
`start_monitor:=true` 开启。

