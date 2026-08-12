# kiwi_slam

`kiwi_slam` 是 Kiwi 底盘与 PaceCat LDS-E110-R 的 ROS 2 Humble 二维建图集成包。

## 系统链路

```text
RoboMaster C -> kiwi_base_node
  -> /chassis/odom/raw + /chassis/imu/data_raw
  -> ekf_filter_node
  -> /odometry/filtered + odom -> base_footprint

LDS-E110-R -> bluesea2 -> /scan -> slam_toolbox
  -> /map + map -> odom

robot_state_publisher
  -> base_footprint -> base_link -> chassis_imu_link / lidar_link / wheels
```

底盘和 EKF 同时读取 `kiwi_base/config/kiwi_base.yaml`。E110 参数在
`config/lds_e110.yaml`，SLAM 参数在 `config/slam_toolbox.yaml`，结构与安装外参在
`urdf/kiwi.urdf.xacro`。

## 构建与启动

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_slam
source install/setup.bash
```

启动二维建图：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py
```

同时启动 RViz：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

只启动底盘、EKF 和 RobotModel：

```bash
ros2 launch kiwi_slam kiwi_robot.launch.py
```

## 诊断

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
```

完整 TF 链应为 `map -> odom -> base_footprint -> base_link -> lidar_link`。
