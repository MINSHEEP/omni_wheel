# kiwi_slam

`kiwi_slam.launch.py` 是独立建图入口，读取 `kiwi_platform/config/active_platform.yaml`。Kiwi 启动轮式里程计、C 板 IMU 和 EKF；履带启动 M10P、`tracked_rf2o.yaml` 对应的 RF2O、C 板 IMU 和 EKF。两种底盘分别读取 `slam_toolbox.yaml` / `tracked_slam_toolbox.yaml` 和 `mapping_gate.yaml` / `tracked_mapping_gate.yaml`，并使用各自的 URDF。参数文件列表与切换命令见 [平台架构](../../documents/platform_architecture.md)。

```text
Kiwi:  电机反馈 /chassis/odom/raw + IMU -> EKF -> /odometry/filtered
履带: /scan -> RF2O /chassis/odom/laser + IMU -> EKF -> /odometry/filtered
建图: /scan -> 建图门控 /scan_slam -> SLAM Toolbox -> /map
TF:   SLAM 或 AMCL: map -> odom；EKF: odom -> base_footprint；URDF: 底盘到雷达/IMU
```

RF2O 的 `init_pose_from_topic` 设为空以避免等待不存在的真值位姿，`publish_tf=false` 避免与 EKF 重复发布 TF。履带门控持续检查原始 IMU、激光里程计和融合里程计的新鲜度；`mapping_sensors_verified=false` 时在启动前拒绝建图。

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
ros2 launch kiwi_slam kiwi_robot.launch.py start_ekf:=false start_lidar:=false
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
```

M10P 默认串口 `/dev/wheeltec_lidar`，C 板 `/dev/robomaster_c`；按实测 USB 序列号创建稳定 udev 规则和 `dialout` 权限的命令见[根 README](../../README.md)。M10P 驱动单独测试仍可运行 `ros2 launch kiwi_slam m10p_uart_lidar.launch.py`，它只启动雷达，不读取底盘开关。

建图 RViz 的固定坐标系为 `map`；SLAM 回环修正 `map → odom` 时地图应保持在屏幕
固定位置，机器人位姿会随修正变化。当前已打开的旧 RViz 需在 `Global Options →
Fixed Frame` 手动选择 `map`，或重启 RViz 加载更新后的配置。若墙体仍出现重影，
应继续检查实际扫描匹配，不能仅凭固定坐标系设置判定地图可用于导航。
履带 2026-09-24 录制的同一份 bag 做 425 秒离线对照：开启自动回环时，
约 414 秒机器人地图位姿一步跳约 2.0 m；关闭回环后，同区间最大一步
约 0.05 m。因此暂在履带 `tracked_slam_toolbox.yaml` 关闭自动回环，
保留 `use_scan_matching=true`。这会失去自动闭环对长程累计误差的修正，
后续须低速重录、核验墙体与起终点，再决定更严格的回环条件。
