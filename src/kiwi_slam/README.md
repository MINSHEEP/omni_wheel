# kiwi_slam

`kiwi_slam.launch.py` 是独立建图入口，读取 `kiwi_platform/config/active_platform.yaml`。Kiwi 启动轮式里程计、C 板 IMU 和 EKF；履带启动 M10P、`tracked_rf2o.yaml` 对应的 RF2O、协方差适配器、C 板 IMU 和 EKF。两种底盘分别读取 `slam_toolbox.yaml` / `tracked_slam_toolbox.yaml` 和 `mapping_gate.yaml` / `tracked_mapping_gate.yaml`，并使用各自的 URDF。参数文件列表与切换命令见 [平台架构](../../documents/platform_architecture.md)。

```text
Kiwi:  电机反馈 /chassis/odom/raw + IMU -> EKF -> /odometry/filtered
履带: /scan -> RF2O /chassis/odom/laser_raw -> 协方差适配 /chassis/odom/laser + IMU -> EKF -> /odometry/filtered
建图: /scan -> 建图门控 /scan_slam -> SLAM Toolbox -> /map
TF:   SLAM 或 AMCL: map -> odom；EKF: odom -> base_footprint；URDF: 底盘到雷达/IMU
```

RF2O 的 `init_pose_from_topic` 设为空以避免等待不存在的真值位姿，`publish_tf=false` 避免与 EKF 重复发布 TF。RF2O 上游里程计协方差全为零；履带启动链额外运行 `rf2o_covariance_adapter`，将 `/chassis/odom/laser_raw` 的平面估计转换到 `/chassis/odom/laser`，并按 `tracked_rf2o.yaml` 设置非零协方差。履带门控持续检查原始 IMU、适配后激光里程计和融合里程计的新鲜度；`mapping_sensors_verified=false` 时在启动前拒绝建图。

2026-09-25 域 42 的 129 秒静止 bag 显示：原始扫描两秒间中位差约 6 mm、地图栅格完全不变，RF2O 却漂移约 11.7 cm、2.8°，EKF 随之漂移。履带适配器现在只在连续两秒的扫描、原始 IMU 与 RF2O 位移同时满足静止阈值时固定输出位姿和零速度；运动恢复时通过平面位姿偏移连续续接，不把静止期间累积的 RF2O 伪位移一次释放。Kiwi 轮式里程计不经过此适配器。该约束不能补偿真实运动中的激光里程计误差，也不能代替狭窄通道的运动测试。

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

履带配置另将 SLAM Toolbox 的 `scan_queue_size` 设为 10。2026-09-25 实测扫描间隔
约 83 ms，而扫描收到后，覆盖其时间戳的 `odom → base_footprint` TF 通常晚约
100 ms、95 分位晚约 134 ms；默认 1 帧队列会导致扫描被丢弃，`map → odom` 持续
重复发布旧时间戳，即使 `/map` 仍有消息也不能证明建图仍在处理新扫描。
`kiwi_slam_monitor` 现在会将动态 TF 时间戳超过 `timeout_sec` 的情况报为错误。
用 `ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger {}` 检查时，
`map_to_odom_tf`、`odom_to_base_tf` 均应为 `transform available`。

RViz 固定坐标系为 `map` 时，地图固定，模型依赖 `map → odom → base_footprint`
移动；固定坐标系改为 `odom` 时，模型仍必须跟随 `odom → base_footprint`，
此时地图可能因 `map → odom` 校正而在屏幕上移动。若模型在 `odom` 下不动，
先在底盘运动时运行 `ros2 run tf2_ros tf2_echo odom base_footprint`：数值不变则
检查 RF2O、EKF；数值变化则展开 RViz 的 `RobotModel → Status` 核对模型和 TF。
旧的 `map → odom` 时间戳会使 `map` 下的扫描与模型显示失效，但单独不能解释
`odom` 下模型不动。
