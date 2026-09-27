# kiwi_slam

MID360S + 内部 IMU 的 FAST-LIO2 链路在 `mid360_pipeline.launch.py`，由 `kiwi_navigation/mid360_navigation.launch.py` 编排二维建图/AMCL。FAST-LIO2 内部完成惯性融合，新链路不启动外部 EKF；旧 M10P、RF2O、C 板 IMU 融合链保留为独立入口，C 板 IMU 话题仍可供诊断。具体部署和实机 IP 配置见[部署说明](../../documents/navigation/mid360_fastlio2_deployment.md)。

### 查询不到话题、录包为空或建图停止更新

启动、RViz、查询、录包终端必须使用一致的通信设置。`source tools/activate_workspace.sh`
默认设置 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`，已有显式环境变量优先。
本地开发机若使用其他域，保留该域。跨机 ROS 通信需要所有参与进程显式设置
`ROS_LOCALHOST_ONLY=0`；仅通过 SSH 或网页操作远程上位机不需要开放 ROS 跨机发现。
改变通信设置后，旧节点需要停止并在新环境重新启动；查询 daemon 也需要重启。

上位机建图终端：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh
ros2 daemon stop
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

另一终端先核对直接发现，收到状态后再移动。校准通常需要静置约 15 秒：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh
ros2 node list --no-daemon --spin-time 5
ros2 topic echo /chassis/imu/calibration_ready std_msgs/msg/Bool --once --qos-durability transient_local
ros2 topic echo /chassis/imu/data_ready std_msgs/msg/Bool --once --qos-durability transient_local
ros2 topic echo /chassis/estimation/valid std_msgs/msg/Bool --once --qos-durability transient_local
ros2 topic echo /chassis/estimation/state std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /chassis/inertial/reason std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /kiwi_slam/mapping_gate_ready std_msgs/msg/Bool --once --qos-durability transient_local
ros2 param get /slam_toolbox do_loop_closing
```

启动时四个 Bool 状态（标定、数据、估计有效、建图）都必须是 `data: true`，估计 String 状态为 `VALID`；`false` 表示当前未就绪，须等候或查日志。运动中的 `INERTIAL` 或 `PREDICTED` 表示有界续接，扫描此时暂停入图。
录制终端也必须执行上述 `export` 和 `source`。录包显示 `Recording...` 只表示
录制器启动，不能证明传感器数据已订阅。确认 `/scan`、`/chassis/imu/data_raw`、
`/chassis/odom/laser_raw` 等订阅成功后再开始动作。可用现场采集脚本同时保存日志：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh
bash tools/record_incident.sh
```

按 Ctrl+C 结束后脚本生成故障现场 `.tar.gz`；解包后用 `ros2 bag info <解包后的现场目录>/bag/all_topics` 检查消息数，也可先查看包内 `snapshots/bag_info.txt`。
若出现 `RF2O increment disagrees with background alignment` 或 `Laser tracking lost`，
会拒绝异常帧并核验原参考，进而暂停 `/scan_slam`；有效惯导可短时续接公共里程计和 TF，详见后文。明确报 `reference_expired_restart_required` 时已超出自动恢复范围，必须重新定位或重新建图；RViz 显示旧地图不能说明定位仍有效。
这只能证明保护触发，不能证明 RF2O 一定错误，也不能证明保护没有误判。
先停车并保留 bag 和节点日志，分析冲突帧后再调整；仅重启 RViz 不能解除该状态。

履带使用 `tracked_imu_acceleration`：在上位机校正三轴加速度并扣除固定平放时的重力/偏置，补偿 IMU 前置 8 cm 的转向加速度后，供 EKF 融合 ax/ay，并供独立 `tracked_inertial_odometry` 短时积分；原有 yaw 转换保持不变。独立惯导没有绝对 xy 观测，不保证无特征长走廊的长期精度。[当前建图与惯导复核报告](../../documents/tracked_mapping_optimization_20260926.md)；[此前标定依据与适用范围](../../documents/tracked_inertial_validation.md)。

`kiwi_slam.launch.py` 是独立建图入口，读取 `kiwi_platform/config/active_platform.yaml`。Kiwi 启动轮式里程计、C 板 IMU 和 EKF；履带启动 M10P、`tracked_rf2o.yaml` 对应的 RF2O、协方差适配器、C 板 IMU、惯性预处理、独立惯导、EKF 和估计保护。两种底盘分别读取 `slam_toolbox.yaml` / `tracked_slam_toolbox.yaml` 和 `mapping_gate.yaml` / `tracked_mapping_gate.yaml`，并使用各自的 URDF。参数文件列表与切换命令见 [平台架构](../../documents/platform_architecture.md)。

```text
Kiwi:  电机反馈 /chassis/odom/raw + IMU -> EKF -> /odometry/filtered
履带: /scan -> RF2O /chassis/odom/laser_raw -> 协方差适配 /chassis/odom/laser + IMU -> EKF -> 保护节点 -> /odometry/filtered
履带惯性: /chassis/imu/data_raw -> 启动校准 -> /chassis/imu/gyro + /chassis/imu/acceleration -> EKF
履带续接: 已校准IMU + 可信激光位置/多帧速度 -> 独立惯导 /chassis/odom/inertial -> 保护节点有界选择
履带部分观测: 可观测的局部激光速度 /chassis/odom/laser_partial -> EKF速度观测 / 独立惯导强平移方向修正
履带建图: /scan -> 背景质量检查 /scan_validated -> 建图门控 /scan_slam -> SLAM Toolbox -> /map
Kiwi 建图: /scan -> 建图门控 /scan_slam -> SLAM Toolbox -> /map
TF:   SLAM 或 AMCL: map -> odom；Kiwi EKF / 履带保护节点: odom -> base_footprint；URDF: 底盘到雷达/IMU
```

RF2O 的 `init_pose_from_topic` 设为空以避免等待不存在的真值位姿，`publish_tf=false` 避免与 EKF 重复发布 TF。RF2O 上游里程计协方差全为零；履带启动链额外运行 `rf2o_covariance_adapter`，将 `/chassis/odom/laser_raw` 的平面估计转换到 `/chassis/odom/laser`，并按 `tracked_rf2o.yaml` 设置非零协方差。履带门控持续检查原始 IMU、适配后激光里程计和融合里程计的新鲜度；`mapping_sensors_verified=false` 时在启动前拒绝建图。

### 当前履带静止与运动处理

启动时在特征充分处静置约 15 秒：背景扫描确认静止后，IMU 收集 10 秒有效窗口，校正本次残余 gyro_z 偏置和静止加速度合量。驱动的符号/倍率、安装旋转保留。`/chassis/imu/calibration_ready` 表示本次标定有效；`/chassis/imu/data_ready` 表示当前连续数据可以用于融合。短时中断保留标定、清空跨间隔积分，至少验证 0.3 秒和 15 帧连续数据后恢复。时间倒退、无效数值/坐标或超过 2 秒的 IMU 中断需重启。

`rf2o_covariance_adapter` 增加鲁棒背景对齐、归一化几何可观测性、方向相关协方差和 RF2O 增量检查。固定停车参考避免逐帧积累零漂；人员经过不能仅因测距中位差超过 11 mm 就解除保持。正常起步的持续性确认不再将有效背景匹配标为质量失败，也不触发惯导接管/重置；解除保持时将确认期间累计的真实位移带回原 odom 坐标，不能吞掉起步距离。只有背景确认停车时才发布 `/chassis/odom/stationary` 零速度约束。它是有条件的运动约束，不是编码器测量。

完整位姿不再可观测、但连续局部扫描仍有可靠匹配时，适配器可发布
`/chassis/odom/laser_partial` 和 `/chassis/odom/laser_partial_ready`。该消息只有
`twist` 是观测，姿态/位置没有测量含义，不能作为定位输入。局部匹配需通过重叠率、残差、
可观测秩及至少 3 帧、0.2 秒的持续性检查；弱方向给出大方差并保留相关项，不能用伪零速度
约束不可观测方向。完整 `/chassis/odom/laser` 与部分观测逐帧互斥；EKF 只融合部分消息的
速度，独立惯导只吸收其中强可观测的平移速度方向，不重复融合其中的角速度。
部分观测不更新完整激光参考、不刷新惯导完整锚点或 2 秒期限、不置完整 `laser_ready`，
也不产生 `/scan_validated`；因此不能靠部分观测无限延长无绝对位置约束的走廊航行。

`quality_enabled=true` 时使用新背景检查，旧 `stationary_max_scan_median_delta_m` 等 legacy 参数不参与新判据。扫描与 RF2O 消息按原始时间戳配对，质量通过的原扫描才发布到 `/scan_validated`。未知运动、退化或中断暂停建图。距最后可信扫描 2 秒内，可验证原参考，或验证连续局部扫描与有界惯性位移的一致性；通常至少 3 帧且持续 0.2 秒通过才恢复。2–10 秒内惯导已停止续接，只允许强旧参考几何匹配并至少 5 帧、持续 0.4 秒核验恢复。累计位移与 RF2O 冲突时也采用更严格的旧参考匹配判据。超过 10 秒明确要求重新定位或重新建图，不把新扫描直接置零。恢复始终保留原 odom 坐标；长走廊位置并未因此获得长期保证。

### 履带估计有效性与恢复

`tracked_estimation_guard`、`tracked_inertial_odometry` 与惯性预处理复用本包，不新增功能包。参数统一放在
`kiwi_base/config/tracked_pwm.yaml`，各底盘/EKF、建图、导航入口通过同一个平台选择器启动。
Kiwi 轮式链路不启动该节点，继续由自己的 EKF 发布里程计和 TF。

履带 EKF 设置 `publish_tf=false`，内部消息改为 `/chassis/odometry/filtered_unchecked`；
该话题仅用于诊断，不能接到导航。保护节点核验后才发布 `/odometry/filtered` 和唯一
`odom → base_footprint` TF。激光短时失效时，若 IMU 与独立惯导仍有效，保护节点切到
`INERTIAL`，最多按最后可信锚点续接 2 秒；这段时间 EKF 保持运行，公共输出选用惯导，
惯导结果不反灌 EKF。IMU 失效、续接超时、数据超时或方差越界后停止公共有效输出，
并通过 EKF toggle 服务暂停内部积分。`max_prediction_s=0.2` 仍限制当前数据/输出
的新鲜度，不是惯导的总续接时间；`max_measurement_age_s=0.35` 限制消息头测量延迟。

激光质量心跳短时跌落且惯导也不可用时，`PREDICTED` 只从进入故障前最后可信的公开位姿
用新鲜陀螺和常速度模型最多续推 0.45 秒，同时传播模型误差协方差；自己的预测输出
不能刷新这个期限。恢复后先核验 EKF 与激光的同时间一致性再交回。该状态与单源
`LASER` 状态都不允许向建图写扫描；期限到或陀螺失效仍停止有效输出。

`max_position_variance=4.0` m² 是惯导 xy 的正常方差预算；锚点本身已超过该值时，
保留真实继承方差，不因继承例如 12.5 m² 就立即判死，也不人为压低锚点。此时允许的总量
受继承基线加有限增长约束；所有情况仍检查 `max_position_growth_variance=0.09` m²
和 `max_yaw_growth_variance=0.05` rad²，以及 2 秒完整观测期限。瞬时几何测量方差与
跨中断累计的参考不确定度分开保存，避免一次大几何方差成为以后永久叠加的下限；实际
惯导桥接新增的不确定度仍保留。这些是保护预算，不是本次已实现的实车定位精度。

短暂 `INERTIAL` 恢复顺序：IMU 数据验证 → 旧激光参考或有界惯性桥验证 → 持续健康
0.3 秒 → 按同一测量时间比较 EKF 与激光 → 直接交回仍在运行的 EKF。此路径不调用
`SetPose`，不清零速度；公共输出交接保持时间戳递增，并有界消化两源差异。真正进入
`PAUSED`，或时间对齐后仍持续不一致时，才暂停滤波、在原 odom 坐标设置已核验的激光
位姿、清除旧速度/加速度及积压观测，再恢复滤波并预热核验。即使惯导仍提供有效定位，
建图门控也必须等到估计状态 `VALID` 才恢复 `/scan_slam`；
`LASER`、`INERTIAL`、`PREDICTED`、重置和预热期间的扫描均不入图。不会在恢复时归零机器人位置，
也不会把冻结显示冒充有效定位。
保护状态服务未确认、观测不新鲜或 EKF 与激光位置/航向差超过配置阈值时继续拒绝。

履带 INS、估计保护、激光适配器和建图门控内部的实时就绪心跳订阅使用可靠、
VOLATILE、深度 10；对外状态发布仍为 TRANSIENT_LOCAL，命令行晚加入仍可读取
最近状态。内部必须接收新心跳后才放行，避免把旧保留状态视为新观测。
全向底盘只在标定状态改变时发布，建图门控仍使用 TRANSIENT_LOCAL 订阅该状态，
兼容先启动底盘、后启动建图。此改动不放宽数据新鲜度或惯导续接时限。

静止位姿保持不等于仍有可靠的零速度观测。扫描进入模糊/运动确认阶段，或校准后的
平面加速度在有限窗口内持续显著时，停止发布零速度约束，防止起步加速度被压制。
加速度只用于否决零速，不单独判定绝对位置；激光匹配仍负责核验真实位移。恢复成功后，
提前缓存的后续扫描按已核验参考重新匹配，不能继续使用旧失效状态，也不强行放行坏扫描。

- `/chassis/estimation/valid`：带心跳的定位有效性状态；`INERTIAL`、`LASER`、`PREDICTED` 期间可为 true，但此时建图门控仍为 false。消费端还应检查心跳新鲜度。
- `/chassis/estimation/state`：`PAUSING`、`PAUSED`、`RESETTING`、`ENABLING`、`WARMING`、`VALID`、`INERTIAL`、`LASER`、`PREDICTED`。
- `/chassis/inertial/reason`：惯导当前锚定、续接或失效原因；如 `coast_timeout` 或 `position_uncertainty_growth_limit`。
- 短暂中断可自动核验恢复；超过 10 秒旧参考期限，停车后通过可靠重定位或重新建图处理，不能只重启 RViz。
- 自动回环仍开启；数据恢复验证不等于长走廊定位精度已验收。

本轮现场证据、回归结果与验收范围统一见
[建图卡顿与惯导可用性复核](../../documents/tracked_mapping_optimization_20260926.md)。
[惯性续接与建图恢复](../../documents/tracked_inertial_recovery_20260926.md)是历史前版报告，
其中六节点测试通过不能替代本轮验证，更不能宣布真实窄道精度通过。历史 09-25 的短静止
记录及 09-26 人员经过测试仍见 [此前修复记录](../../documents/tracked_estimation_repair_20260926.md)。

录包停止后可运行只读分析；将目录替换为实际 rosbag2 目录（其中有 `metadata.yaml`）：

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
bag_dir="$HOME/ros2_ws/bags/tracked_YYYYMMDD_HHMMSS"
python3 tools/analyze_tracked_estimation.py "$bag_dir" --output "${bag_dir}_review"
```

可追加 `--launch-log /绝对路径/bringup.log`，也可按[真值格式约定](../../documents/tracked_mapping_optimization_20260926.md#外部真值输入约定)
传入 `--groundtruth`。报告将连续性、源间一致性和外部精度分别列出；没有独立实测距离/
角度及对应时间的真值，精度为 `NOT_EVALUATED`。激光、惯导、EKF 共用输入，三条轨迹
相似不能证明位置准确，也不能用 EKF 导出的轨迹充当独立真值。

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

另起已加载同一工作区和 ROS 域的终端，每次运行一条频率检查，Ctrl+C 结束该检查后再运行下一条：

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
```

M10P 默认串口 `/dev/wheeltec_lidar`，C 板 `/dev/robomaster_c`；按实测 USB 序列号创建稳定 udev 规则和 `dialout` 权限的命令见[根 README](../../README.md)。M10P 驱动单独测试仍可运行 `ros2 launch kiwi_slam m10p_uart_lidar.launch.py`，它只启动雷达，不读取底盘开关。

建图 RViz 的固定坐标系为 `map`；SLAM 回环修正 `map → odom` 时地图应保持在屏幕
固定位置，机器人位姿会随修正变化。当前已打开的旧 RViz 需在 `Global Options →
Fixed Frame` 手动选择 `map`，或重启 RViz 加载更新后的配置。若墙体仍出现重影，
应继续检查实际扫描匹配，不能仅凭固定坐标系设置判定地图可用于导航。
履带已恢复 `do_loop_closing=true`，粗/精匹配最低响应分别为 0.5/0.6。此前旧数据曾发生误回环，因此新配置必须用新录制的低速闭合路线验收；开启参数和静止重放成功不等于真实回环已验证。Kiwi 的回环配置保持原值。

履带地图栅格发布周期 `map_update_interval=0.5` 秒，约对应 2 Hz 的配置目标；它不是
雷达约 12 Hz 的扫描处理频率。新增关键帧仍取决于 `minimum_travel_distance=0.20` m、
`minimum_travel_heading=0.20` rad 等条件，静止时可持续发布相同栅格。不能只因地图
没有逐扫描刷新就认定链路卡住，应同时检查 `/scan_slam`、TF、估计状态和门控。

履带配置另将 SLAM Toolbox 的 `scan_queue_size` 设为 10。2026-09-25 实测扫描间隔
约 83 ms，而扫描收到后，覆盖其时间戳的 `odom → base_footprint` TF 通常晚约
100 ms、95 分位晚约 134 ms；默认 1 帧队列会导致扫描被丢弃，`map → odom` 持续
重复发布旧时间戳，即使 `/map` 仍有消息也不能证明建图仍在处理新扫描。
`kiwi_slam_monitor` 现在会将动态 TF 时间戳超过 `timeout_sec` 的情况报为错误。
`inertial_fallback` 另外检查惯导 valid 心跳：可用显示 `bounded inertial fallback available`，
不可用显示 `inertial fallback unavailable`，并给出 reason 与年龄。即使主 EKF 当前正常，
该项仍可报告备用惯导不可用，避免把节点已启动误认为退化后必能续接。
地图监控同时显示接收年龄、`header_age_sec` 和 `stamp_unchanged_sec`，并检查履带
估计状态及建图门控；周期重发旧地图不再掩盖上游暂停。健康静止时地图可不生成
新关键帧，单独的旧地图时间戳不判故障；静态地图模式也不要求持续生成新地图。
用 `ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger {}` 检查时，
`map_to_odom_tf`、`odom_to_base_tf` 均应为 `transform available`。

RViz 固定坐标系为 `map` 时，地图固定，模型依赖 `map → odom → base_footprint`
移动；固定坐标系改为 `odom` 时，模型仍必须跟随 `odom → base_footprint`，
此时地图可能因 `map → odom` 校正而在屏幕上移动。若模型在 `odom` 下不动，
先在底盘运动时运行 `ros2 run tf2_ros tf2_echo odom base_footprint`：数值不变则
检查 RF2O、EKF；数值变化则展开 RViz 的 `RobotModel → Status` 核对模型和 TF。
旧的 `map → odom` 时间戳会使 `map` 下的扫描与模型显示失效，但单独不能解释
`odom` 下模型不动。

### RViz 关闭崩溃与启动等待（2026-09-26）

默认 RViz 配置移除了 `slam_toolbox::SlamToolboxPlugin` 控制面板。
实机崩溃栈指向该插件的后台参数读取线程 `updateCheckStateIfExternalChange()`，
本次发生在 launch 收到 Ctrl+C 时。地图、激光扫描和模型显示保留，自动回环仍由 SLAM 参数控制。
建图 RViz 的 LaserScan 改为 `/scan_slam`：等待 IMU 校准和扫描门控放行后才接收点云，
避免启动阶段原始 `/scan` 在 TF 尚未齐备时持续填满 RViz 的变换队列。
远程 Intel 核显实测关闭强制软件渲染后 RViz CPU 占用显著下降，建图、建图导航及定位导航
三个入口的 `rviz_software_rendering` 默认均改为 `false`；确需强制软件渲染时可在启动命令中
传 `rviz_software_rendering:=true`。RViz 的 shader 警告仍需结合画面确认，不能仅凭日志认定地图损坏。
请重新加载工作区的 `kiwi_slam.rviz`，不要继续使用包含该面板的旧个人配置。
保存地图使用命令：

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/warehouse
```

履带本次启动残余偏置校准窗口改为 10 秒，前后还需扫描静止确认和融合放行时间；通常约 15 秒。
静止、数据连续性、噪声及窗口前后均值检查仍启用，不稳定时会重新收集，不能保证固定 10 秒就绪。
这只是当前开机残余偏置估计，不代替安装轴向、倍率和温漂标定。短时数据恢复不会重新收集 10 秒窗口。
`tools/record_incident.sh` 默认录制到 Ctrl+C，并没有固定 30 秒等待；已启动系统可用
`bash tools/record_incident.sh --duration 10` 做短故障采集。若要观察长期零漂，仍需保留几分钟数据。
