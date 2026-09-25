# 履带平面加速度标定与惯性辅助融合

## 范围与实现

本功能针对用户确认的 C 板及底盘固定平放条件，进行上位机侧的三轴加速度坐标校正、静止重力/偏置合量扣除及平面杆臂补偿。不是六面加速度内参标定，不适用于坡地或车身明显俯仰/侧倾。单个静止姿态不能独立确定加速度偏置、比例系数及完整外参；因此用直线启停数据额外确定前进轴，不从重力猜测水平朝向。

保持驱动现有 gyro_z 符号、倍率和零偏；加速度节点不再转换 yaw，不修改固件协议。配置仍集中在 `kiwi_base/config/tracked_pwm.yaml`。

```text
/chassis/imu/data_raw 的 gyro_z ───────────────────→ EKF imu0
                         加速度 → tracked_imu_acceleration → EKF imu1(ax/ay)
/scan → RF2O → 协方差/静止保持 ─────────────────────→ EKF odom0
                                                     ↓
                                  /odometry/filtered + odom→base_footprint
                                                     ↓
                                                 slam_toolbox
```

这是带加速度预测的现有 EKF，尚不是带在线偏置估计和完整退化处理的三维 INS。它不增加独立的 IMU 位置话题，也不将两次积分得到的相关位置再次作为独立观测重复融合。没有激光可靠约束时仍会累积误差，不能承诺穿越长走廊的精度。

全向底盘仍走原有轮式里程计链路；只有 RF2O 履带档案启动加速度节点。TF 仍由一个 EKF 发布。`imu1_remove_gravitational_acceleration: false`，因为节点已扣除静止重力合量；imu1 只融合 ax/ay，原始 IMU yaw 仅融合一次。

## 自动标定工具

编译后可执行 `ros2 run kiwi_slam calibrate_planar_imu --help`。脚本不发送运动指令，不覆盖活动配置；输出完整的候选底盘 YAML 和同名 `.report.json`。

基于已经停止录制的 bag：

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
ros2 run kiwi_slam calibrate_planar_imu \
  --static-bag "$HOME/ros2_ws/bags/tracked_static_verified_20260925_152840" \
  --motion-bag "$HOME/ros2_ws/bags/tracked_motion_20260924_164557" \
  --config src/kiwi_base/config/tracked_pwm.yaml \
  --output /tmp/tracked_imu_candidate.yaml
```

重新采集静止基线时，先只启动底盘驱动；人工确认静止后在第二个终端运行：

```bash
ros2 run kiwi_slam calibrate_planar_imu \
  --live-static-seconds 60 \
  --motion-bag "$HOME/ros2_ws/bags/tracked_motion_20260924_164557" \
  --config src/kiwi_base/config/tracked_pwm.yaml \
  --output /tmp/tracked_imu_candidate_new.yaml
```

运动 bag 应为特征充分区域的多次直线启停，同时包含加速和减速。脚本使用原始 IMU 和 `/chassis/odom/laser`，排除明显转向、侧滑和激励不足的窗口。对速度差分和加速度使用相匹配的时间窗口，避免起步脉冲造成虚假的比例误差。不读取正在录制的 bag；当前支持 sqlite3 `.db3` 或其目录。

更换 C 板、固件数据坐标处理、安装位置后，必须重新采集直线启停，不应复用旧 motion bag。静止检查只能拒绝明显旋转/振动，不能区分无加速度的匀速运动，人工静置前提不能省略。激光参考不等于独立地面真值，脚本不会按它拟合加速度比例系数。

候选配置手动建图测试：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py \
  base_params_file:=/tmp/tracked_imu_candidate.yaml use_rviz:=true
```

仍由活动底盘 YAML 选择底盘。测试后使用原来的建图命令即可回到活动配置。`tracked_imu_acceleration.ros__parameters.enabled: false` 可单独关闭加速度输出，回到激光 + 原有 gyro_z 融合；不需要修改 yaw 标定。

## 时间与转向

现有 IMU 时间戳为上位机接收时间，约 100 Hz 的样本存在 USB 成批到达。节点先聚合至少 20 ms 的样本，并用约 100 ms 的角速度窗口求角加速度；遇到时间倒退、重复或超过 0.2 s 的断流会清空历史，不跨断点求导。这不能替代 MCU 真实采样时间，后续精度升级仍应增加或验证采样时序。

IMU 位于底盘中心前方 0.08 m，节点扣除 `alpha × r + omega × (omega × r)`。否则原地转向的向心/切向加速度会被误认为底盘平移。只采用已转换的 yaw 角速度；不混用尚未核验的原始 gyro_x/gyro_y 构造三维姿态。

## 2026-09-25 数据依据与回放

- 静止基线：`tracked_static_verified_20260925_152840`，29894 帧；原始均值约 `[-7.890208, 0.017248, -5.919085] m/s²`。
- 前进方向：`tracked_motion_20260924_164557`，17 个符合直线条件的加减速窗口；统一时间窗口后的前进加速度与激光速度差分相关系数约 0.957。
- 拟合前进轴约 `[-0.600094, -0.002131, 0.799926]`，重力轴约 `[-0.799929, 0.001749, -0.600092]`。这是加速度数据的转换，不修改原有 yaw。
- 激光参考响应增益约 0.895，仅记录作诊断，不据此放大加速度。
- 本地隔离 ROS 域对五分钟静止 bag 做 4 倍速回放，仅启动两个 EKF 和加速度节点，没有启动底盘驱动。新 EKF 的 XY 最大偏移约 1.27 mm，末端偏移约 0.27 mm；yaw 波动范围约 0.078°，与不接加速度的基线一致。该 bag 同时用于静止基线标定，属于集成检查，不是独立精度验收。
- 受控运动 bag 回放：新旧融合轨迹最大 XY 差约 3.54 mm，yaw 一致，未出现新位置跳变。这同样不构成真实轨迹精度提升的证明。
- 独立的 `tracked_static_20260925_143153` 旧静止 bag（当时尚无静止保持）回放：校正后加速度均值约 `[0.00180, -0.00012, -0.00028] m/s²`；新旧 EKF 都跟随了约 13 cm 的激光最大偏移，两者最大 XY 差约 1.65 mm。说明加入加速度不会自动识别并修复错误的激光位置观测。

这些数据支持上位机平面加速度校正，并不构成新地图精度或长走廊精度验收。当前 RF2O 静止保持仍可能在特征退化的匀速直线运动中误判；本次没有把“加速度接近零”新增为强制停车依据，后续应结合独立运动证据及退化检测处理。

## 18 点附近走一圈后漂移

远程启动记录为 `2026-09-25-17-54-41-830660-a-M9-PRO-1210370`，17:58:58 停止。部署的履带建图配置仍为 `do_loop_closing: false`，安装配置指向源码同一文件。节点退出后无法再读取当时实际运行参数；日志没有回环事件，但默认日志级别也不能证明不存在所有匹配问题。不能仅凭回到起点后的截图判定是自动回环。

此次没有新 bag，普通日志没有同步保存 map→odom、EKF 位姿和扫描，因此无法区分局部扫描匹配错误、累计里程计误差、静止保持误判及运行时参数更改。RF2O 日志中的重复 logger 警告也不能单独证明存在第二套定位节点。

下一次测试启动后先保存实际参数，再录制下列话题；测试前后各静置 15 秒：

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
mkdir -p bags
run_dir="$HOME/ros2_ws/bags/tracked_inertial_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
ros2 param dump /slam_toolbox > "$run_dir/slam_runtime.yaml"
ros2 param dump /ekf_filter_node > "$run_dir/ekf_runtime.yaml"
ros2 param dump /tracked_imu_acceleration > "$run_dir/accel_runtime.yaml"
ros2 node list > "$run_dir/nodes.txt"
ros2 bag record -o "$run_dir/bag" \
  /scan /scan_slam /chassis/imu/data_raw /chassis/imu/acceleration \
  /chassis/odom/laser_raw /chassis/odom/laser /odometry/filtered \
  /tf /tf_static /map /rosout /parameter_events /cmd_vel
```

`/cmd_vel` 不代表遥控器实际输入。停止录制后再分析：检查 odom→base 是否先偏，再检查 map→odom 是否跳变，并结合原始扫描、静止保持日志和实际回环参数判断。惯性辅助是否改善整圈建图，必须用同步记录及独立测距确认。

## 18:29 最新实车记录复核

已将远程 `bags/tracked_20260925_182915`、对应 ROS 日志及部署参数保存到本地
`bags/remote_20260925_182915`，共 34 个文件，逐个比对远程和本地 SHA-256 一致。
bag 主文件为 190267392 字节。用户随后断开了外设，本次复核只读取本地数据，没有启动实车驱动。

该 bag 从 18:29:15 开始，包含 18:28:10 启动进程的尾段，以及 18:29:29 重新启动的建图过程。
分析运动和跳变时排除录制开始后前 20 秒，避免把节点重启/原点重置当成算法漂移。

### 是否接入

- 第二次启动日志有 `tracked_imu_acceleration` 进程（PID 1380922）；节点日志明确记录
  `Planar acceleration enabled=True`，输入 `/chassis/imu/data_raw`，输出 `/chassis/imu/acceleration`。
- 部署参数中的 EKF `imu1` 指向该输出，只有 ax/ay 参与融合；`imu0` 的 yaw 角速度转换保持原样。
- 节点随用户 Ctrl-C 正常退出，日志未发现该节点崩溃或配置错误。
- 本次 bag 没有记录校正后的加速度、参数快照和参数事件，因此不能由 bag 直接逐帧证明加速度接收率和运行中参数未被修改。已部署并启用的结论有日志/配置依据，逐帧融合状态仍缺失观测。
- 启动日志是 SLAM Toolbox 建图链，没有 Nav2/AMCL；本次观察的是建图定位漂移。

### 具体异常与限制

以下时间均相对 bag 开始时刻：

| 时间段 | IMU z 角速度积分 | RF2O 航向变化 | 解释 |
| --- | ---: | ---: | --- |
| 70–80 s | +0.19° | −6.25° | 两者出现约 6.4° 的相对航向分歧；不是独立真值精度结论 |
| 85–97 s | +211.76° | +209.95° | 明显快速大角度转向，IMU 峰值约 2.96 rad/s |

约 12 Hz 扫描下，2.96 rad/s 对应一帧间隔约 14° 的转动。该 bag 的 LaserScan
`scan_time`、`time_increment` 都为零，现有消息没有提供可直接用于逐束去畸变的有效时间信息。
这会限制快速转向时的处理能力，但不能仅据此量化全部地图误差。

约 91.54 s、93.47 s，`map→odom` 航向修正分别变化约 +4.2°、−6.0°。
这两次 TF 平移字段变化约 2.73 m、3.89 m，但它们包含远离 odom 原点时的旋转补偿，
**不能解读成机器人位置跳了这么多米**。将新旧 TF 作用于同一时刻的机器人 odom 位置，
估计的机器人地图位置修正约为 5 cm、9 cm；航向跳变对远处墙面仍可能造成明显错位。

按已部署标定参数重算本次原始 IMU，开头静止段的校正后侧向均值约 −0.024 m/s²，
后续静止保持段均值仍会变化；整段运动的校正加速度 1%/99% 分位在数 m/s² 量级。
这说明固定静止基线并没有消除跨时段残差、运动振动等影响，不能把它用于无约束的长期位置积分。

当前协方差适配器对激光平面位姿仍使用固定方差，缺少按方向识别退化和降低权重的机制。
EKF 使用 RF2O 的绝对 XY/yaw 及线速度，错误激光观测仍会主导融合；加速度辅助不等于解决了
狭窄环境的可观测性问题。部署配置仍关闭自动回环，本次没有证据支持“重新关闭回环即可修复”。

后续优先级是激光退化质量评估、错误观测降权/拒绝、静止保持的运动误判检查，以及扫描时序核验。
在这些问题处理并完成低速实车对比之前，不将当前地图和融合结果作为导航精度已验收的版本。
