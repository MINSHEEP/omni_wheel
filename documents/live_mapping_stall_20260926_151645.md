# 履带建图停更现场复核（2026-09-26 15:16:45）

## 现场现象及时间线

远程上位机 `a@192.168.66.112`，ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1；15:13:49 启动 `kiwi_slam.launch.py use_rviz:=true`。调查期间保持该进程运行，未发送底盘运动指令、未停止节点。

- 15:16:45 左右，`rf2o_covariance_adapter` 的扫描质量从 `moving` 转 `background_mismatch`，继而 `low_background_support`、`low_overlap`；激光准备状态变为 `false`。
- 15:16:45.479，`tracked_estimation_guard` 进入 `PAUSING (laser_not_ready)`，15:16:45.529 进入 `PAUSED`，停用 EKF 处理及公共里程计/TF 输出。
- 随后仍有 `/scan` 约 12 Hz、RF2O 原始里程计约 11–12 Hz、IMU 原始数据和 gyro。IMU 的 `calibration_ready`、`data_ready` 均为 true；但 `/chassis/odom/laser`、`/scan_validated`、`/scan_slam`、`/odometry/filtered` 均不再发帧。
- `/map` 约 0.5 Hz 重发旧栅格，其 header.stamp 固定 `1790407005.2581306`（15:16:45），现场 15 秒包开始时已滞后 268 秒。监控仅按接收时间标为 map valid，不能表示建图内容仍在更新。`odom→base`、`map→odom` TF 同样变旧。
- RViz PID 1801988 在现场仍存活，约 10% CPU、183 MB RSS；15:19:43 GDB 抓到主线程在 Qt/GLib 事件循环 31 ms `poll`，无当时的 GUI 线程死锁证据。画面停更与上游估计/扫描暂停一致。

## 数据验证

现场 15 秒 bag：`bags/live_corridor_20260926_152113/`，14.74 s、7062 消息。`/scan` 177、原始 RF2O 166、校准后平面加速度 380，可信激光里程计、建图扫描、公共融合里程计各为 0；`/map` 8 条但所有 header.stamp 相同。原始 RF2O 在该 15 秒中 x/y 变化范围约 3.2/1.4 cm，无法说明故障发生时机器人实际位移。

按当前 `laser_quality.align()` 算法离线匹配该 bag 的 176 对相邻扫描：176 对全部通过，支持率最小 0.91、信息比最小 0.084（配置门槛 0.015）。现场质量适配器仍对**故障前最后可信参考**报低重叠。故障瞬间的扫描未收进本 bag，不能仅凭后续样本判定瞬间由窄道几何、快速转向、遮挡或其他环境变化触发；故障后的长期无法恢复已由旧参考失配解释。

15:16:15 与 15:16:20，guard 曾两次因 EKF 与激光位置差 0.253–0.257 m（阈值 0.25 m）拒绝输出，其中一次 EKF 速度 0.687 m/s，超过底盘配置的 0.30 m/s 最大线速，说明失效前融合运动估计也需复核。

## IMU 为什么没有修正 xy

当前 EKF 的 `odom0_config` 仅接收激光 x/y/yaw，`imu0_config` 接收 gyro_z，`imu1_config` 接收平面 ax/ay。IMU 没有绝对 x/y 或可长期可靠积分的速度观测。激光质量失败后，`tracked_estimation_guard` 依据 `max_prediction_s=0.2` 暂停公共输出并通过 `/chassis/ekf/toggle` 停用 EKF；这是为了避免无观测积分发散。此前称“接入惯导”准确说是 IMU 惯性观测辅助，**尚不具备独立、长期的 xy 惯性导航**。

本次后段近静止样本的平面加速度均值约 `[0.0183, 0.0096] m/s²`，15 秒内原始 RF2O 变化数厘米。若将该 x 向残余量直接积分 10 秒，单是常值项就产生约 0.92 m 位移误差；运动振动和姿态误差还会增大不确定度。这解释了为何不能简单打开 IMU 双重积分让车走过长窄道。

## 恢复边界和后续工作

`ScanEvidence` 在失配时保留最后可信参考，连续低重叠时不会自动把新扫描当作新原点。`ReferenceRecovery` 只核验短间隔（当前 2 秒）的旧参考；数分钟后直接重锚会伪造连续里程计，因此当前运行不能靠 IMU 自行恢复原 map/odom 对齐。

要长期穿过弱特征通道，需要独立平移观测（驱动编码器、光流/视觉里程计或其他定位源）；未测速的电机 PWM 和 `/cmd_vel` 只能作为短时弱预测，必须限制时间、随时间增大协方差并在失效时停车。离开通道后需要将扫描与已有地图重新定位，并核验多帧一致性，才能重置 EKF/odom；不能单纯放宽激光门槛或把新扫描直接置零续接。

现场已保存部分地图和 SLAM 位姿图到远程 `/home/a/ros2_ws/maps/diagnostic_partial_20260926_1524.{yaml,pgm,posegraph,data}`；副本已下载到本地 `bags/live_corridor_20260926_152113/`。这是故障前地图，未经新路径完整性验收，不应用于自动导航。
