# 履带惯性续接与建图恢复（2026-09-26）

## 问题与目标

2026-09-26 15:16:45 的故障发生后，雷达、IMU 与 RF2O 原始输出继续存在，但扫描质量适配器一直拿新扫描与失效前的旧参考比较，持续返回低重叠。融合保护器随后停用 EKF，公共里程计、TF、建图扫描都停止。SLAM Toolbox 仍重发旧栅格，原监视器只检查接收时间，误报 `map valid`。现场依据见 `documents/live_mapping_stall_20260926_151645.md`。

本次改动要解决两个不同问题：短时激光观测缺失时保留有限的定位连续性；激光重新可靠后经过核验恢复地图更新。惯导输出新位姿并不意味着扫描可靠，也不意味着已经获得绝对位置修正。

## 惯性续接边界

独立节点 `tracked_inertial_odometry` 使用已完成轴向、倍率和启动残余偏置标定的 IMU，从最后可信激光的位置、航向和多帧拟合速度出发，积分校正后的平面加速度和角速度。无编码器条件下，没有独立的绝对 xy 观测；预测过程中持续传播协方差，并检查时间、数据新鲜度和状态合理性。当前 `kiwi_base/config/tracked_pwm.yaml` 配置最长续接 2 秒；独立输出 `/chassis/odom/inertial` 不反灌 EKF，不单独发布 TF。

现场健康扫描的动态 xy 协方差最大特征值约为 1.45–2.81 m²，所以惯导必须继承锚点已有的不确定度，不能人为压低。当前总位置方差上限 `max_position_variance=4.0` m²，新增位置方差上限 `max_position_growth_variance=0.09` m²；两者与 2 秒期限共同生效。新增方差门槛对应额外标准差 0.30 m，但这只是模型内部保护量，不能作为实车定位误差承诺。

2 秒是允许尝试恢复的工程时间上限，不是保证误差的指标。加速度残余偏置、姿态误差、振动和倾斜都会产生位移误差；匀速通过长直走廊时，单个 IMU 不能补足缺失的绝对平移约束。超过可信时间或协方差限值必须标为定位无效，不能靠不断重置计时器无限续接。

短时激光缺失且惯导有效时，保护节点选择惯导续接公共里程计和唯一 `odom→base_footprint` TF，状态为 `INERTIAL`，EKF 保持运行。`/chassis/estimation/valid=true` 表示当前短时定位仍处于允许范围；它不等同于 `/kiwi_slam/mapping_gate_ready=true`。质量不合格的扫描不能入图，恢复阶段也必须等估计状态回到 `VALID` 才放行 `/scan_slam`。IMU 数据失效、惯导超时或方差越界后停止公共有效输出，并暂停无约束的 EKF 预测。

## 激光恢复原则

距最后可信扫描 2 秒内，恢复可用旧参考几何匹配，或用连续局部扫描验证有界惯性运动；通常至少 3 帧且持续 0.2 秒通过。惯性桥必须同时满足新扫描几何约束、重叠支持率和多个连续候选位姿与惯性预测的一致性。成功后继续使用已验证的 odom 坐标参考，不得把当前扫描置零并伪装为机器人从未移动，也不得将低质量扫描直接用于更新旧地图。

距最后可信扫描 2–10 秒时，惯导不再提供有效续接，只允许通过旧参考的强几何匹配恢复：至少 5 个连续候选帧、持续 0.4 秒，且核验重叠支持、匹配残差、几何条件和运动一致性。超过 10 秒报告 `reference_expired_restart_required`，此时需要可靠重定位或重新建图；实现没有声称自动找回任意旧地图位置。单凭新扫描彼此能匹配，不能证明其在旧地图中的绝对位置。

激光恢复后持续健康 0.3 秒，在原 odom 坐标内设置 EKF 位姿并确认服务完成，再经过预热和输出一致性核验。异步服务按实际操作记录完成结果，等待 EKF 输出也有超时；恢复中的晚到响应不能绕过当前数据有效性检查。切换公共来源时要求时间戳严格递增。

底盘选择仍只修改 `kiwi_platform/config/active_platform.yaml`，履带驱动、校准、EKF、惯导和保护参数统一在 `kiwi_base/config/tracked_pwm.yaml`。各底盘使用自己的雷达里程计、建图和导航参数；切回 `kiwi` 后沿用轮式链路，不启动履带惯导/保护节点，不需要新增启动入口。

## 地图监控的实际语义

监视器对动态地图分别输出 `age_sec`（接收新消息后经过的时间）、`header_age_sec`（地图时间戳距 ROS 当前时间）、`stamp_unchanged_sec`（地图时间戳多久没有改变）。周期重发旧地图只会更新第一项。

履带建图入口可通过 `mapping_gate_ready_topic`、`estimation_state_topic` 启用状态诊断；两个参数默认为空，不给全向底盘增加履带话题依赖。`INERTIAL` 时报告 `mapping paused: inertial prediction only`；`PAUSED` 等无效状态报告建图因估计暂停；状态消息超时也不能报健康。门控未放行时明确报告暂停，不用 `/map` 的接收频率证明正在建图。

机器人静止时，SLAM Toolbox 可能不产生新关键帧，所以地图时间戳旧不能单独判错。上游状态、里程计仍健康时允许旧地图；旧时间戳持续且里程计失联时报告上游停更。`map_is_static=true` 的地图服务器模式不要求更新地图或接收建图门控心跳。

## 验证记录

- 监视器回归测试：`src/kiwi_slam/test/test_monitor.py`，13 项通过，包含旧地图重发、估计暂停、惯性续接、状态超时、静止关键帧不变、静态地图免门控与 TF 过期检查。
- 本地和远程均完成 `kiwi_base`、`kiwi_platform`、`kiwi_slam`、`kiwi_navigation` 的 colcon 构建与测试；分别 80、22、107、18 项，共 227 项通过。本次验证未启动实车外设。
- 本地、远程分别以六个真实 ROS 节点运行合成数据集成回归，输入 gyro 100 Hz、加速度 30 Hz、扫描 12 Hz。包含启动、真实运动模型、0.5 秒缺扫描、恢复、2.7 秒缺扫描和强旧参考恢复；两次均 `passed=true`。

| 合成回归指标 | 本地 | 远程上位机 |
| --- | ---: | ---: |
| 0.5 秒缺扫描期间公共 odom/TF 最大间隔 | 0.04031 s | 0.04488 s |
| 同期内部 EKF 输出最大间隔 | 0.04031 s | 0.04135 s |
| 公共 odom / TF 消息数 | 各 394 | 各 394 |
| 放行建图扫描 / 地图消息数 | 99 / 7 | 98 / 7 |
| 最终真值 x / 估计 x | 1.61748 / 1.57032 m | 1.61715 / 1.56611 m |
| 最终 x 偏差 | 4.72 cm | 5.10 cm |

0.5 秒中断期间公共位姿和内部 EKF 保持更新，坏扫描暂停入图；2.7 秒中断超过惯导期限后出现无效/暂停状态，随后强旧参考核验成功、EKF 恢复到 `VALID`、地图重新更新。表中厘米偏差仅反映指定合成运动与场景，不能用于宣称真实走廊的定位精度。

本地证据：`bags/repair_validation/recovery_integration_20260926_local/result.json` 及同目录节点日志；远程结果已下载至 `bags/repair_validation/inertial_recovery_20260926/remote_integration/result.json`。远程原目录为 `/home/a/ros2_ws/bags/repair_validation/inertial_recovery_20260926_remote`。

## 操作与后续实车验收

上位机仍使用原建图入口，启动后在特征充分处静置完成校准：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

另起相同环境的终端查询状态：

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
ros2 topic echo /chassis/estimation/state std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /chassis/inertial/reason std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /kiwi_slam/mapping_gate_ready std_msgs/msg/Bool --once --qos-durability transient_local
bash tools/record_incident.sh
```

采集脚本复用正在运行的系统，记录全部话题和日志，默认按 Ctrl+C 结束；短故障记录可使用 `--duration 10`。不再手写固定话题列表，避免遗漏新惯导和估计状态。

验收应覆盖静止有人经过、短时扫描中断后恢复、真实低速运动不被静止保持锁住、长时间退化超过续接上限、恢复位姿与旧参考不一致，以及切回全向底盘。
