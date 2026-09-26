# 履带建图卡顿与惯导可用性复核（2026-09-26）

**后续行驶复测更新：** 18:44–18:48 用户正常行驶再次出现暂停，暴露了静止及既有合成场景未覆盖的重置历史隔离缺陷，并记录到惯导运动超限和激光参考失配。见[本轮暂停复核](tracked_pause_review_20260926_184449.md)。以下 PASS 仅对应各自已执行场景，不代表真实行驶和走廊问题已全部解决。

**上一轮交付状态（不是最新行驶验收）：** 50 个相关文件已同步远程；本地和远程各 282 项检查通过；三个严格合成恢复场景通过；65 秒实机静止链路及独立回放显示核验通过。真实移动和走廊外部精度尚未验收。下文按排查先后保留失败记录，最终结果见文末。

## 本次现场证据能够说明什么

审查数据为 `bags/map_stutter_20260926_165031/bag`，ROS 采样日期为 2026-09-26，录包接收窗口约 16:50:32–16:50:52，共 19.731 秒、20387 条消息。`launch_before.log` 与 `launch_after.log` 内容和 SHA256 相同，各 15 行，仅记录 16:37:57 的进程启动，没有后续运行状态。不能把“进程已启动”视作惯导已经输出有效位置。

本录包窗口内 `/chassis/estimation/state` 的 374 条消息全部为 `VALID`，估计有效 374 条全部为 true；激光准备 432 条、建图门控 236 条全部为 true，没有状态切换。`/map` 11 条消息有 11 个不同时间戳。**本段没有捕获估计保护切换或建图停止更新，不能据此确认用户此前移动时卡顿的原因。**

本段确实暴露出独立惯导不可用：`/chassis/inertial/state` 3149 条全部 `INVALID`，valid 全为 false；3151 条 reason 全为 `position_uncertainty_limit`，`/chassis/odom/inertial` 没有消息。因此这里公共里程计始终来自 EKF，独立惯导没有接管过。

可信激光里程计的 xy 协方差最大特征值全程约 **12.51658 m²**，yaw 方差约 **0.01632864 rad²**，超过当前履带配置的惯导总位置方差上限 4.0 m²。该数据与 `position_uncertainty_limit` 一致，解释了惯导为何不能产生有效输出；须继续检查可信激光协方差累积、恢复下限继承和惯导准入条件。录包没有参数快照，实际运行阈值仍需与启动参数/远程版本一并核对。不能仅扩大阈值后宣布位置精度提高，也不能通过压小观测方差伪造可信度。

## 时间戳、链路延迟与消息间隔

延迟为 bag 接收时刻减消息 header 采样时刻。它包含处理及传输延迟，不等于算法纯运算时间。以下均来自原始时间戳：

| 数据流 | 帧数 | 最大采样间隔 | 接收延迟 p95 | 接收延迟最大值 |
| --- | ---: | ---: | ---: | ---: |
| `/scan` | 235 | 83.96 ms | 2.51 ms | 4.93 ms |
| `/chassis/odom/laser_raw` | 235 | 83.96 ms | 114.68 ms | 139.78 ms |
| `/chassis/odom/laser` | 235 | 83.96 ms | 116.09 ms | 141.72 ms |
| `/scan_slam` | 235 | 83.96 ms | 117.34 ms | 142.98 ms |
| `/chassis/imu/gyro` | 1956 | 20.68 ms | 3.89 ms | 8.75 ms |
| `/chassis/imu/acceleration` | 565 | 60.01 ms | 4.00 ms | 5.98 ms |
| 内部 EKF | 561 | 40.39 ms | 20.23 ms | 20.72 ms |
| 公共 `/odometry/filtered` | 561 | 40.39 ms | 21.69 ms | 24.66 ms |

`/scan`、RF2O 原始里程计、可信激光、`/scan_validated` 和 `/scan_slam` 各 235 帧，说明本段质量门控没有减少扫描数量；处理后的扫描比原始扫描多约 0.08–0.14 秒延迟。评估显示流畅度应同时看 TF 对采样时间的覆盖、处理延迟和 RViz 行为，不能把地图栅格更新频率直接等同于扫描频率。

`odom→base_footprint` 共 574 条：最大 200.11 ms 的 stamp 间隔和最大 964.43 ms 的接收延迟，均落在录包开始约 1.073 秒集中到达的一批旧时间戳附近。此后存在与 561 条公共里程计同 stamp、同位姿的 TF；这些值与录包发现初期收到旧消息相容，不能单凭全窗口最大值断言运行链路曾中断 0.96 秒。完整时间点保存在 JSON 的 `largest_stamp_gaps`，便于复核。

`map→odom` 有周期重复及未来时间戳，延迟中位数约 −74.9 ms；SLAM 发布的未来 TF 时间戳应结合实际 transform 配置解释，不能直接当作时间同步故障。

## 地图内容和位姿变化

本段地图约每 2 秒重新发布，11 个不同 stamp 对应的尺寸、分辨率、原点和栅格 CRC 只有 1 种。可信激光位姿、速度均保持不变，`/chassis/odom/stationary` 发布 235 条。记录表现为估计链静止保持，静止时不生成新地图内容是合理行为；但没有外部真值，不能仅凭估计保持判定机器人在物理上确实完全静止。

RF2O 原始 x/y/yaw 的变化范围为 2.662 cm / 1.562 cm / 0.00503 rad；公共 EKF x/y/yaw 的范围约 0.379 mm / 0.525 mm / 0.001078 rad。公共里程计与内部 EKF、同 stamp TF 的 561 对位姿完全一致。与可信激光插值比较，位置差 p95 约 0.397 mm，最大 0.738 mm。

这些是**连续性和源间一致性**，并不构成外部定位精度。位姿差与 Twist 积分的速度残差 p95 约 0.00876 m/s，属于同一估计内部自洽指标；它不能证明实际走过的距离正确。此现场没有独立真值，精度状态必须是 `NOT_EVALUATED`。

## 可重复分析工具

新工具 `tools/analyze_tracked_estimation.py` 只读 rosbag2，不启动底盘或发布 ROS 消息。依赖工作区已有 ROS 2 Python 消息支持、NumPy 和 PyYAML。现场复现命令：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
python3 tools/analyze_tracked_estimation.py \
  bags/map_stutter_20260926_165031/bag \
  --output bags/map_stutter_20260926_165031/estimation_review \
  --launch-log bags/map_stutter_20260926_165031/launch_before.log \
  --launch-log bags/map_stutter_20260926_165031/launch_after.log
```

工具生成 `.json` 和中文 `.md`：

- 话题/TF 数量、频率、采样间隔、接收延迟、重复/倒序 stamp，以及最大间隙出现时刻。
- 有效性开关、状态计数、按接收时间记录的切换区间。
- 位姿步长、位置/航向范围、位姿与速度积分残差、xy 协方差特征值和 yaw 方差。
- 同采样时刻的公共里程计与激光、惯导、内部 EKF、TF 一致性。
- 地图时间戳和栅格内容是否改变，WARN/ERROR 日志摘要及可选 launch 日志 SHA256。
- 独立外部真值精度单列；无真值时固定 `NOT_EVALUATED`，不输出精度 PASS。

已有现场输出位于 `bags/map_stutter_20260926_165031/estimation_review.{json,md}`。工具支持通过 rosbag2 插件读取 metadata 指定的存储格式；系统缺少对应存储插件或消息类型时，需安装相应 ROS 依赖。

## 外部真值输入约定

CSV 必须包含 `stamp,x,y,yaw`，单位依次为秒、米、米、弧度；位姿应表示目标里程计的同一车体参考点，使用完全相同的坐标系、轴向和原点。通过 `--truth-frame` 显式声明 frame。工具不估计旋转、平移、尺度或最佳时间偏移，不先移动真值让误差变小。示例格式如下，数值仅说明文件结构，不是现场测量：

```csv
stamp,x,y,yaw
1790412633.00,0.00,0.00,0.00
1790412633.10,0.01,0.00,0.00
```

```bash
python3 tools/analyze_tracked_estimation.py /绝对路径/bag \
  --groundtruth /绝对路径/independent_truth.csv \
  --truth-frame odom \
  --truth-time-basis ros \
  --truth-max-gap-s 0.2 \
  --output /绝对路径/review_with_truth
```

默认 CSV stamp 为 ROS epoch 秒；`--truth-time-basis bag-relative` 表示从 bag 首条消息接收时间起算的秒数。只有已知同步偏移时才使用 `--truth-time-offset-s`，计算为 CSV stamp 加该显式偏移，记录在报告中。超过指定插值间隔不外推，不覆盖的样本另计。yaw 按最短角度插值，避免跨 ±π 时误判。

有真值且帧/时间覆盖合格时报告 `EVALUATED`、位置/航向 RMSE 和分位数，仍不自动给出 PASS；验收门槛必须由独立任务需求确定。frame 不匹配、无覆盖或无真值时为 `NOT_EVALUATED`。用 EKF 自身导出的 CSV 不能当作独立真值。

## 工具验证与后续实车取证

工具已对本次现场 bag 完整运行并通过 Python 编译检查。边界验证包含：保留人为 1 米固定偏差、不自动对齐 5 秒时间差、frame 不一致拒绝评估、跨大时间间隔不插值、重复真值 stamp 拒绝、无真值不评分、yaw 跨 π 插值。结果记录于 `bags/map_stutter_20260926_165031/tool_validation.json`。

修复后应先确认惯导节点实际产生有效消息，并在激光暂时无效时观察公共输出是否按预期进入 `INERTIAL`，而非只确认节点名称存在。移动卡顿复测需要完整覆盖“稳定行驶→卡顿→恢复”以及用户实际动作；录制 `tools/record_incident.sh` 的全部话题，保留运行参数和节点日志。连续性通过、地图可更新、源间位姿接近与外部精度通过是不同验收项，应分别报告。

## 增强恢复回归的验收口径

`tools/test_tracked_recovery.py` 已增加以下回归检查，使用真实六节点链路和可审查的合成输入。阈值只是软件场景的工程验收要求，不代表实车精度：

- 启动后必须同时收到有效惯导状态和独立惯导消息；不能只看到 EKF/地图就认为惯导接入成功。
- 正常起步不允许进入 `PAUSING/PAUSED/RESETTING/ENABLING/WARMING`，不允许调用 SetPose；`/scan_slam` 最大接收断档不超过 0.65 秒。
- 0.5 秒短断流必须进入 `INERTIAL` 且保留有效公共定位；恢复过程不允许失效/重置，扫描恢复延迟不超过 0.85 秒。
- 通过透明 SetPose 服务代理转发真实 EKF 请求并记录全部调用时刻，避免状态采样漏掉快速 reset。
- 持续有效期间，公共位姿增量与体坐标 Twist 的积分必须自洽。位置残差阈值为 `0.015 m + 0.35 m/s × dt`，yaw 为 `0.025 rad + 0.30 rad/s × dt`；会拒绝约 7 cm / 30 ms 的跳变。明确失效后的重新初始化单独记录，不伪装成连续区间。
- 保存 `truth.csv`、`truth.json` 和完整 `observations.json`，包括 `stamp,x,y,yaw,vx,vy,wz`；全程按源时间戳和同一 odom 坐标比较，不自动对齐坐标、比例或时间。位置/yaw 的 p95 工程阈值分别为 0.15 m / 0.12 rad，同时报告每个源的速度和角速度误差。
- `--laser-xy-variance 12.5166` 显式重现实测大协方差锚点，必须真的输出惯导并完成接管；不能只把阈值改大却未触发回归条件。
- 可选旋转和不同到达延迟覆盖异步融合：gyro 约 100 Hz、acceleration 约 30 Hz、扫描约 12 Hz，采集 stamp 保留，延迟仅影响消息到达。

```bash
source /opt/ros/humble/setup.bash
python3 tools/test_tracked_recovery.py --domain-id 94 \
  --output-dir bags/repair_validation/recovery_strict_default
python3 tools/test_tracked_recovery.py --domain-id 95 \
  --laser-xy-variance 12.5166 \
  --output-dir bags/repair_validation/recovery_strict_high_covariance
python3 tools/test_tracked_recovery.py --domain-id 96 \
  --turn-rate 0.10 --laser-delay-s 0.08 \
  --gyro-delay-s 0.01 --acceleration-delay-s 0.025 \
  --output-dir bags/repair_validation/recovery_strict_delayed_turn
```

截至本节更新，脚本语法、帮助入口及七项判定函数验证已通过；验证包含跳变拒绝、持续正常运动放行、失效重置隔离、体坐标速度旋转、保留固定坐标误差、yaw 跨 π 插值和大间隔不插值。证据为 `bags/repair_validation/recovery_acceptance_checks_20260926.json`。**增强后的六节点集成结果尚待稳定代码完成后的实际运行，不沿用此前宽松版本的 PASS。**


### 严格集成首次失败记录

首轮默认场景证据保留在 bags/repair_validation/map_stutter_20260926/default_before_final_guard/，没有覆盖。短断流恢复时 valid 在 t=9.1057–9.2047 秒出现约 99 ms 的 false，状态仍为 INERTIAL，没有 SetPose 调用。新激光样本先到、独立 INS 仍短暂落后该激光 stamp 时，准入判断存在异步恢复的错误失效脉冲。该轮最终为 FAIL。

同轮正常起步扫描最大间隔约 89.5 ms、短断流恢复扫描约 720 ms、持续有效区间位姿/Twist 跳变检查和全轨迹同 stamp 真值检查通过；这些局部通过不能抵消有效性检查失败。最终保护代码需要修复并重新运行默认、大协方差、延迟转向三个场景；严格阈值保持不变。

### 新鲜隔离域三场景结果（最终调优前）

原始记录分别位于 bags/repair_validation/map_stutter_20260926/default_final、high_covariance_final、delayed_turn_final。三次源码 SHA256 在运行期间均未改变。首次重复域 94 的两次启动因接收新鲜度异常未通过，证据仍分别保留 default_before_cov_baseline 和 default_readiness_failure；第二次还有配置并发变化，不能当最终版本验收。换用全新域 97/98/99 后完成所有阶段，尚未证明旧域失败的确切中间件原因。

| 场景 | 结果 | 公共位置 p95 / 最大 | 正常扫描最大间隔 | 短恢复扫描延迟 |
| --- | --- | --- | --- | --- |
| default_final | PASS | 0.05889 / 0.06584 m | 0.09164 s | 0.63147 s |
| high_covariance_final | FAIL | 0.17070 / 0.18109 m | 0.09074 s | 0.72142 s |
| delayed_turn_final | FAIL | 0.06405 / 0.07101 m | 0.10052 s | 1.08663 s |

三场景短恢复均没有 SetPose，公共定位 valid 连续，位姿/Twist 连续性检查没有超限；默认场景全部通过。大协方差场景的位置 p95 为 0.17070 m，超过固定 0.15 m 工程阈值；延迟转向恢复扫描耗时 1.08663 s，超过固定 0.85 s 工程阈值。两项仍为 FAIL，不能由局部链路通过推断整体验收通过。

大协方差分段证据：正常起步公共速度误差 p50 为 0.07934 m/s（真速 0.12 m/s），位置 p95 为 0.14868 m；短断流时独立 INS 本体位置误差约 0.000072 m，但公共输出为了连续继承了 EKF 偏置，误差约 0.16754 m。配置的完整激光 odom0 只融合位姿，不融合速度；强静止零速度约束以及偏大的位置协方差需一起核对，不能通过缩小继承协方差伪造可信度。

延迟转向分段证据：短恢复从 t=8.5843 s 开始；laser_ready 在 8.9501 s 为 true、9.0392 s 又为 false、9.3114 s 再为 true，guard 在 9.6607 s 返回 VALID，9.6710 s 扫描门打开。中途一次激光准备失效追加了恢复确认时间。

以上均为本地合成输入结果，尚未填写或声称远程新版本部署、实机运动或真实走廊精度通过。

### 冻结版本第二轮默认场景失败（隔离域 81）

证据保留在 `bags/repair_validation/map_stutter_20260926/default_verified/`。本次使用未运行过的新隔离域 81；脚本内 SHA256 检查通过，额外核对包含 `laser_quality.py`、`reference_recovery.py` 和平台解析器在内的 13 个源码/配置文件也没有变化，摘要保存在 `verified_source_hashes.json`。

本次整体为 **FAIL**：0.5 秒短断流期间没有惯导消息、没有进入 `INERTIAL`，保护节点转入 `PAUSING/PAUSED`；短恢复调用 SetPose 两次，扫描恢复延迟 1.3451 秒，超过 0.85 秒固定阈值。长断流末尾惯导无效原因是 `imu_callback_stale`，不符合预期的有限预测到期。正常段扫描最大间隔 0.0931 秒、公共位置真值 p95 0.06273 米，以及位姿/Twist 连续性检查通过，不能抵消以上失败。

准备信号的合成发布记录与回归节点订阅 `/chassis/imu/data_ready` 都是 175 帧，最大间隔约 0.111 秒；独立惯导却反复出现 `imu_data_not_ready_or_stale`、`imu_callback_stale` 和 `imu_gap_requires_new_anchor`。惯导总输出仅 70 帧，最大间隔 5.128 秒；内部 EKF 最大输出间隔为 0.0422 秒。证据表明当前失败直接关联惯导节点准备信号接收/新鲜度链路，不能归因于测试输入主动停发，也尚未证明具体中间件或执行器根因。已暂停后两场景，不通过放宽阈值或重复盲跑掩盖该问题；后续修复须使用新目录重新验收。

### 准备信号读端对照与正式复测

临时仪表对照记录位于 `bags/repair_validation/map_stutter_20260926/ready_instrumentation/summary.json`。原准备信号发布器保持 RELIABLE、TRANSIENT_LOCAL、depth 1，输入持续约 10 Hz；个别内部读端却有约 2 秒的回调间隙，同节点其它高频输入仍正常，已测回调和 publish 耗时均为毫秒级。只把 INS/估计保护的实时准备信号读端改为 RELIABLE、VOLATILE、depth 10，在同一隔离域 81 的两次对照均恢复完整心跳并通过回归。新鲜度、运动、精度和恢复阈值未放宽；发布器仍保留供晚加入工具读取的锁存状态。该对照支持应用层 QoS 修复，不等于已经定位某个确定的 Fast DDS 内部缺陷。

正式版本在不加载诊断 `sitecustomize`、无 `KIWI_READY_*` 环境变量下依次运行，源码/配置 13 项摘要在各次运行期间保持不变，记录为 `verified_after_ready_fix_source_hashes.json`。结果目录均位于 `bags/repair_validation/map_stutter_20260926/`：

| 目录 / 隔离域 | 结果 | 公共位置 p95 / 最大 | 正常扫描最大间隔 | 短恢复扫描延迟 |
| --- | --- | --- | --- | --- |
| `default_verified_after_ready_fix` / 81 | PASS | 0.05884 / 0.06517 m | 0.09110 s | 0.72123 s |
| `high_covariance_verified_after_ready_fix` / 82 | PASS | 0.12978 / 0.16254 m | 0.09139 s | 0.63255 s |
| `delayed_turn_verified_after_ready_fix` / 83 | FAIL | 未进入运动验收阶段 | 未进入运动验收阶段 | 未进入运动验收阶段 |

默认与 12.5166 m² 大协方差场景分别通过全部 17 / 18 项固定检查；短断流均进入 INERTIAL，保持有效且没有 SetPose，长断流仍在有限预测超限后明确失效并核验旧参考后恢复。大协方差场景的最大位置误差 0.16254 米如实保留，固定验收项使用 p95 0.15 米，不能把 PASS 解释为每个样本均低于 0.15 米。

延迟转向场景在启动 25 秒内未得到 `/scan_slam` 或 `/map`，因此 `execution_completed` 失败，不能把启动窗口的静止真值比较当作运动精度通过。准备输入和回归订阅各 231 帧、最大间隔 0.111 秒；INS 已不再出现准备信号/IMU 回调过期，而是因为可信激光不断间断进入 `coast_timeout`。`laser_ready` 多次在约 0.2–0.7 秒 true 后又保持约 1–2 秒 false，适配器重复核验旧参考并回到 settling。适配器内部的标定准备信号读端仍是 TRANSIENT_LOCAL depth 1，是否同类交付问题尚在仪表验证；此处只记录已观察的故障，不作已修复结论。

### 最终冻结版本：三场景严格集成验收通过

2026-09-26 完成最终正式复测。INS/估计保护的实时准备信号读端具有上述同域对照证据；适配器、建图门控中要求持续新鲜心跳的履带读端随后统一为相同语义。后两者没有直接抓到迟达回调，不能进一步声称已证明某一中间件缺陷。所有状态发布器仍锁存；全向底盘一次性标定状态的订阅兼容性保留，未修改任何验收阈值。

再次使用原隔离域 81 / 82 / 83，未加载诊断 `sitecustomize` 或 `KIWI_READY_*` 环境，最终 13 项源码/配置摘要在三次运行前后均相同，记录在 `bags/repair_validation/map_stutter_20260926/after_all_ready_fix_source_hashes.json`。每次脚本自己的运行期摘要检查也通过。完整结果、各节点日志、观测与合成真值仍分别保留；汇总为同目录 `strict_final_summary.json`。

| 最终目录 / 隔离域 | 固定检查 | 公共位置 p95 / 最大 | yaw p95 | 正常扫描最大间隔 | 短恢复扫描延迟 |
| --- | --- | --- | --- | --- | --- |
| `default_after_all_ready_fix` / 81 | PASS，17 / 17 | 0.05950 / 0.06645 m | 0.0000411 rad | 0.09112 s | 0.72602 s |
| `high_covariance_after_all_ready_fix` / 82 | PASS，18 / 18 | 0.13170 / 0.15689 m | 0.0000509 rad | 0.09145 s | 0.72241 s |
| `delayed_turn_after_all_ready_fix` / 83 | PASS，17 / 17 | 0.06385 / 0.07058 m | 0.00594 rad | 0.09992 s | 0.72672 s |

三个场景均启动并实际输出独立惯导；0.5 秒激光中断时进入 INERTIAL，公共有效性保持，短恢复没有 SetPose 或暂停重置，位姿/Twist 连续性超限次数均为 0。短断流公共里程计与 TF 的最大采样间隔分别为 **39.94 / 40.09 / 41.20 毫秒**。2.7 秒长断流均在有限预测边界达到后返回 `coast_timeout`，再经旧参考核验与明确失效后的重新初始化恢复建图。正常起步无重置，短恢复后的地图时间戳实际继续更新。

大协方差场景使用 `--laser-xy-variance 12.5166`，运行中激光 xy 最大特征值最高约 15.3768 m²，未通过压小继承方差制造可信度。其最大位置误差 0.15689 米仍按实值报告；固定位置验收项是 p95 ≤ 0.15 米，不能声称每个样本都小于 0.15 米。延迟转向使用角速度 0.10 rad/s、激光到达延迟 0.08 秒、gyro 0.01 秒、加速度 0.025 秒，采集时间戳和真值均保留，未自动调整坐标、尺度或时间差。

本节只证明最终软件版本在指定合成输入下的连续性、短时续接、受限失效和恢复，以及相对于合成真值的误差符合固定工程阈值。它不构成远程实车走廊定位精度验收；现场仍须使用独立距离/姿态真值，并覆盖完整的正常运动、退化、恢复过程，不能以三种估计彼此接近替代真实精度。此前失败目录均保留，未覆盖或改写为 PASS。


## 最终部署与实机静止验证

2026-09-26 18:35 完成冻结代码部署，按显式清单核验本地与远程 50 个文件 SHA256，29 个文件内容需要更新。原文件备份在远程 /home/a/ros2_ws/bags/stutter_deployment_backup_20260926_183555，清单保存在本地 bags/repair_validation/map_stutter_20260926/deployment_manifest.json。未覆盖网页包改动，也未改写实测底盘速度倍率、IMU 轴向或安装位姿。

本地、远程分别构建 kiwi_base、kiwi_platform、kiwi_slam、kiwi_navigation 四包，均成功；两端 colcon test-result 均为 80 + 28 + 156 + 18 = **282 项检查，0 errors、0 failures、0 skipped**。日志分别为 local_build_accepted.log、local_tests_accepted.log 和 remote_build_test.log，均位于上述本地验证目录。全向校准状态只在变化时发布的兼容性已有构造测试覆盖。

18:36:58 起执行 65 秒远程静止测试。使用临时复制的底盘 YAML，将 enable_motion、keyboard_trial_enabled、open_loop_test_enabled 全部关闭，未发送非零运动指令；串口仍发送中性电机帧。未启动 Nav2。测试证据目录：

- 远程：/home/a/ros2_ws/bags/repair_validation/stutter_stationary_20260926_183658。
- 本地副本：bags/repair_validation/map_stutter_20260926/remote_stationary，含 bag、bringup.log、result.json、observations.json 和只读分析 estimation_review.json/md。

本轮开机 IMU 残余标定在录包相对约 13.0 秒完成，997 个样本，残余 z 轴陀螺偏置约 **0.0001831 rad/s**；约 13.9 秒公共定位进入 VALID，约 14.9 秒建图门打开，之后没有回到失效状态。此数值是本次静止采样结果，不写成跨开机通用标定常量。

最后 30 秒稳态结果：

| 检查 | 实测 | 结果 |
| --- | --- | --- |
| 公共定位、IMU 标定、INS 有效、建图门控 | 全部持续有效 | PASS |
| /odometry/filtered | 900 帧，30.0 Hz，最大 stamp 间隔 40.9 ms | PASS |
| /chassis/odom/inertial | 855 帧，28.5 Hz，最大 stamp 间隔 60.0 ms | PASS |
| /scan_slam | 361 帧，约 12.0 Hz，最大 stamp 间隔 83.8 ms | PASS |
| /map | 60 帧，2.0 Hz，最大 stamp 间隔 584.3 ms | PASS |
| RViz、启动器、录包退出 | 收到停止信号后正常结束，无强杀 | PASS |

稳态公共里程计 x/y 峰峰变化约 **0.467 / 0.354 mm**，独立 INS 约 **2.251 / 1.929 mm**。这是保持静止条件下本次输出波动，**不是实车绝对定位精度或长时间零漂保证**。整包 98 条地图有 98 个不同 stamp，但仅 1 种栅格内容，与静止无新增地图信息相符；不能把地图 CRC 不变当作建图卡住。SLAM 自动回环保持 do_loop_closing: true。

启动未就绪期间 monitor 如实报告尚无里程计/地图，门控随后正常放行。日志还保留首次订阅 /scan_slam 的 QoS 提示、SLAM 最小量程提示，以及首次地图纹理创建时的一条 RViz GLSL 链接告警；录包实际收到全部 593 帧 /scan_slam。因此这里不声称“日志完全无告警”，也不将正常退出单独当成图形显示验收。

本次实机验证覆盖启动、静止有效性、消息/TF 连续性和地图周期更新；**没有进行真实运动、长走廊退化或闭环回到起点的外部真值验收**。INS 仍受 2 秒续接和协方差增长预算限制。EKF、激光、INS 共享数据，三条轨迹接近不能替代独立测量；分析结果的外部精度保持 NOT_EVALUATED。原失败记录全部保留，未通过扩大验收阈值或缩小实测协方差取得 PASS。

### RViz 显示单独核验

另在远程隔离域 84 回放本次 bag 的地图、扫描、TF 和模型话题，未启动底盘驱动，也未回放运动命令。窗口截图 rviz_replay.png 已保存到本地与远程上述静止证据目录：地图栅格和机器人模型正常可见，窗口 Global Status 为 Ok，截图时显示 31 fps。回放仍有首张地图纹理创建时的一次相同 GLSL 告警，随后可见地图正常渲染；未为此修改系统 RViz、显卡驱动或融合参数。首次根窗口截图不受当前 X 服务支持，改用 RViz 窗口级 XWD 截图完成核验；截图失败不等同于 RViz 卡死。回放和静止测试的自有进程均已停止。
