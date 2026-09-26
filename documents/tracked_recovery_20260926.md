# 履带中断恢复与估计失效保护：完整修复记录

## 交付范围

针对 `incident_20260926_105811_529872` 中的 332 ms IMU 间隔、RF2O 处理延迟、
一次约 6.5 cm 的背景匹配冲突以及随后的 EKF 发散，完成 IMU 恢复、激光参考核验、
融合输出保护及建图门控联动。复用 `kiwi_slam`，没有新增功能包；Kiwi 轮式链路保持
自己的 EKF/TF 发布方式。驱动协议、yaw 符号/倍率、速度换算、机身和安装参数未改变。

## 状态与参数

| 配置/状态 | 含义 |
|---|---|
| `/chassis/imu/calibration_ready` | 本次启动偏置标定有效；短时中断不会将其撤销 |
| `/chassis/imu/data_ready` | 当前 IMU 连续数据可用于融合；中断时为 false |
| IMU `recovery_duration_s=0.3` | 清空跨间隔处理缓存后，至少 0.3 秒且 15 帧连续样本才能恢复；积压数据突发到达不算立即恢复 |
| IMU `max_interruption_s=2.0` | 超过该间隔，或时间倒退/数值、坐标无效，明确失效并要求重新启动校准 |
| 激光 `recovery_max_gap_s=2.0` | 使用最后可信扫描保留的参考，核对中断期间累计平移和转角；参考过期不伪造恢复 |
| 激光 `recovery_duration_s=0.2` | 至少 3 帧且持续 0.2 秒核验通过，才继续原 odom 坐标；不把新位置归零 |
| 保护层 `max_prediction_s=0.2` | 没有新的有效观测超过该接收间隔就关闭公共输出；显式状态失效立即参与关闭判定 |
| `max_measurement_age_s=0.35` | 消息头的总测量延迟上限，包含采样、计算和传输延迟，与接收中断判据分开 |
| `/chassis/estimation/valid` | 公共估计有效性心跳；消费端同时检查心跳新鲜度 |
| `/chassis/estimation/state` | PAUSING / PAUSED / RESETTING / ENABLING / WARMING / VALID |

IMU、EKF、保护节点参数均在 `src/kiwi_base/config/tracked_pwm.yaml`。
激光恢复参数在 `src/kiwi_slam/config/tracked_rf2o.yaml`。
`tracked_mapping_gate.yaml` 必须同时检查激光质量和估计有效状态；短时恢复不重新要求
机器人停车做启动校准。未知运动和无法核验的参考仍然拒绝输出，不能将冻结显示解释为
机器人实际静止。

## EKF 输出与 TF 的处理

履带 EKF 设置 `publish_tf=false`，输出重映射到
`/chassis/odometry/filtered_unchecked`。该消息仅用于诊断，不允许作为导航输入。
`tracked_estimation_guard` 核验后才发布 `/odometry/filtered` 和唯一
`odom → base_footprint` 动态 TF。

失效时关闭公共输出，同时调用 EKF toggle 服务暂停内部积分。恢复时先等 IMU 数据、
旧激光参考和连续健康窗口通过，再用已核验的原 odom 坐标下激光位姿调用 set_pose，
清除旧速度/加速度和积压观测，恢复处理并核对输出，最后开放公共消息与 TF。
建图门控订阅 `/chassis/estimation/valid`，不能用刚失效时缓存的旧里程计继续放行。

上游 toggle 在关闭处理时仍可能发布状态，因此不能只调用服务而保留直接 TF。
这是单独保护公共输出的原因。[robot_localization Humble 实现](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ros_filter.cpp)

## 验证结果

### 完整现场包，正常速度回放

隔离 ROS 域，仅启动惯性处理、激光适配、EKF、保护、门控和 SLAM，不启动硬件驱动。
结果目录：`bags/repair_validation/ros_recovery_final/`。

- 完整回放 216.68 秒，播放器正常结束，所有估计与建图进程在关闭前仍存活。
- IMU 标定完成后始终保持有效；原故障处 data_ready 暂时变假后自动恢复。
- 公共里程计 5320 条，对应 `odom → base_footprint` TF 5320 条。
- 公共里程计最大距原点约 5.201 m；内部 EKF 同样有界。旧记录中故障后的约
  31.5 秒曾出现约 151.8 m 的位置变化，本次没有该无观测发散。
- 最大公共输出间隔约 2.240 秒，覆盖数据恢复、参考核验和 EKF 重新验证；没有
  在此期间伪造连续位姿。随后恢复至录制末尾 216.64 秒。
- `/scan_slam` 2109 帧，末帧 216.59 秒；地图更新 96 次，建图恢复后继续至结束。
- 40–160 秒静止段，公共位置最大变化约 0.368 mm。这是静止约束后的输出稳定性，
  不是独立测量的位置精度。
- 运行参数核实：自动回环为 true，粗/精匹配阈值分别为 0.5 / 0.6。

中断恢复期间也遇到一次单帧增量冲突：新实现拒绝该帧，并经最后可信参考的持续核验
后恢复；没有单帧异常立即永久锁死，也没有直接忽略冲突。

### 主动停止全部观测

在本地及远程独立 ROS 域启动实际 EKF、保护节点和合成观测源；连续 4 秒停止观测。

- 断流后公共里程计/TF 停止，valid 持续为 false。
- 断流中段内部 EKF 位置变化范围为 0，证实暂停了内部积分。
- 本地重新提供已核验的观测后约 0.864 秒恢复公共输出。
- 远程对应约 0.812 秒；断流开始后最后一帧公共输出约晚 0.012 秒。
- 该测试验证保护节点的停用/恢复服务链。整条链路中，真实长时 IMU 失联仍按
  上述 2 秒规则要求重新校准，不能用合成观测测试冒充长期失联后定位可自动恢复。

### 编译与回归

本地及远程四包构建通过，colcon 报告均为：

| 包 | 测试数 | 错误/失败 |
|---|---:|---:|
| kiwi_base | 80 | 0 / 0 |
| kiwi_platform | 18 | 0 / 0 |
| kiwi_slam | 61 | 0 / 0 |
| kiwi_navigation | 18 | 0 / 0 |

覆盖短间隔恢复、积压数据、长时中断、错误参考拒绝、正反向运动、雷达杆臂转向、
滤波发散拒绝、双 TF/错误就绪话题拒绝、建图有效性门控以及两种底盘切换。

远程修改前备份：`/home/a/ros2_ws/backups/recovery_20260926_113427`。
远程没有启动真实底盘或发送电机命令。未做新的实车运动/地图精度验收。
恢复保护不能消除无特征长走廊的可观测性限制，也不能将本次回放当作回环精度真值。

## 使用

远程已构建；停止旧实例后重新启动完整入口，不能混用旧 EKF 和新保护节点：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

启动平放静止约 35 秒，检查 calibration_ready、data_ready、estimation/valid 和
mapping_gate_ready 都为 true。短时中断会自动核验恢复；如果明确提示
`restart required`，停车并重启完整链路。硬件运动测试仍使用遥控器低速操作，记录
`bash tools/record_incident.sh` 的现场包。
