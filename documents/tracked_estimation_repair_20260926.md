# 履带启动校准、激光质量与自动回环修复

日期：2026-09-26。范围：现有 kiwi_base / kiwi_platform / kiwi_slam / kiwi_navigation 链路，不新增功能包。

## 已实现

1. `tracked_imu_acceleration` 启动时等待背景扫描确认停车，再采集连续 30 秒有效 IMU。更新本次 gyro_z 残余偏置、平放静止加速度合量及完整加速度协方差；保留现有轴向、yaw 符号/倍率和安装矩阵。协方差保留保守下限，不将静止样本方差冒充运动精度。校准时移动、窗口不稳定或断流会重采；完成后的传感器中断进入需要重启的故障状态。
2. 校准前不发布惯性观测；校准后分别输出 `/chassis/imu/gyro` 和 `/chassis/imu/acceleration`，EKF 仅分别融合 gyro_z、ax/ay，避免重复融合姿态或重力。原始数据话题保留。这里是现有 EKF 的平面惯性预测，不是完整三维 INS，也没有新增独立 odom TF。
3. 激光适配器用点到线背景对齐、截尾/Huber 抑制动态前景、归一化几何信息矩阵检测弱约束方向。固定停车参考防止逐帧零漂；运动证据采用位置/角度及 gyro 的持续性判断，避免旧版 11 mm 中位差一越界就释放。
4. 匹配重叠率、残差和方向信息改变观测协方差；严重退化不发布可信激光观测。不可信状态不等价于零速度。移动中持续丢失、RF2O 超过物理界限或与背景运动不一致时，暂停建图并要求停车重启，不悄悄重置里程计。
5. 扫描和 RF2O 按原始时间戳配对，支持两者任意接收先后。只有质量通过的扫描才进入 `/scan_validated`，随后由 ready 门控转发到 `/scan_slam`。
6. 仅背景确认停车时发布 `/chassis/odom/stationary` 条件零速度约束，EKF 使用其 vx/vy；运动、未知状态或退化时停止该约束。激光主输入只融合位姿，不再把 RF2O 固定填写的 vy=0 当成运动中的独立测量。条件约束和激光位姿共享背景证据，不能宣称它们是两套独立位置传感器。
7. 履带 SLAM 恢复 `do_loop_closing=true`，粗/精匹配最低响应分别为 0.5/0.6。Kiwi 的独立配置和轮式链路保留。

## 配置入口及启动

底盘仍由 `src/kiwi_platform/config/active_platform.yaml` 统一选择。IMU、EKF 参数在 `src/kiwi_base/config/tracked_pwm.yaml`；激光质量在 `src/kiwi_slam/config/tracked_rf2o.yaml`；SLAM/门控使用各自 tracked 配置。

新增系统依赖：`python3-scipy`。远程系统已安装 scipy 1.8.0 / numpy 1.21.5。仅使用发行版 Python 运行 ROS 节点；本地存在用户安装的 numpy 1.26.4 与旧 scipy 的兼容版本警告，本轮执行未失败，未修改用户全局 Python 环境。

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

启动后保持平放静止，通常约 35 秒；在特征充分区域等待两个状态均为真后再遥控：

```bash
ros2 topic echo /chassis/imu/calibration_ready --once
ros2 topic echo /kiwi_slam/mapping_gate_ready --once
ros2 param get /slam_toolbox do_loop_closing
```

若只检查底盘/估计链：`ros2 launch kiwi_slam kiwi_robot.launch.py`，不要同时再启动一套驱动。
旧 bag 目录中的临时 base_locked.yaml 不含新链路参数，不能继续使用。静止测试应从当前 tracked_pwm.yaml 复制配置并关闭 enable_motion、keyboard_trial_enabled、open_loop_test_enabled。

### 主要参数和行为

| 参数/条件 | 当前值或含义 |
| --- | --- |
| startup_duration_s | 30.0 秒连续有效静止 IMU 窗口 |
| gyro_variance_floor | 0.001 (rad/s)²，保守测量噪声下限 |
| acceleration_variance | 三轴 0.04 (m/s²)² 下限；保留实测交叉协方差 |
| minimum_information_ratio | 0.015，归一化几何信息最小/最大特征值比阈值 |
| max_rejection_s | 1.0 秒，移动中持续缺乏可信扫描后不能直接恢复位置 |
| quality_enabled | 履带必须 true；旧 stationary_* 中位差参数仅用于 legacy 对照路径 |
| lidar_xy | [0.24, 0.0]，与模型配置校验一致，用于协方差和运动检查杆臂转换 |
| ready/quality | 启动、断流和质量变化均实时检查；失效后不继续转发建图扫描 |

初版背景判断采用 15 mm/0.012 rad 静止容差、25 mm/0.02 rad 运动证据、初次运动约 0.15 秒确认；明显 gyro 转动可迅速释放。固定参考让慢速位移可以累积越过阈值，避免永远保持；仍有检测迟滞和有限运动分辨率，必须通过真实低速启停验收。

## 测试与证据

### 编译和测试

本地和远程均执行四包 colcon build / colcon test，全部通过。colcon 汇总：

| 包 | 汇总 tests | errors / failures |
| --- | ---: | --- |
| kiwi_slam | 55 | 0 / 0 |
| kiwi_platform | 15 | 0 / 0 |
| kiwi_navigation | 18 | 0 / 0 |
| kiwi_base | 80 | 0 / 0 |

覆盖启动运动拒绝、残余偏置估计、断流失效、ready 撤销、前景人员干扰、低速前进/后退、左右转向、长走廊退化，以及两个底盘的参数和启动契约。

### 原始数据回归

- `covariance_20260926_091507`：全部静止；新背景判断不再出现旧版两次错误释放。约 32.06 秒满足校准窗口。
- `static_odom_20260926_094946`：约第 5 分钟有人经过。新背景判断未进入 moving；两帧 ambiguous 暂停输出后恢复原停车参考，不接纳错误位移。约 32.16 秒满足校准窗口。该时间来自按原始接收时间的纯算法重放。
- `tracked_motion_20260924_164557`：受控前进/转向数据，背景判断能够释放并进入 moving，未被永久锁住。该旧数据首次运动前不足 30 秒静止，因此不能作为“完整启动校准成功”的证据；其运动检测与新启动协议分别验证。
- `tracked_20260925_182915`：包含快速转向及匹配丢失；新逻辑会降级。没有将这份问题数据宣称为已恢复准确轨迹。

### 实际 ROS 节点重放

隔离 ROS 域 87、无底盘驱动，启动真实 IMU 处理、激光适配器、EKF、建图门控和 SLAM，按 4 倍速播放同一份 09:49:59 静止原始数据。没有播放旧 EKF 或旧 TF 动态输出。

本地最终记录 `bags/repair_validation/ros_static_v2/output`：

- 新激光输出约 11.99 Hz（以源消息时间戳计），相对初始化位置无漂移。
- EKF 相对初始化位置最大变化约 **0.349 mm**，末端约 **0.108 mm**；yaw 全程跨度约 0.0844°。
- 对照旧在线结果：最大位置变化约 **132.3 mm**，末端约 **131.6 mm**。
- 成功产生校准 gyro/加速度、条件零速度、scan_validated、scan_slam、map 和完整 TF 链。
- 运行参数快照确认 `do_loop_closing=true`，粗/精回环响应阈值为 0.5/0.6。
- 所有估计/建图进程在播放结束前均仍运行，随后由测试脚本停止。

这些数字是**条件停车约束下的估计输出漂移**，不是独立测得的亚毫米定位精度。4 倍速重放改变回调/定时器调度，不能直接代替实时实车验收。静止重放也没有验证真实闭合路线上的回环效果。

![相同静止输入的输出对照](../bags/repair_validation/before_after.png)

## 部署与回退

修复源文件已同步 `/home/a/ros2_ws` 并编译测试。未启动远程电机或远程建图。
覆盖前文件备份：`/home/a/ros2_ws/backups/tracked_estimation_20260926_103434`；含文件清单及 COLCON_IGNORE，避免备份被当成重复 ROS 包。同步没有覆盖 kiwi_web 的既有修改。

现有正在运行的旧节点不会自动加载新 Python 模块/参数；下次测试请完整结束旧估计/建图节点再启动。
本次未执行 Git 提交。

## 尚未证明的能力

- 没有编码器且走廊沿线几何不可观测时，IMU 不能长期提供准确位置。当前实现会报告/保持降级并暂停建图；不会把长期惯性漂移包装为正常激光定位。
- 启动静止仍是操作前提。无外部速度参考时，不能数学上排除完全无特征环境中的匀速运动；因此初始校准要求背景可观测。
- 静止统计只校正本次残余偏置，不能完成三轴六面内参、在线温漂、坡地姿态或 EKF Q/P₀ 的动态标定。Q/P₀ 保留现有库设置，尚不能称为已实测标定。
- 方向协方差是局部匹配质量的保守近似，不是已经用独立真值验证的累计位姿误差模型。
- 条件零速度和低速运动释放仍需当前设备实测，尤其遥控低速起步、缓慢转向、驶入/驶出狭窄通道。
- 自动回环已开启，但误回环风险不能仅凭提高匹配阈值排除。必须低速走一条短闭合路线，核对回到原地后的墙体、扫描与 map→odom 变化。
- 自主导航尚未实车验收；本次交付用于先复测估计与建图。
