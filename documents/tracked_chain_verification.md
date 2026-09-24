# 履带标定与传感器链路验证记录（2026-09-24）

本地工作区 `/home/hi/ros2_ws_old`，远程工作区 `a@192.168.66.112:~/ros2_ws`，
两端使用 `test` 分支，活动底盘均为 `tracked_pwm`。
本次完成配置同步、静止链路检查及受控遥控运动录制。遥控器直接控制下位机，
上位机未下发运动命令；移动建图、AMCL 定位及 Nav2 目标导航仍待验收。

## 1. 标定数据及当前开关

| 固件命令 | 距离 | 平均时间 | 实测平均速度 | 命令 / 速度 |
| --- | --- | --- | --- | --- |
| 5 | 1.6 m | 6.85 s | 0.2336 m/s | 21.4063 |
| 10 | 1.6 m | 3.37 s | 0.4748 m/s | 21.0625 |

两档倍率均值为 21.2344，配置 `command_per_m_s: 21.2` 作为初步直线换算系数。
固件字段名为 rad/s，但它控制 PWM；没有编码器反馈，不能据此认定电机实际转速。

远程测速期间的配置已保存到 Git stash，说明为“地面测速时远程原始电机试车参数”。
当前源码和安装目录读取同一份 `src/kiwi_base/config/tracked_pwm.yaml`：

```yaml
command_per_m_s: 21.2
hardware_calibrated: false
enable_motion: false
keyboard_trial_enabled: true
keyboard_trial_command_gain: 21.2
keyboard_trial_limit_rad_s: 5.0
open_loop_test_enabled: false
open_loop_test_limit_rad_s: 10.0
mapping_sensors_verified: true
```

链路测试结束后，键盘试车倍率由原先 50 更新为直线初测值 21.2，每路上限仍为 5。
旧键盘前进目标 0.10 对应固件命令 2.12；现场反馈前进慢、转向有反应、后退不动。
后续将受限键盘前后目标改为 ±0.235，输出两路约 ±4.98；转向目标改为 ±0.95 rad/s，
左右输出 -5/+5 或 +5/-5，仍受每路 ±5 上限约束。遥控器可正常后退、转向。
这些改动仅解决原先指令幅度过小的问题；提高后的反向起步及实际角速度等待现场复测。
普通运动仍锁定，原始电机标定话题已关闭；当前由 keyboard_trial_command_gain
使用该初测倍率，正常模式的 command_per_m_s 尚未启用。

## 2. 安装外参与实时结果

用户确认的安装位姿已写入 YAML，并通过远程 TF 查询核对加载结果：

| 相对 base_link | xyz（m） | rpy（rad） |
| --- | --- | --- |
| lidar_link | (0.24, 0, 0) | (0, 0, 0) |
| chassis_imu_link | (0.08, 0, 0) | (0, 0, 0) |

基于 RF2O 修复后的 15 秒连续采样，未发送速度命令：

| 话题 | 消息数 | 接收频率 | 首尾变化 |
| --- | --- | --- | --- |
| /scan | 180 | 12.00 Hz | 末帧有限距离占约 42.4% |
| /chassis/imu/data_raw | 1496 | 99.79 Hz | gyro z 均值约 -0.00339 rad/s |
| /chassis/odom/laser | 179 | 11.99 Hz | 平面位置净变化 6.03 mm，航向 -0.132° |
| /odometry/filtered | 450 | 30.00 Hz | 平面位置净变化 7.37 mm，航向 -0.040° |

首尾净变化不是累计漂移、最大偏差或动态精度。这是短时链路观测，尚不能替代
长时间静止、运动方向和真实距离验证。激光负责平面位姿，IMU 提供 z 轴角速度，
由 EKF 融合；当前 IMU 不提供可用绝对姿态，也未融合加速度来估计平移位置。

## 3. 本次修复与核验

- RF2O 原先停止时可能在 ROS 关闭后创建执行器，出现 guard condition 异常。
  已改为复用执行器；远程五个链路节点均正常退出，无残留测试进程。
- RF2O 原先忽略首次 TF 查询失败；已改成等待有效安装 TF 后才初始化。
  本地模拟扫描测试确认：延迟 3 秒提供 TF 前无里程计，提供 TF 后恢复输出，SIGINT 退出码为 0。
  此项验证启动/退出行为，不用于评估 RF2O 算法精度。
- 20 Hz 处理循环会在 12 Hz 雷达的两帧之间等待；正常等待日志改为 DEBUG，
  保留原有处理频率，TF 缺失警告每 5 秒最多打印一次。
- 补丁位于 `tools/patches/rf2o-startup-shutdown.patch`，使用
  `bash tools/apply_rf2o_patch.sh` 应用到固定 RF2O 版本，然后重新构建该包。
  本地和远程构建均通过，补丁脚本可重复运行。
- 本地 `kiwi_platform` 和 `kiwi_slam` 配置/链路测试通过。
  当时远程建图和定位入口仍因传感器未核验而被拒绝；受控运动录制核验后已更新建图开关。
- 远程 `/home/a/ros2_ws/maps/warehouse.yaml` 和 `ceshi3.yaml` 的地图文件及引用图像存在；
  未验证这些旧地图与当前场地是否匹配。

远程原始观测结果：`/tmp/kiwi_sensor_result_patched.json`；运行日志：
`/tmp/kiwi_sensor_diag_patched.log`。临时文件可能在重启或清理后消失，本文件保留主要结果。

## 4. 复测传感器链路

先停止已占用底盘和雷达串口的本项目节点。在工作区根目录生成临时配置，
关闭所有运动模式，并使用独立 ROS 域进行观测。以下不修改持久化试车配置。

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
export ROS_DOMAIN_ID=181
python3 - <<'PY'
from pathlib import Path
import yaml
data = yaml.safe_load(Path('src/kiwi_base/config/tracked_pwm.yaml').read_text())
params = data['tracked_base_node']['ros__parameters']
for key in ('hardware_calibrated', 'enable_motion', 'keyboard_trial_enabled', 'open_loop_test_enabled'):
    params[key] = False
Path('/tmp/tracked_sensor_diagnostic.yaml').write_text(yaml.safe_dump(data))
PY
ros2 launch kiwi_base kiwi_base_ekf.launch.py params_file:=/tmp/tracked_sensor_diagnostic.yaml
```

另开终端加载相同工作区和域，可逐项运行以下观测命令；`topic hz` 用 Ctrl-C 停止：

```bash
cd ~/ros2_ws
source tools/activate_workspace.sh
export ROS_DOMAIN_ID=181
ros2 topic hz /scan
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /chassis/odom/laser
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link chassis_imu_link
```

复测完用 Ctrl-C 停止启动终端。后续键盘试车重新开终端，加载工作区后使用 README
中的普通底盘和键盘命令；不要让同一串口由两个底盘节点同时打开。

## 5. 建图、导航前仍需完成

1. 测低于固件命令 5 时能否起步、停车，补测反向、左右履带差异和原地转向。
   Nav2 当前最高直线速度 0.15 m/s 按 21.2 换算只得到命令 3.18，低于已测最低档。
2. 遥控运动录制已核对 IMU 与 RF2O 的双向转角及短时静止漂移；原始 IMU 存在约
   -0.185°/s 的静止偏置，EKF 应继续依赖激光里程计约束航向。原地转向时机体估计
   有平移，需目视核对实际旋转中心并在后续地图上检查重影。
3. 先用遥控器完成手动移动建图，检查 `/map`、`map → odom`、闭环后的墙体一致性。
4. 上位机低速起步、反向与转向实测完成后，关闭试车模式并启用正常运动，再进行低速导航。
   建图使用 `kiwi_slam.launch.py`，定位导航使用 `localization_navigation.launch.py`，
   两者继续读取同一活动底盘选择文件和履带专属配置；完整命令见根 README。

## 6. 受控遥控运动录制与建图门控复核

2026-09-24 16:45:57 开始的 73.1 秒录制在远程上位机：
`~/ros2_ws/bags/tracked_motion_20260924_164557`。原始 `/scan` 877 帧（约 12 Hz）、
原始 IMU 7290 帧（约 100 Hz）、RF2O 878 帧（约 12 Hz）、EKF 2193 帧（约 30 Hz）。
记录中没有 `/cmd_vel`，因为这次动作使用遥控器直接控制下位机，不能用它标定
上位机命令到实际速度的换算。

以下平移是在各段起点的机体坐标系下计算的激光里程计估计值，角度正值为逆时针；
用户只给出约 1 米、约 90° 的目标动作，没有外部测距/测角真值。

| 动作及录制时间 | RF2O 前向 / 左向 | RF2O 转角 | IMU 积分转角，扣除初段静止偏置 |
| --- | ---: | ---: | ---: |
| 前进 11–16.5 s | +1.66 / +0.10 m | -1.7° | -1.5° |
| 后退 18.5–26 s | -1.65 / -0.16 m | +1.1° | +1.1° |
| 逆时针 38–41.5 s | +0.14 / +0.16 m | +94.5° | +93.8° |
| 顺时针 48–52.5 s | +0.31 / -0.16 m | -100.5° | -101.3° |

两种直行方向及两种转角符号都正确；旋转段 RF2O 与 IMU 的角度差分别约 0.7°、
0.9°。原始 IMU 前 10 秒 z 轴静止中位偏置约 -0.00323 rad/s（-0.185°/s）；
没有静止偏置校正时，不应长期单独积分作绝对航向。初段 10 秒 RF2O 净平移约
1.4 cm、航向 +0.17°；末段 20 秒约 1.0 cm、+0.46°。这些只是本场地短时表现。
旋转段估计有约 0.21 / 0.35 m 的合成平移，可能包含实车履带绕非中心点旋转，
现有录制无法区分真实位移与激光匹配误差，建图时要检查墙体是否重影。

初始静止时相邻扫描的共同有效束比例中位约 0.91、共同束距离差中位约 7 mm；
有效距离束比例约 0.37，均满足建图首帧扫描阈值。真正阻塞首帧的是 EKF/RF2O
静止速度噪声：初始 10 秒 EKF `|vx|` 中位 0.013 m/s、95 分位 0.044 m/s。
原履带门控 `max_linear_speed_m_s=0.02` 回放此包至约 44.3 秒才放行；仅将履带
专属阈值设为 0.04 后，回放约 1.8 秒放行。其余新鲜度、扫描重叠及角度阈值未改。
据此 `mapping_sensors_verified=true`，允许进入手动建图测试；
`hardware_calibrated=false`、`enable_motion=false` 仍禁止正常运动及 Nav2。
