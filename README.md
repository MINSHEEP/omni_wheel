# Kiwi ROS 2 操作手册

履带底盘现有两套独立入口：MID360S + FAST-LIO2 新链路见下文“MID360S 履带建图与导航”；原 M10P + RF2O/C 板 IMU 链路保留，后续旧流程只适用于该入口。两套入口不能同时启动。

## 1. 环境准备与底盘切换

启动、RViz 和录包终端必须统一 `ROS_DOMAIN_ID` 与 `ROS_LOCALHOST_ONLY`。
工作区脚本默认使用域 42、本机发现（`ROS_LOCALHOST_ONLY=1`），保留显式环境变量。
如果查不到节点、录包只收到 `/rosout`，或地图突然停止更新，请先按
[通信检查与建图暂停排查](src/kiwi_slam/README.md#查询不到话题录包为空或建图停止更新)操作。

使用原 `~/ros2_ws` 工作区时，每个新终端先加载环境；MID360S 使用下文单独的工作区和加载顺序：

```bash
cd ~/ros2_ws
# 首次或代码变更后编译工作区
colcon build --symlink-install
source tools/activate_workspace.sh

```

切换活动底盘（修改后重启相关节点生效）：

```bash
# 切换为履带底盘
sed -i 's/^chassis:.*/chassis: tracked_pwm/' src/kiwi_platform/config/active_platform.yaml

# 切换为 Kiwi 底盘
sed -i 's/^chassis:.*/chassis: kiwi/' src/kiwi_platform/config/active_platform.yaml

```

## MID360S 履带建图与导航（2026-09-27）

实机 MID360S 位于 `192.168.1.117`，远程上位机 `a@192.168.66.112` 的雷达网卡 `enp2s0` 已持久设为 `192.168.1.50/24`。新链路部署在远程 `~/ros2_ws_mid360`，Livox 驱动和 FAST-LIO2 分别安装在 `~/livox_ws`、`~/fastlio_ws`；没有覆盖原 `~/ros2_ws`。活动底盘须为 `tracked_pwm`。每个新终端按顺序加载：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/fastlio_ws/install/setup.bash"
source "$HOME/ros2_ws_mid360/install/setup.bash"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
mkdir -p "$HOME/maps"
```

FAST-LIO2 用三维点云与 MID360S 内置 IMU 在内部完成惯性融合，输出本地三维里程计；本模式不启动外部 EKF，C 板 IMU 话题仍保留供诊断。Nav2 使用投影的 `/scan_mid360` 和 `/fastlio/odom_base`：建图时由 SLAM Toolbox 生成二维栅格图，已知图导航时由 AMCL 发布 `map → odom`。FAST-LIO2 的本地里程计不能代替 AMCL 的全局重定位，三维 PCD 也不能直接作为 AMCL 地图。

首次受控建图可先不启动 Nav2，确认雷达和 TF 后再低速遥控车辆：

```bash
ros2 launch kiwi_navigation mid360_navigation.launch.py \
  mode:=mapping start_nav2:=false lidar_model:=Mid360s \
  lidar_interface:=enp2s0 host_ip:=192.168.1.50 lidar_ip:=192.168.1.117 \
  map_pcd:="$HOME/maps/mid360_3d.pcd" use_rviz:=true
```

`use_rviz:=true` 应在远程上位机的图形会话运行；只通过 SSH 启动时可改为 `false`。确认 `/livox/lidar`、`/livox/imu`、`/fastlio/odom_base`、`/scan_mid360` 均持续更新后，在另一个终端加载相同环境并启动键盘遥控：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  --params-file "$HOME/ros2_ws_mid360/src/kiwi_base/config/tracked_pwm.yaml"
```

建图完成后，在已加载相同环境的终端保存二维和三维地图：

```bash

ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/mid360_2d"
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

人工核对二维地图后，停止建图入口，再启动 AMCL 与 Nav2；在 RViz 设置真实初始位姿，确认定位稳定后才发送目标：

```bash
ros2 launch kiwi_navigation mid360_navigation.launch.py \
  mode:=localization map:="$HOME/maps/mid360_2d.yaml" \
  lidar_model:=Mid360s lidar_interface:=enp2s0 \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.117 use_rviz:=true
```

静止实机检查已验证点云约 10 Hz、内置 IMU 约 200 Hz、三维里程计与二维扫描约 10 Hz；二维图、三维 PCD 保存及零速 Nav2 启动均通过。静止测试图不能用于实际导航，安装外参、行驶漂移、地图质量与 AMCL 重定位仍需现场运动验收。详细安装版本、校验步骤与注意事项见 [MID360S/FAST-LIO2 部署说明](documents/navigation/mid360_fastlio2_deployment.md)。

---

## 原 M10P 履带启动校准与建图更新（2026-09-26）

新增运行依赖（ROS 2 已安装）：

```bash
sudo apt update
sudo apt install -y python3-scipy ros-humble-ros2bag ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins ros-humble-rosidl-runtime-py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base kiwi_platform kiwi_slam kiwi_navigation
source tools/activate_workspace.sh
```

履带启动后，在特征充分的区域保持平放静止约 15 秒，等待 `/chassis/imu/calibration_ready` 和 `/kiwi_slam/mapping_gate_ready` 为 `true` 后再遥控建图。校准会更新本次角速度残余偏置、静止加速度基线和测量协方差；不改变已有安装轴向和 yaw 倍率。校准前不会向 EKF 发布惯性/激光观测。

```bash
ros2 topic echo /chassis/imu/calibration_ready --once
ros2 topic echo /chassis/imu/data_ready --once
ros2 topic echo /chassis/estimation/valid --once
ros2 topic echo /chassis/estimation/state --once
ros2 topic echo /chassis/inertial/reason --once
ros2 topic echo /chassis/odom/laser_ready --once
ros2 param get /slam_toolbox do_loop_closing
```

履带自动回环已开启；新扫描先经过背景匹配质量检查。正常起步的运动确认不再被当成激光失效，解除静止保持时保留确认期间的真实位移。激光短时失效且 IMU、惯导协方差仍有效时，独立 `tracked_inertial_odometry` 最多续接 2 秒，保护节点显示 `INERTIAL`，公共里程计和 TF 继续更新，EKF 保持运行。质量不合格的扫描仍暂停入图；激光恢复后，先将 EKF 与激光对齐到同一测量时间核验，通过后直接交回 EKF，不例行调用 `SetPose` 清零速度。真正进入 `PAUSED` 或持续不一致后才走重置恢复流程；回到 `VALID` 后恢复 `/scan_slam`。

走廊内仍能可靠匹配的速度方向通过独立 `/chassis/odom/laser_partial` 提供，与完整激光里程计逐帧互斥。部分观测只约束可观测的速度方向，不补出走廊纵向位置，不重置惯导的 2 秒完整观测时限，也不单独放行建图。`max_position_variance=4.0` m² 是正常方差预算；继承的可信锚点方差已超过该值时保留真实方差，按新增 xy 方差不超过 0.09 m²、yaw 方差不超过 0.05 rad² 及时间上限判断是否可续接，不能把这些配置当成已测精度。

超过 2 秒或惯导不确定度越界时停止有效定位输出；距最后可信扫描 2–10 秒内，只能通过旧参考的强几何匹配、至少 5 帧且持续 0.4 秒核验恢复。超过 10 秒明确要求重新定位或重新建图，不能把新扫描直接设为新原点继续旧地图。2 秒是保守保护上限，尚未完成实车走廊精度验收，不能保证长走廊定位。两种底盘仍统一由 `active_platform.yaml` 切换；Kiwi 使用原轮式参数和链路。参数及状态见 [履带估计有效性与恢复](src/kiwi_slam/README.md#履带估计有效性与恢复)。

履带地图栅格发布周期 `map_update_interval` 为 0.5 秒，不等于雷达约 12 Hz 的扫描频率；静止时即使发布新消息，地图内容也可不变。`kiwi_slam_monitor` 的 `inertial_fallback` 单独报告惯导备用输出是否可用及原因，不能只看到 EKF 或节点名称就认定惯导可接管。

当前现场证据、修复边界和本轮验证结果统一见 [建图卡顿与惯导可用性复核](documents/tracked_mapping_optimization_20260926.md)。[惯性续接与建图恢复](documents/tracked_inertial_recovery_20260926.md)、[早期中断恢复记录](documents/tracked_recovery_20260926.md)与[此前静止测试](documents/tracked_estimation_repair_20260926.md)均为历史前版记录，旧测试通过不代表本轮实车精度通过。当前行为以有效 YAML 和当前报告为准。旧测试目录中的 `base_locked.yaml` 是旧参数快照，不能再拿来启动新版；临时静止测试应从当前 `src/kiwi_base/config/tracked_pwm.yaml` 复制，并关闭运动/试车开关。

## 2. 原 M10P 链路基础测试（底盘与 EKF）

```bash
# 启动底盘、传感器与 EKF 融合
ros2 launch kiwi_base kiwi_base_ekf.launch.py

# 启动键盘遥控（W/S 前后，Q/E 转向，空格/X 刹车）
ros2 run kiwi_base kiwi_teleop_keyboard

```

---

## 3. 原 M10P 链路移动建图流程

### 步骤 1：启动建图节点

```bash
# 方式 A：纯建图（推荐履带底盘使用）
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true

# 方式 B：建图 + Nav2（支持探索导航）
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true

```

### 步骤 2：遥控建图

履带先静置完成上述启动校准并确认建图门控 ready，再另起终端控制车辆：

```bash
cd ~/ros2_ws && source tools/activate_workspace.sh
ros2 run kiwi_base kiwi_teleop_keyboard

```

建图已启动时，另起终端录制当前系统；脚本采集全部话题、节点日志和运行快照，默认按 Ctrl+C 结束：

```bash
cd ~/ros2_ws
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source tools/activate_workspace.sh

bash tools/record_incident.sh
```

录制结束后，用只读分析工具检查实际录包目录（替换示例目录名；现场包解压后的 bag 位于 `bag/all_topics`）：

```bash
bag_dir="$HOME/ros2_ws/bags/tracked_YYYYMMDD_HHMMSS"
python3 tools/analyze_tracked_estimation.py "$bag_dir" --output "${bag_dir}_review"
```

工具输出 `.json` 和中文 `.md`，分别审查状态切换、数据/TF 连续性、地图内容和源间一致性。没有独立实测距离、角度及对应时间的外部真值时，精度必须为 `NOT_EVALUATED`；激光、惯导、EKF 三条轨迹相似不能证明实际距离和方向准确。真值 CSV 约定及参数见[当前报告](documents/tracked_mapping_optimization_20260926.md#外部真值输入约定)。

### 步骤 3：保存地图

建图完成后，保存地图文件至 `~/ros2_ws/maps/`：

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/warehouse

```

---

## 4. 原 M10P 链路已知地图定位与导航

启动定位与 Nav2 导航（必须指定 `.yaml` 地图路径）：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/warehouse.yaml \
  use_rviz:=true

```

> RViz 启动后：
> 1. 点击工具栏 **2D Pose Estimate** 标记车辆初始位姿。
> 2. 点击 **Nav2 Goal** 发送目标点。
> 
> 

---

## 5. Web 控制台（可选）

MID360S 履带链路可通过 Web 启动建图、遥控和已知二维图导航。网页显示 MID360S 内置 IMU、FAST-LIO2 里程计、二维投影扫描及 AMCL 状态；二维栅格用于地图编辑和 Nav2，不把三维点云误称为 AMCL 地图。MID360S Web 速度经私有 `/kiwi_web/cmd_vel`、雷达运动门控和 `/mid360/cmd_vel_safe` 后到达底盘，外部 `/cmd_vel` 不会进入这条 Web 链路。原 M10P 链路可在 Web 配置中选择 `sensor_pipeline: legacy`。

在远程 `a@192.168.66.112` 的 `~/ros2_ws_mid360` 使用 Web 时，先加载 MID360S 节列出的四层 ROS 环境，再启动服务；`maps_dir` 指向实机已有的 `~/maps`，与命令行保存的二维地图共用目录：

```bash
ros2 launch kiwi_web web.launch.py maps_dir:="$HOME/maps"

```

浏览器打开 `http://192.168.66.112:8080`。服务启动本身不会启动雷达和底盘；进入对应页面并点击开始后才拉起所需节点。建图停车后点击“保存”会生成同名二维 YAML/PGM 与三维 PCD，设置页分别提供下载；PCD 使用本次 FAST-LIO2 启动的局部坐标，不直接用于 AMCL。导航前仍需提供实际运动建成的二维地图、在地图上设置 AMCL 初始位姿并检查扫描对齐。详细配置和校验见 [Web 控制台说明](src/kiwi_web/README.md)。

---

## 6. 原 M10P 链路快捷排错命令

```bash
# 检查关键数据流频率
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map

# 检查坐标变换链
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link

```
