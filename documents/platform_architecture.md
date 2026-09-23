# 可切换底盘平台架构

统一入口为 `ros2 launch kiwi_navigation platform.launch.py`。`chassis`、`lidar`、`imu`、`odom` 和 `mode` 独立选择，启动时先检查不兼容组合。默认 `chassis:=kiwi lidar:=m10p imu:=chassis odom:=auto mode:=mapping`。旧启动文件仍可用于原车。

| 选择 | 当前实现 | 统一接口 |
| --- | --- | --- |
| `chassis` | `kiwi`、`tracked_pwm` | 订阅 `/cmd_vel`，对外提供对应的 IMU；履带无轮速里程计 |
| `lidar` | `m10p`、`external_scan` | 唯一 `/scan`，frame 为 `lidar_link`；`external_scan` 由外部驱动发布 |
| `imu` | `chassis`、`external` | EKF 使用 `/chassis/imu/data_raw` 或 `external_imu_topic`；外部惯导须自行校准轴向并与 URDF 中的 IMU frame 对齐 |
| `odom` | Kiwi=`wheel`，履带=`rf2o` 或 `external` | 输入里程计消息，EKF 独占发布 `/odometry/filtered` 和 `odom -> base_footprint` |
| `mode` | `driver`、`mapping`、`localization` | 驱动联调、SLAM Toolbox 建图、AMCL 地图定位与 Nav2 |

坐标树只有一个发布者负责每段 TF：SLAM Toolbox 或 AMCL 发布 `map -> odom`，EKF 发布 `odom -> base_footprint`，URDF 发布 `base_footprint -> base_link -> lidar_link/chassis_imu_link`。RF2O 的 `publish_tf=false`，仅发布 `/chassis/odom/laser`，避免与 EKF 抢同一 TF。履带底盘不会发布 `/chassis/odom/raw` 或伪造的 `/joint_states`；固件 0x3B 中的全零电机速度不能进入 EKF。

## 共享节点与底盘配置

统一入口按 `chassis` 选择 `kiwi_base_node` 或 `tracked_base_node`；这两个节点分别处理
三轮反馈里程计和双路 PWM 无电机反馈协议。其余 `kiwi_teleop_keyboard`、
`robot_state_publisher`、M10P 驱动、EKF、建图门控、SLAM Toolbox 和 Nav2 复用。
Kiwi 与履带专属参数分别集中在 `src/kiwi_base/config/kiwi_base.yaml` 和
`src/kiwi_base/config/tracked_pwm.yaml`。雷达、SLAM 和 Nav2 的公共参数继续共用；
统一入口会从所选底盘 YAML 读取建图门控及差速 Nav2 覆盖值。

键盘沿用同一节点；Kiwi 直接运行 `ros2 run kiwi_base kiwi_teleop_keyboard`，
履带运行 `ros2 run kiwi_base kiwi_teleop_keyboard --ros-args --params-file src/kiwi_base/config/tracked_pwm.yaml`。
履带使用 `w/s` 前后和 `q/e` 转向；`a/d` 发布横移，会被履带节点拒绝并停车。
当前运动锁定时键盘仍可发布 `/cmd_vel`，下位机只收到零电机命令。

## 两电机履带底盘

上位机沿用 C 板 USB CDC 协议：0x3A 的 `motor1=左履带`、`motor2=右履带`、`motor3=0`，无 ACK；0x3B 仍读取惯导字段，不把电机字段当反馈。遥控优先级和 USB 超时由下位机负责；上位机另有 0.20 s `/cmd_vel` 看门狗。横向速度命令和非有限值会被拒绝。双履带速度按 `v_left = vx - wz * track_separation_m / 2`、`v_right = vx + wz * track_separation_m / 2` 转为固件命令，并在超限时等比例缩放。

`src/kiwi_base/config/tracked_pwm.yaml` 中以下值为 TODO，**必须在断开电机负载的状态下核对方向、量纲和上限**：

- 左右履带中心距 `track_separation_m`。
- 每 1 m/s 对应固件电机命令的 `command_per_m_s`；需考虑减速比、驱动器 PWM 映射和正反方向。
- `max_motor_command` 已按当前固件 `PWM_DIFF_USB_MAX_COMMAND=20.0 rad/s` 设置；后续固件变化时同步。
- `tracked_robot_model` 的机身、履带、雷达及惯导安装尺寸，以及 Nav2 代价地图的外接半径或足迹。
- 0x3B 惯导安装轴和 gyro_z 比例；更换惯导时重新标定协方差与外参。

当前 `hardware_calibrated=false`、`enable_motion=false`：驱动联调模式可以启动串口和 IMU，但发送零命令。完成实测和空载方向验证后再开启运动；建图和导航入口在两个开关打开前会报错退出。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to kiwi_navigation
source install/setup.bash

# 先检查 USB 通讯、0x3B 惯导；电机保持零命令。
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm mode:=driver
```

履带使用 M10P 2D 激光里程计 RF2O。仓库通过 `tools/platform_dependencies.repos` 固定其 Humble 源码版本；这是独立 GPLv3 软件包，按其 LICENSE 使用。联网准备一次后构建：

```bash
vcs import src < tools/platform_dependencies.repos
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-up-to kiwi_navigation rf2o_laser_odometry
source install/setup.bash
```

完成上面的 TODO 和参数验证后：

```bash
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm lidar:=m10p imu:=chassis mode:=mapping
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm lidar:=m10p imu:=chassis mode:=localization map:=/绝对路径/map.yaml
```

外部雷达需发布 `/scan` 且 frame 为 URDF 中对应的 `lidar_link`；用 `lidar:=external_scan` 选择。外部惯导用 `imu:=external external_imu_topic:=/your/imu`，其 frame 必须在 URDF 中有正确静态外参。外部激光/视觉里程计用 `odom:=external external_odom_topic:=/your/odom`，消息须使用 `odom` 和 `base_footprint` frame，且不得额外广播 `odom -> base_footprint`。新增硬件类型只需添加对应驱动封装、URDF 外参和配置选项，不修改导航算法内部话题契约。

建图前检查 `/scan`、`/chassis/odom/laser`、`/odometry/filtered` 的频率及 `ros2 run tf2_ros tf2_echo odom base_footprint`；定位导航还需检查 `map -> odom`。激光扫描匹配在空旷或高度对称环境中可能退化，实车导航前应验证静止漂移、直行/原地转向、急停以及地图闭环质量。
