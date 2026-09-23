# 双底盘统一选择架构

## 唯一开关与各底盘档案

所有常用入口都读取 `src/kiwi_platform/config/active_platform.yaml` 的 `chassis`：`kiwi` 或 `tracked_pwm`。档案 `src/kiwi_platform/config/profiles/{kiwi,tracked_pwm}.yaml` 指定底盘节点、轮式或 RF2O 里程计、底盘/EKF、键盘、建图门控、SLAM、Nav2、雷达与 URDF 文件。启动前校验所选参数中的话题、frame、运动类型、外形和锁定条件；显式覆盖文件也受同样检查。

```bash
cd "$HOME/ros2_ws"
sed -i 's/^chassis:.*/chassis: tracked_pwm/' src/kiwi_platform/config/active_platform.yaml
source tools/activate_workspace.sh
```

使用 `--symlink-install` 时重启 ROS/Web 进程即可生效；普通安装还需重建 `kiwi_platform`。Web 服务会记录启动时的底盘，如果运行中修改开关，重启 Web 后再启动模式。

| 路径 | Kiwi | 履带 |
| --- | --- | --- |
| 底盘/EKF/键盘 | `kiwi_base/config/kiwi_base.yaml` | `kiwi_base/config/tracked_pwm.yaml` |
| 建图门控 | `kiwi_slam/config/mapping_gate.yaml` | `kiwi_slam/config/tracked_mapping_gate.yaml` |
| 建图算法 | `kiwi_slam/config/slam_toolbox.yaml` | `kiwi_slam/config/tracked_slam_toolbox.yaml` |
| 里程计 | 电机反馈 `/chassis/odom/raw` | `kiwi_slam/config/tracked_rf2o.yaml`，`/chassis/odom/laser` |
| 定位导航 | `kiwi_navigation/config/nav2.yaml`，AMCL 全向模型 | `kiwi_navigation/config/tracked_nav2.yaml`，AMCL 差速模型 |
| 机器人模型 | `kiwi_slam/urdf/kiwi.urdf.xacro` | `kiwi_slam/urdf/tracked.urdf.xacro` |

目前两个档案都使用 M10P 雷达 `/scan` 和 C 板 IMU `/chassis/imu/data_raw`。更换雷达或惯导时，应新增驱动、专属参数与 URDF 外参，并更新对应档案和校验；不能仅改型号字符串。M10P 配置 `kiwi_slam/config/ls_m10p_uart.yaml` 目前共用。

## 启动命令

```bash
# 底盘节点与模型，履带当前可用于串口/IMU 和开环试车。
ros2 launch kiwi_slam kiwi_robot.launch.py start_ekf:=false start_lidar:=false
# 完整里程计链路；履带自动带 M10P、RF2O、EKF。
ros2 launch kiwi_base kiwi_base_ekf.launch.py
# 只建图；履带速度未标定时使用遥控或原始电机话题移动。
ros2 launch kiwi_slam kiwi_slam.launch.py
# 建图 + 可用时的 Nav2。履带速度未标定时不启动 Nav2。
ros2 launch kiwi_navigation mapping_navigation.launch.py
# 已知地图定位导航；履带标定前拒绝启动。
ros2 launch kiwi_navigation localization_navigation.launch.py map:="$HOME/ros2_ws/maps/warehouse.yaml"
# 前台键盘，自动读取所选底盘参数。
ros2 run kiwi_base kiwi_teleop_keyboard
# 网页：跟随相同开关，切换底盘后重启服务。
ros2 launch kiwi_web web.launch.py
```

这些是独立入口。`platform.launch.py` 仅保留旧命令兼容转发，不再负责组装算法参数，不能用 `chassis:=...` 覆盖活动选择。可用 `platform_file:=/绝对路径/active.yaml` 在单次 launch 测试另一选择；生产运行建议只编辑唯一开关文件。

## 坐标、里程计和关锁

Kiwi：`/chassis/odom/raw + /chassis/imu/data_raw → EKF → /odometry/filtered`。履带：`/scan → RF2O /chassis/odom/laser + /chassis/imu/data_raw → EKF → /odometry/filtered`。RF2O 显式设置 `init_pose_from_topic: ""`，不等待实车不存在的真值位姿话题；`publish_tf=false`，EKF 独占发布 `odom → base_footprint`。SLAM Toolbox 或 AMCL 发布 `map → odom`；URDF 发布机体到雷达、惯导的静态 TF。履带不发布轮式 `/chassis/odom/raw`，固件 0x3B 的全零电机字段不得进入 EKF。

履带机身长 0.70 m、宽 0.59 m、左右履带中心距 0.50 m 已实测，Nav2 先用 0.48 m 外接半径。机身高度、履带宽高、质量、雷达和 IMU 安装位姿、IMU 轴向、RF2O 静止漂移、速度换算 `command_per_m_s` 仍需标定；配置中的临时结构尺寸不是实测值。履带 Nav2 禁止 `vy`，速度平滑开环，AMCL 用差速模型。导航上限只是待实测的保守起点。

`tracked_pwm.yaml` 默认 `hardware_calibrated=false`、`enable_motion=false`、`mapping_sensors_verified=false`：普通 `/cmd_vel` 只发送零电机命令。核验雷达/惯导外参、轴向及 RF2O 后，设 `mapping_sensors_verified=true` 可手动建图。导航还需标定 `command_per_m_s`、确认空载方向、关闭 `open_loop_test_enabled`，再启用 `hardware_calibrated` 和 `enable_motion`。未标定时定位导航和 Web 导航均在启动前拒绝；建图入口不会启动 Nav2。

驱动联调时先架空履带，原始命令 `/chassis/open_loop_motor_cmd` 为 `std_msgs/msg/Float32MultiArray` 的 `[左, 右]` 固件 rad/s 数值，初始每路限制 ±10 rad/s，0.20 s 超时停车。当前固件上位机 `0 / +10 / +20 / -20 rad/s` 约对应 PWM `1500 / 1750 / 2000 / 1000 µs`；地面测 1 m 用时后 `10/v` 仅是下一次试验换算起点。遥控器满摇杆脉宽尚未测量，不能由此推断真实速度。

Web 就绪链也按活动底盘切换：Kiwi 检查轮式反馈和 IMU 标定状态；履带检查 RF2O 激光里程计、融合里程计与实时 IMU。履带未启用正常运动时，Web 手动遥控保持未就绪。修改活动开关或底盘标定文件后需重启 Web 服务。
