# Kiwi ROS 2 移动机器人工作区

新增两电机履带底盘与底盘/雷达/惯导一键选择入口，参见 [可切换底盘平台架构](documents/platform_architecture.md)。本文件下方仍记录原 Kiwi 平台的运行方法。

这是 ROS 2 Humble 移动机器人工作区。原 Kiwi 三轮全向底盘的运行链路使用：

- RoboMaster C 板 USB CDC 底盘驱动（0x3A 命令无 ACK，0x3B 电机/IMU 异步遥测）
- 镭神 LSLIDAR M10P 二维 UART 激光雷达
- `robot_localization` 平面 EKF
- `slam_toolbox` 二维在线建图
- Nav2 AMCL 已知地图定位和全向导航


## 1. 工作区结构

```text
ros2_ws/
├── src/
│   ├── kiwi_base/       底盘、运动学、反馈里程计、IMU、键盘
│   ├── kiwi_slam/       M10P 配置、SLAM Toolbox、URDF、链路诊断
│   ├── kiwi_navigation/ AMCL、Nav2、建图/定位导航 launch 和 RViz
│   ├── kiwi_web/        本地 Web 控制台、地图编辑与任务管理
│   ├── lslidar_driver/  M10P UART 驱动
│   ├── lslidar_msgs/    镭神驱动消息接口
│   └── rf2o_laser_odometry/ 履带平台激光里程计（vcs 导入）
├── build/                colcon 构建目录
├── install/              colcon 安装目录
└── maps/                 用户保存的地图（需自行创建）
```

地图文件不放在源码包中。建议统一保存到：

```text
$HOME/ros2_ws/maps/
```

每张地图至少包含同名的 `.yaml` 和 `.pgm` 文件，例如：

```text
maps/warehouse.yaml
maps/warehouse.pgm
```

导航启动参数使用 `.yaml` 文件：`map:=$HOME/ros2_ws/maps/warehouse.yaml`。

## 2. 坐标、话题和速度接口

ROS 机体坐标遵循 REP-103：`+X` 向前，`+Y` 向左，`+Z` 向上，`+wz` 逆时针。
键盘中 `W` 发布 `vx+`，`A` 发布 `vy+`，`Q` 发布 `wz+`。`base_footprint` 和
`base_link` 的固定关节没有旋转，实车机械前方必须与红色 X 轴一致。

```text
/scan                    M10P LaserScan
/cmd_vel                 所有速度源的统一入口
/chassis/odom/raw        根据滤波后的电机实测速度推算的轮式里程计
/chassis/imu/data_raw    C 板异步 IMU 遥测（绝对 yaw、角速度、原始加速度）
/chassis/feedback/raw    C 板异步 0x3B 原始字段调试消息
/joint_states            根据电机实测速度换算的三个轮关节状态
/odometry/filtered       EKF 输出，Nav2 使用此话题
/map                     SLAM Toolbox 或 map_server 发布的地图
```

TF 链应为：

```text
建图：       map -> odom -> base_footprint -> base_link -> lidar_link
已知图导航： map -> odom (AMCL) -> base_footprint (EKF) -> base_link -> lidar_link
```

`kiwi_base.yaml` 是底盘和 EKF 的唯一参数入口。键盘的 `speed_m_s`、`yaw_rad_s` 是手动
速度，底盘的 `max_*` 是所有 `/cmd_vel` 输入的最终安全上限；两者不是重复配置。
Nav2 还会在 `kiwi_navigation/config/nav2.yaml` 中对自主导航速度进行单独限制。

## 3. 全新上位机部署

### 3.1 已安装 ROS 2 的上位机：准备构建工具

前提：上位机是 Ubuntu 22.04，已安装 ROS 2 Humble，且
`/opt/ros/humble/setup.bash` 可读取。本项目部署不重复安装 ROS 2；新终端先加载它，
再安装 `rosdep`、源码管理/构建工具和驱动遗漏的 `yaml-cpp` 开发包：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git usbutils python3-rosdep python3-vcstool \
  python3-colcon-common-extensions python3-serial libyaml-cpp-dev
```

`rosdep` 会按工作空间的 `package.xml` 安装 M10P 驱动需要的 PCL、Boost、Eigen、
`libpcap`，以及导航和 `kiwi_web` 的 ROS/Python 依赖。`lslidar_driver` 源码还使用
`yaml-cpp`，但包清单未声明它，因此上面单独安装 `libyaml-cpp-dev`。

### 3.2 获取源码并安装工作空间依赖

以下以 `~/ros2_ws` 为例。已有完整 `test` 分支源码的上位机可直接进入工作空间，
不用再次克隆；全新机器克隆前，源机器需先将本地 `test` 分支发布到远端
（`git push -u origin test`）。不要复制旧机器的 `build/`、`install/`。

```bash
# 仅首次获取项目时执行；已有 ~/ros2_ws 则跳过此行。
git clone --branch test https://gitee.com/yonheybot/omni-wheel-arm-chassis.git "$HOME/ros2_ws"
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
# 仅首次需要履带里程计时执行；已有 src/rf2o_laser_odometry 则跳过。
vcs import src < tools/platform_dependencies.repos
```

`vcs import` 从 `tools/platform_dependencies.repos` 获取履带底盘的 RF2O 固定版本，
需要访问 GitHub；只用 Kiwi 底盘时可跳过此行。项目源码已包含 M10P 的
`lslidar_driver` 和 `lslidar_msgs`。

这台上位机访问 GitHub Raw 时 `rosdep init` 出现 TLS 握手超时。按
[清华 TUNA 的 GitHub Raw 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/github-raw/)
和 [ROS Distro 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/)
手动配置 `rosdep` 数据源和索引；**配置后不再执行 `sudo rosdep init`**：

```bash
sudo install -d -m 755 /etc/ros/rosdep/sources.list.d
sudo tee /etc/ros/rosdep/sources.list.d/20-default.list >/dev/null <<'SOURCES'
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/base.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/python.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/ruby.yaml
SOURCES
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys cmake_modules -y
```

RF2O 上游包清单有未被 CMake 使用的旧 `cmake_modules` 键，Humble 下无对应
rosdep 定义，所以只跳过该键。以后在新终端再次运行 `rosdep update` 前，需重新设置
`ROSDISTRO_INDEX_URL`；若镜像也无法访问，先检查网络，不要关闭 TLS 校验。

### 3.3 配置这台上位机的雷达与下位机 USB 串口

实测设备信息如下。`ttyACM0/1` 仅为本次插入顺序，udev 规则按设备序列号匹配，
重启或换 USB 口后仍使用固定名称：

| 设备 | VID:PID | 序列号 | 固定名称 |
| --- | --- | --- | --- |
| M10P 雷达 WCH USB 串口 | `1a86:55d4` | `0001` | `/dev/wheeltec_lidar` |
| STM32 C 板 USB CDC | `0483:5740` | `364E30673533` | `/dev/robomaster_c` |

在该上位机创建两条规则：

```bash
sudo tee /etc/udev/rules.d/99-wheeltec-lidar.rules >/dev/null <<'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", SYMLINK+="wheeltec_lidar", GROUP="dialout", MODE="0660"
RULE
sudo tee /etc/udev/rules.d/99-robomaster-c.rules >/dev/null <<'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{serial}=="364E30673533", SYMLINK+="robomaster_c", GROUP="dialout", MODE="0660"
RULE
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo usermod -aG dialout "$USER"
```

注销并重新登录，使 `dialout` 组权限生效；然后检查链接目标及权限：

```bash
id -nG | grep -w dialout
ls -l /dev/wheeltec_lidar /dev/robomaster_c
readlink -f /dev/wheeltec_lidar
readlink -f /dev/robomaster_c
```

更换雷达转串口模块或 C 板后，先运行 `lsusb` 与
`udevadm info --query=property --name=/dev/ttyACM0` 等命令重新确认 VID/PID、
`ID_SERIAL_SHORT`，再改规则；不要照搬旧序列号。雷达路径在
`src/kiwi_slam/config/ls_m10p_uart.yaml` 的 `serial_port`；Kiwi 和履带底盘路径分别在
`src/kiwi_base/config/kiwi_base.yaml`、`src/kiwi_base/config/tracked_pwm.yaml` 的 `device`。
M10P 串口速率 500000 由驱动按型号设置；C 板 USB CDC 不需要设置波特率。

完成第 4 节构建后，每次新终端先加载环境：

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
```

不要在同一终端最后加载含旧 `kiwi_base` 的其他工作区。激活脚本会把当前工作区放到
overlay 最前面，并校验 `kiwi_base`、`kiwi_slam`、`kiwi_navigation` 的实际来源；这可避免
EKF 因误读旧 YAML 而静默回退到没有 `odom0/imu0` 的默认配置。

## 4. 构建和测试

依赖安装和 udev 配置完成后，从工作区根目录构建导航、M10P 驱动、Web 控制台和履带 RF2O：

```bash
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kiwi_web rf2o_laser_odometry
source tools/activate_workspace.sh
```

不使用履带时可把构建命令末尾的 `rf2o_laser_odometry` 去掉；不部署网页时，
将 `kiwi_web` 换成 `kiwi_navigation`。构建后先检查包来源和设备：

```bash
ros2 pkg prefix kiwi_navigation
ros2 pkg prefix lslidar_driver
ros2 pkg prefix kiwi_web  # 部署网页时
ros2 pkg prefix rf2o_laser_odometry  # 仅履带需要
ros2 run kiwi_slam kiwi_slam_compatibility --require-hardware  # 原 Kiwi 配置检查
```

要单独检查雷达驱动，在一个终端运行 `ros2 launch kiwi_slam m10p_uart_lidar.launch.py`，
另一个已加载工作区的终端运行 `ros2 topic hz /scan`。底盘联调和建图导航的完整启动命令见
[可切换底盘平台架构](documents/platform_architecture.md)；履带参数尚未标定时只能使用
`chassis:=tracked_pwm mode:=driver`；核验雷达、惯导和 RF2O 后，可在速度未标定时手动建图，导航仍锁定。

只构建底盘：

```bash
colcon build --symlink-install --packages-select kiwi_base
```

运行测试：

```bash
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_base kiwi_slam kiwi_navigation kiwi_web \
  --event-handlers console_cohesion+ --ctest-args --output-on-failure
colcon test-result --all
```

构建后可先做平台、依赖和设备检查：

```bash
ros2 run kiwi_slam kiwi_slam_compatibility
ros2 run kiwi_slam kiwi_slam_compatibility --require-hardware
```

不带 `--require-hardware` 时，缺少串口只会给出警告，适合在无实车环境检查配置；实车启动前
应使用带该选项的检查。

### 统一入口：切换底盘、建图与导航

`chassis` 选择底盘节点、模型和**一份底盘 YAML**；公共 M10P、SLAM Toolbox 和
Nav2 参数继续共用。两种底盘的专属运动、EKF、模型、键盘、建图门控及 Nav2 覆盖值分别在：

| 底盘 | `chassis` | 底盘节点 | 底盘 YAML | 默认里程计 |
| --- | --- | --- | --- | --- |
| 三轮全向 | `kiwi` | `kiwi_base_node` | `src/kiwi_base/config/kiwi_base.yaml` | 电机反馈轮式里程计 |
| 两电机履带 | `tracked_pwm` | `tracked_base_node` | `src/kiwi_base/config/tracked_pwm.yaml` | RF2O 激光里程计 |

`robot_state_publisher`、M10P 驱动、EKF、建图门控、SLAM Toolbox、Nav2 和
`kiwi_teleop_keyboard` 均复用已有节点；只需切换底盘 YAML 和底盘驱动节点。
**建图和导航使用相同的 `mode` 命令，只改 `chassis`。**履带的 `/scan` 由 RF2O 生成
`/chassis/odom/laser`，EKF 再融合 `/chassis/imu/data_raw`，输出唯一的
`/odometry/filtered` 和 `odom → base_footprint`；没有编码器里程计。履带建图门控要求
原始激光里程计、IMU 和融合里程计持续新鲜，断流时暂停向 SLAM Toolbox 转发扫描。
履带专属 YAML 还覆盖差速 AMCL、DWB 无横移约束、速度平滑、原地转向平滑器和 0.48 m
避障半径。导航速度及加减速度只是待标定的保守配置，`hardware_calibrated=false` 时不会启动导航。
旧 Kiwi 专用启动文件仍可使用，但下面的命令统一由 `platform.launch.py` 选择。

```bash
# 原 Kiwi：驱动联调 / 建图
ros2 launch kiwi_navigation platform.launch.py chassis:=kiwi mode:=driver
ros2 launch kiwi_navigation platform.launch.py chassis:=kiwi mode:=mapping

# 履带：当前可用 driver；核验传感器并设置 mapping_sensors_verified=true 后可手动建图。
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm mode:=driver
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm mode:=mapping
```

`mode:=mapping` 启动 SLAM Toolbox；Kiwi 或已完成运动标定的履带还会启动 Nav2。
履带若只完成传感器核验、尚未标定速度，则可通过遥控或原始电机话题手动建图，
不会启动 Nav2，也不会把命令当作准确的 m/s。地图稳定后，在另一个已加载工作区的终端保存：

```bash
mkdir -p "$HOME/ros2_ws/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/ros2_ws/maps/warehouse"
```

保存成功后先停止建图进程，再用已有地图定位导航；`map` 必须是存在的绝对 YAML 路径：

```bash
ros2 launch kiwi_navigation platform.launch.py chassis:=kiwi \
  mode:=localization map:="$HOME/ros2_ws/maps/warehouse.yaml"
# 履带完成标定后：
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm \
  mode:=localization map:="$HOME/ros2_ws/maps/warehouse.yaml"
```

**键盘节点只需一份**：它发布 `/cmd_vel`，不直接打开下位机串口。在另一个前台终端运行；
默认命令读取 Kiwi YAML，履带必须显式指定自己的 YAML：

```bash
# Kiwi
ros2 run kiwi_base kiwi_teleop_keyboard
# 履带
ros2 run kiwi_base kiwi_teleop_keyboard --ros-args \
  --params-file "$HOME/ros2_ws/src/kiwi_base/config/tracked_pwm.yaml"
```

履带键盘用 `w/s` 前后、`q/e` 原地转向、空格或 `x` 停车。`a/d` 会发送横移
`linear.y`，履带驱动会拒绝并停车；它们仍是 Kiwi 的左右横移键。当前履带
`hardware_calibrated=false`、`enable_motion=false`，即使键盘已经发布 `/cmd_vel`，
底盘也只发送零电机命令。核验激光里程计、惯导外参和轴向后，可先手动建图；
完成速度换算与空载方向验证后，才能按 [履带标定说明](documents/platform_architecture.md) 开启正常运动和导航。
执行 Nav2 导航目标时不要同时运行键盘；切换模式前先停止旧启动进程。

### 履带开环试车与速度标定

已实测机身长 0.70 m、宽 0.59 m、左右履带中心距 0.50 m，已写入
`src/kiwi_base/config/tracked_pwm.yaml`。机身高度、履带宽高、雷达/惯导外参和速度换算仍待测。
`hardware_calibrated=false`、`enable_motion=false` 保持不变：普通 `/cmd_vel` 和键盘仍不会驱动电机；
`mapping_sensors_verified=false` 时建图和导航仍会被启动校验阻止。

| 条件 | 实际表现 |
| --- | --- |
| `hardware_calibrated=false`、`enable_motion=false` | 底盘节点仍启动、打开 USB、发布 IMU；普通 `/cmd_vel` 输出零电机命令 |
| `enable_motion=true` 但未标定，或标定后 `command_per_m_s` 仍为零 | 底盘节点启动时报错；不能用猜测倍率解锁键盘 |
| `open_loop_test_enabled=true` | 可在 `mode:=driver` 和传感器核验后的手动建图中使用；与正常运动或定位导航不能同时开启 |
| `/dev/robomaster_c` 不可读写、被占用或下位机未切到上位机模式 | 串口无法打开，或固件不执行上位机命令 |
| 命令超过 `cmd_vel_timeout_s=0.20` 秒未刷新 | 上位机持续发送零电机命令；固件还有自己的超时停车逻辑 |

已单独开启供驱动联调及传感器核验后手动建图使用的 `/chassis/open_loop_motor_cmd` 话题，消息为
`std_msgs/msg/Float32MultiArray`，`data` 必须恰好是 `[左履带, 右履带]` 两个固件
**rad/s 命令值**，不是实测履带速度。每路初始上限 ±10 rad/s；非法或超限命令停车，
消息超过 0.20 s 未刷新也停车。此话题和正常 `/cmd_vel` 不能同时使能。

更新工作区并构建 `kiwi_base` 后，先把履带架空、切到上位机控制模式，启动驱动；
在第二个已加载工作区的终端依次短时测试单侧、双侧方向：

```bash
ros2 launch kiwi_navigation platform.launch.py chassis:=tracked_pwm mode:=driver
# 另一个终端，先测左履带；-r 10 -t 10 表示 10 Hz 连发 1 秒。
ros2 topic pub -r 10 -t 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [5.0, 0.0]}'
ros2 topic pub -r 10 -t 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [0.0, 5.0]}'
ros2 topic pub -r 10 -t 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [5.0, 5.0]}'
```

确认两路正方向一致后，地面测量前可把命令改为 `[10.0, 10.0]`，以 10 Hz 持续发布并
计时走 1 m；持续命令示例：

```bash
ros2 topic pub -r 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [10.0, 10.0]}'
```

按 Ctrl+C 停止发布后，0.20 s 内由上位机看门狗发零命令。若实际速度为 `v` m/s，
`10/v` rad/s 只作为下一次试验的换算起点，最终仍需多次实测标定，并受固件
`±20 rad/s` 饱和上限约束。当前固件上位机命令 `0 / +10 / +20 / -20 rad/s`
分别约对应 `1500 / 1750 / 2000 / 1000 µs` 的 PWM4/5 输出；遥控 PWM2/3 满摇杆
输出尚未测量，不能据此推断。已测遥控开机居中约 `1510/1510 µs`、关机后约
`1445/1525 µs`；固件把输入 `1400～1600 µs` 归零到 `1500 µs`，最终输出限在
`1000～2000 µs`。手动建图前须核验雷达/惯导安装位姿、IMU 轴向及 RF2O 静止漂移，
再把同一 YAML 中的 `mapping_sensors_verified` 改为 `true`。正常键盘控制和导航仍要等
`command_per_m_s` 等参数标定、关闭 `open_loop_test_enabled` 后再开启。

### 网页控制台（当前适用于 Kiwi 底盘）

`kiwi_web` 的服务和静态页面都在本机运行，不需要 Node.js、CDN 或 rosbridge。
`rosdep` 会按 `src/kiwi_web/package.xml` 安装 Python 运行依赖。上面的完整构建完成后：

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
mkdir -p maps
ros2 launch kiwi_web web.launch.py
```

另一个终端运行 `hostname -I` 查看上位机地址，在同一局域网浏览器打开
`http://上位机IP:8080`。默认监听 `0.0.0.0:8080`；若启用防火墙，需允许可信局域网访问
TCP 8080。网页启动时不会自动启动电机、雷达或导航，需在页面选择模式。
已有地图可把旧工作区 `maps/` 中的 YAML、PGM 及 `.versions/`、`.routes/` 一起迁入新机的
`$HOME/ros2_ws/maps/`。访问密码和端口在 `src/kiwi_web/config/web.yaml` 配置。

网页默认使用 `ROS_DOMAIN_ID=42`；从命令行检查其话题时，在检查终端先执行
`export ROS_DOMAIN_ID=42`。网页运行时不要另起底盘、键盘或 Nav2 速度发布者。
当前 Web 的 `stack.launch.py` 使用原 Kiwi 建图入口，**尚未适配两电机履带底盘**；
履带底盘请使用 [统一平台启动入口](documents/platform_architecture.md)。

需要登录后自动启动网页服务时，在目标上位机构建并加载工作区后执行：

```bash
ros2 run kiwi_web kiwi_web_install_service
systemctl --user daemon-reload
systemctl --user enable --now kiwi-web.service
journalctl --user -u kiwi-web.service -f
```

无人登录也要启动时，需由设备管理员启用该用户的 linger。完整的页面功能、地图管理和
服务配置见 [kiwi_web 文档](src/kiwi_web/README.md)。

## 5. 只启动底盘和 EKF

适合先确认 USB 命令、异步电机/IMU 反馈、轮式里程计和 TF，不启动雷达和 SLAM：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

检查输出：

```bash
ros2 topic hz /chassis/odom/raw
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /odometry/filtered
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

0x3A 命令没有 ACK，发送链不等待反馈；独立的 0x3B 遥测同时提供滤波后的电机实测速度
以及绝对 yaw/`gyro_z`。EKF 用后两者修正航向。该链路仍不能从 IMU 单独恢复打滑造成的 x/y 位移误差；
室外激光特征不足时，需要 RTK/GNSS、视觉惯导或其他绝对平移观测才能约束累计漂移。

## 6. 二维建图流程

### 6.1 启动建图系统

推荐使用组合启动文件，它会启动底盘、EKF、M10P、SLAM Toolbox 和 Nav2：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
```

如果只想建图、不启动 Nav2，可以使用：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

默认配置文件为：

```text
底盘/EKF：src/kiwi_base/config/kiwi_base.yaml
M10P：    src/kiwi_slam/config/ls_m10p_uart.yaml
SLAM：    src/kiwi_slam/config/slam_toolbox.yaml
Nav2：    src/kiwi_navigation/config/nav2.yaml
```

### 6.2 开始移动建图

在第二个终端运行键盘节点（必须使用前台可交互终端）：

```bash
cd $HOME/ros2_ws
source tools/activate_workspace.sh
ros2 run kiwi_base kiwi_teleop_keyboard
```

按键：

```text
W/S  前进/后退       A/D  左/右横移
Q/E  逆时针/顺时针   空格或 X 立即停止
Ctrl-C              退出并发布零速度
```

建图时低速、匀速覆盖所有走廊和房间，避免快速旋转、急停和长时间无激光回波。确认：

```bash
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

### 6.3 保存地图

另开终端创建固定地图目录，然后保存：

```bash
mkdir -p $HOME/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f $HOME/ros2_ws/maps/warehouse
```

命令成功后会生成：

```text
$HOME/ros2_ws/maps/warehouse.yaml
$HOME/ros2_ws/maps/warehouse.pgm
```

保存前应停止机器人并等待地图稳定。`.yaml` 是导航启动入口，`.pgm` 是地图图像，不能只保留
其中一个文件。保存后可检查：

```bash
ls -lh $HOME/ros2_ws/maps/warehouse.*
```

保存完毕后在建图终端按 `Ctrl-C`，再停止键盘终端。

## 7. 已知地图定位和 Nav2 导航

### 7.1 选择地图

启动导航时必须传入要使用的地图 YAML，不能传 `.pgm`，也不能省略 `map`：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/ceshi3.yaml \
  use_rviz:=true
```

例如另一张地图：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/lab.yaml
```

该 launch 会启动底盘、EKF、M10P、Nav2 的 AMCL、map_server、costmap、规划器、DWB 全向控制器
和 RViz；不会启动 SLAM Toolbox。导航时地图来源是 `map` 参数指向的 YAML 文件。

### 7.2 在 RViz 中定位和导航

1. 等待 AMCL、map_server 和 TF 正常启动。
2. 在 RViz 点击 `2D Pose Estimate`，在地图上点击机器人当前位置并拖出朝向。
3. 等待激光扫描与地图重合，必要时重新设置初始位姿。
4. 使用 `Nav2 Goal` 或 `2D Goal Pose` 设置目标点。
5. 观察规划路径、局部代价地图和 `/cmd_vel`，确认机器人按期望方向运动。

自主导航期间不要启动键盘节点，否则键盘和 Nav2 会同时向 `/cmd_vel` 发布命令。

## 8. 诊断命令

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
ros2 topic echo /chassis/odom/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic echo /odometry/filtered
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

启用 SLAM 诊断节点时：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py start_monitor:=true
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"
ros2 topic echo /diagnostics
```

常见故障：

- 没有 `/scan`：检查 `/dev/wheeltec_lidar`、`ls_m10p_uart.yaml` 的 `serial_port` 和雷达供电。
- 没有 `/odometry/filtered`：检查 0x3B 遥测、底盘 `/chassis/odom/raw` 及 EKF 参数。
- 地图不更新：同时检查 `/scan`、`map -> odom`、`odom -> base_footprint` 和激光安装 TF。
- 键盘无效：确认键盘节点在前台 TTY 中运行，并且没有 Nav2 同时占用 `/cmd_vel`。
- 方向错误：ROS 约定是 `vx+` 前进、`vy+` 左移；应检查电机编号/符号和实车前方标定，不要重复旋转 TF。

## 9. 主要参数入口

| 功能 | 文件 |
| --- | --- |
| 底盘、运动学、反馈里程计、EKF、键盘 | `src/kiwi_base/config/kiwi_base.yaml` |
| M10P 串口和 `/scan` | `src/kiwi_slam/config/ls_m10p_uart.yaml` |
| SLAM Toolbox | `src/kiwi_slam/config/slam_toolbox.yaml` |
| SLAM 诊断 | `src/kiwi_slam/config/monitor.yaml` |
| Nav2、AMCL、DWB、代价地图 | `src/kiwi_navigation/config/nav2.yaml` |
| 机器人结构和传感器 TF | `src/kiwi_slam/urdf/kiwi.urdf.xacro` |
| 地图 | `$HOME/ros2_ws/maps/*.yaml` 和对应 `.pgm` |

修改参数后重新构建相关包并重新 `source install/setup.bash`。地图文件不需要重新构建，直接在
启动导航时通过 `map:=绝对路径` 选择。
