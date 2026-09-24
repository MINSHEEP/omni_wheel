# Kiwi ROS 2 移动机器人工作区

同一份 `src/kiwi_platform/config/active_platform.yaml` 控制底盘、建图、定位导航、键盘和 Web 的参数选择。三轮 Kiwi 与两电机履带分别使用自己的参数文件和里程计链路；详细契约见 [平台架构](documents/platform_architecture.md)。

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
│   ├── kiwi_platform/   活动底盘开关、底盘档案与统一参数校验
│   ├── kiwi_base/       底盘、运动学、里程计、IMU、键盘
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
/chassis/odom/raw        仅 Kiwi：根据电机反馈推算的轮式里程计
/chassis/odom/laser      仅履带：RF2O 激光里程计（不发布 TF）
/chassis/imu/data_raw    C 板异步 IMU 遥测；履带仅使用角速度，姿态暂标为未知
/chassis/feedback/raw    C 板异步 0x3B 原始字段调试消息
/joint_states            仅 Kiwi：根据电机实测速度换算的三个轮关节状态
/odometry/filtered       EKF 输出，Nav2 使用此话题
/map                     SLAM Toolbox 或 map_server 发布的地图
```

TF 链应为：

```text
建图：       map -> odom -> base_footprint -> base_link -> lidar_link
已知图导航： map -> odom (AMCL) -> base_footprint (EKF) -> base_link -> lidar_link
```

活动底盘档案会指定同一底盘的底盘/EKF、键盘、建图门控、SLAM、Nav2 与 URDF 文件。键盘速度受底盘 `max_*` 限制；Nav2 速度另由所选底盘的 Nav2 YAML 限制。

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
bash tools/apply_rf2o_patch.sh
```

`vcs import` 从 `tools/platform_dependencies.repos` 获取履带底盘的 RF2O 固定版本，
需要访问 GitHub；只用 Kiwi 底盘时可跳过此行。项目源码已包含 M10P 的
`lslidar_driver` 和 `lslidar_msgs`。

履带部署在导入 RF2O 后运行 `bash tools/apply_rf2o_patch.sh`（已有源码也需要执行，
可重复运行）。补丁让 RF2O 等到雷达安装 TF 可用后再初始化，并修复停止时的
guard condition 异常；两帧扫描之间的正常等待改用 DEBUG 日志。补丁保存在
`tools/patches/`，依赖仍使用上述固定版本，应用后需要重新构建。

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
overlay 最前面，并校验 `kiwi_platform`、`kiwi_base`、`kiwi_slam`、`kiwi_navigation` 的实际来源；这可避免
EKF 因误读旧 YAML 而静默回退到没有 `odom0/imu0` 的默认配置。

## 4. 构建和测试

依赖安装和 udev 配置完成后，从工作区根目录构建导航、M10P 驱动、Web 控制台和履带 RF2O：

```bash
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
bash tools/apply_rf2o_patch.sh  # 履带需要；仅使用 Kiwi 时跳过
colcon build --symlink-install --packages-up-to kiwi_web rf2o_laser_odometry
source tools/activate_workspace.sh
```

不使用履带时可把构建命令末尾的 `rf2o_laser_odometry` 去掉；不部署网页时，
将 `kiwi_web` 换成 `kiwi_navigation`。构建后先检查包来源和设备：

```bash
ros2 pkg prefix kiwi_platform
ros2 pkg prefix kiwi_navigation
ros2 pkg prefix lslidar_driver
ros2 pkg prefix kiwi_web  # 部署网页时
ros2 pkg prefix rf2o_laser_odometry  # 仅履带需要
ros2 run kiwi_slam kiwi_slam_compatibility --require-hardware  # 当前活动底盘设备检查
```

要单独检查雷达驱动，在一个终端运行 `ros2 launch kiwi_slam m10p_uart_lidar.launch.py`，
另一个已加载工作区的终端运行 `ros2 topic hz /scan`。底盘联调和建图导航的完整启动命令见
[双底盘平台架构](documents/platform_architecture.md)。履带直线倍率已初步测得，但低速、转向和传感器外参仍未核验；活动 YAML 为 `tracked_pwm` 时可启动驱动并在受限模式试车，建图导航仍保持关锁。

只构建底盘驱动时可执行下列命令，并用 `source install/setup.bash` 加载；这时可启动 `kiwi_base.launch.py` 和键盘，建图/导航及履带 RF2O 仍需上面的完整构建。

```bash
colcon build --symlink-install --packages-up-to kiwi_base
source install/setup.bash
```

运行测试：

```bash
export ROS_LOG_DIR=/tmp/kiwi_ros_logs
colcon test --packages-select kiwi_platform kiwi_base kiwi_slam kiwi_navigation kiwi_web \
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

### 统一选择文件与独立启动入口

只改一处活动底盘开关，重启正在运行的 ROS/Web 进程。以下命令以已构建且使用 `--symlink-install` 的工作区为例；普通安装需重新构建 `kiwi_platform` 才能把修改后的 YAML 安装过去。

```bash
cd "$HOME/ros2_ws"
# 两电机履带
sed -i 's/^chassis:.*/chassis: tracked_pwm/' src/kiwi_platform/config/active_platform.yaml
# 切回 Kiwi
sed -i 's/^chassis:.*/chassis: kiwi/' src/kiwi_platform/config/active_platform.yaml
source tools/activate_workspace.sh
```

`kiwi_platform/config/profiles/{kiwi,tracked_pwm}.yaml` 给每个底盘指定对应的完整参数文件。启动入口各自读取活动开关，不能把履带底盘和 Kiwi 导航参数静默混用：

| 链路 | Kiwi | 两电机履带 |
| --- | --- | --- |
| 底盘/EKF/键盘 | `kiwi_base/config/kiwi_base.yaml` | `kiwi_base/config/tracked_pwm.yaml` |
| 建图门控 | `kiwi_slam/config/mapping_gate.yaml` | `kiwi_slam/config/tracked_mapping_gate.yaml` |
| SLAM Toolbox | `kiwi_slam/config/slam_toolbox.yaml` | `kiwi_slam/config/tracked_slam_toolbox.yaml` |
| 里程计 | 电机反馈 `/chassis/odom/raw` | `kiwi_slam/config/tracked_rf2o.yaml`，RF2O `/chassis/odom/laser` |
| Nav2/AMCL/代价地图 | `kiwi_navigation/config/nav2.yaml` | `kiwi_navigation/config/tracked_nav2.yaml` |
| 结构与外参 | `kiwi_slam/urdf/kiwi.urdf.xacro` | `kiwi_slam/urdf/tracked.urdf.xacro` |
| 雷达/惯导 | M10P `/scan`，C 板 IMU | M10P `/scan`，C 板 IMU；更换型号需新增驱动及档案 |

底盘、建图、导航分别有独立命令；命令本身对两种底盘相同：

```bash
# 驱动和 URDF：履带直线初测倍率已记录，运动关锁仍在，可打开串口读取 IMU。
ros2 launch kiwi_slam kiwi_robot.launch.py start_ekf:=false start_lidar:=false
# 底盘 + 里程计 + EKF；履带会额外启动 M10P、RF2O。
ros2 launch kiwi_base kiwi_base_ekf.launch.py
# 只建图，不启动 Nav2；履带适合先手动建图。
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
# 建图并在具备运动和传感器核验时启动 Nav2；当前履带仍由核验关锁阻止。
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
# 已知地图定位导航；履带运动或传感器未核验时拒绝启动。
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:="$HOME/ros2_ws/maps/warehouse.yaml" use_rviz:=true
```

建图开始前，履带必须先核验雷达与惯导外参、IMU 轴向、RF2O 静止漂移，再把 `tracked_pwm.yaml` 的 `mapping_sensors_verified` 设为 `true`。当前仅有两档直线计时，已写入初步 `command_per_m_s`，但低速起步、反向和转向尚未验证，`hardware_calibrated=false`、`enable_motion=false` 继续锁住正常 `/cmd_vel`。受限试车可用现场遥控器或临时键盘模式；原始电机话题可按下文切换。完成剩余核验后才能启用正常运动和导航；仅改 Nav2 参数不能解除运动锁。RF2O 只发里程计消息，EKF 独占 `odom → base_footprint`；SLAM Toolbox 或 AMCL 发布 `map → odom`。

同一个键盘节点使用同一个命令，并自动读取活动底盘的参数：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

键盘 `w/s` 前后、`q/e` 转向、空格或 `x` 停车。`a/d` 只适用于 Kiwi 横移；履带驱动会拒绝横移并停车。网页控制台也会把履带横移上限设为零。导航时不要同时运行键盘。地图完成后保存：

```bash
mkdir -p "$HOME/ros2_ws/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/ros2_ws/maps/warehouse"
```

### 履带临时键盘试车与速度标定

机身长 0.70 m、宽 0.59 m、左右履带中心距 0.50 m 已实测。直线计时：命令 `5` 走 1.6 m 平均用 6.85 s，实速约 0.2336 m/s、倍率 21.406；命令 `10` 用 3.37 s，实速约 0.4748 m/s、倍率 21.063。两档均值约 21.234，`tracked_pwm.yaml` 先记 `command_per_m_s=21.2`。这不是电机转速闭环；雷达、IMU 相对 `base_link` 的安装位姿分别记录为 `(0.24,0,0)` m 与 `(0.08,0,0)` m、旋转均为零，仍需用 TF 和实测数据核对。低速死区、反向、转向、IMU 轴向及 RF2O 漂移仍待测。`keyboard_trial_enabled=true` 已恢复，`open_loop_test_enabled=false`；正常运动仍锁定；建图和导航仍被传感器核验关锁阻止。

临时倍率 `keyboard_trial_command_gain=50.0` 只把键盘前进刻度 `0.10` 换成两路各 `+5 rad/s` 的固件命令；`keyboard_trial_limit_rad_s=5.0` 将每路限制在 ±5 rad/s。此时键盘参数名中的 `m_s` **不代表实际 m/s**；只有剩余核验完成并启用正常运动后，`command_per_m_s=21.2` 才参与 `/cmd_vel` 换算。固件命令 `+10` 对应约 1750 µs，并非电机实测角速度。

将更新后的源码同步到上位机后，从工作区根目录重新构建底盘及其配置选择包：

```bash
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
sed -i 's/^chassis:.*/chassis: tracked_pwm/' src/kiwi_platform/config/active_platform.yaml
colcon build --symlink-install --packages-up-to kiwi_base
```

先架空履带并确认下位机切到上位机模式。在两个终端都加载同一工作区，且不要同时运行 Web、Nav2 或其他 `/cmd_vel` 发布者：

```bash
# 终端 1：只启动当前所选底盘节点，不需要先启动雷达、RF2O 或 EKF。
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kiwi_base kiwi_base.launch.py
```

```bash
# 终端 2：必须是能接收按键的前台终端。
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

键盘 `w/s` 前后、`q/e` 原地转向、空格或 `x` 停车；`a/d` 横移会被履带驱动拒绝并停车。先短按 `w` 核实两条履带的正方向，再短按 `q/e` 核实转向。停止按键约 200 ms 后键盘自动发布零命令；速度命令断发超过 `cmd_vel_timeout_s=0.20` 秒时，底盘节点持续发零电机命令。按 `Ctrl-C` 退出键盘，再停止驱动。若日志出现 `Cannot open /dev/robomaster_c`，检查串口链接和 `dialout` 权限；串口已打开但不动，检查下位机控制模式与电调状态。

| 配置条件 | 实际表现 |
| --- | --- |
| `keyboard_trial_enabled=true`，`hardware_calibrated=false` | 键盘试车可用，每路受 `keyboard_trial_limit_rad_s` 限制；不解锁导航 |
| `keyboard_trial_enabled=false`、`open_loop_test_enabled=false`、`enable_motion=false` | 普通 `/cmd_vel` 只输出零电机命令 |
| `open_loop_test_enabled=true`、`keyboard_trial_enabled=false` | 原始电机话题可用；与临时键盘及正常运动互斥 |
| 已实测速度并设 `hardware_calibrated=true`、`enable_motion=true`，关闭两种试车模式 | 才可使用真实标定的 `/cmd_vel`；导航仍要求核验传感器和定位 |

若需要旧的 `/chassis/open_loop_motor_cmd` 双路单侧试车，先停止底盘节点，在同一份 `tracked_pwm.yaml` 中把 `keyboard_trial_enabled` 改为 `false`，把 `open_loop_test_enabled` 改为 `true`，再重启底盘。该话题消息为 `std_msgs/msg/Float32MultiArray` 的 `[左履带, 右履带]` 两个固件 rad/s 数值，当前每路上限 ±10 rad/s；非法/超限命令与断发超过 0.20 秒都会停车。架空履带时的短时命令示例：

```bash
ros2 topic pub -r 10 -t 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [5.0, 0.0]}'
ros2 topic pub -r 10 -t 10 /chassis/open_loop_motor_cmd std_msgs/msg/Float32MultiArray '{data: [0.0, 5.0]}'
```

两档直线数据接近，但只能说明中等速度范围的初步倍率。下一步需测命令低于 `5` 时的起步与停车、反向、左右履带差异和原地转向的实际角速度；履带打滑会使有效转向间距不同于实测几何中心距 0.50 m。核验后再考虑将 `hardware_calibrated`、`enable_motion` 设为 `true` 并关闭两种试车模式。当前 Nav2 直线速度上限 0.15 m/s 按 21.2 倍率只产生约 3.18 的固件命令，低于已测最低档 `5`，可能无法起步。雷达/惯导外参与 RF2O 尚未核验时，保持 `mapping_sensors_verified=false`。

### 网页控制台（跟随活动底盘）

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
Web 通过相同活动底盘文件选择底盘、建图和导航参数；履带横移命令被拒绝。Kiwi 的就绪状态检查轮式反馈和 IMU 标定，履带改为检查 RF2O 激光里程计与实时 IMU。切换底盘或修改底盘标定 YAML 后需重启 Web 服务。履带只记录了两档直线初测倍率，正常运动仍锁定；Web 手动遥控保持未就绪，Web 导航也会在启动前被传感器核验关锁拒绝。

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

适合检查底盘和 EKF。Kiwi 使用电机反馈；履带会另外启动 M10P 与 RF2O，以生成激光里程计：

```bash
ros2 launch kiwi_base kiwi_base_ekf.launch.py
```

检查输出：

```bash
ros2 topic hz /chassis/odom/raw    # 仅 Kiwi
ros2 topic hz /chassis/odom/laser  # 仅履带
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /odometry/filtered
ros2 topic echo /joint_states --once  # 仅 Kiwi
ros2 run tf2_ros tf2_echo odom base_footprint
```

0x3A 命令没有 ACK，发送链不等待反馈。Kiwi 的 0x3B 遥测提供电机实测速度和惯导数据；履带的 0x3B 电机字段为零，不进入里程计，只有 IMU 角速度参与 EKF。该链路仍不能从 IMU 单独恢复打滑造成的 x/y 位移误差；
室外激光特征不足时，需要 RTK/GNSS、视觉惯导或其他绝对平移观测才能约束累计漂移。

## 6. 二维建图流程

### 6.1 启动建图系统

组合启动文件会启动所选底盘、EKF、M10P 和 SLAM Toolbox；Kiwi 或已完成运动标定的履带还会启动 Nav2：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py use_rviz:=true
```

如果只想建图、不启动 Nav2，可以使用：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

实际参数文件由本页上方的活动底盘档案选择；切换后重新启动对应 launch。

### 6.2 开始移动建图

在第二个终端运行键盘节点（必须使用前台可交互终端）：

```bash
cd $HOME/ros2_ws
source tools/activate_workspace.sh
ros2 run kiwi_base kiwi_teleop_keyboard
```

按键：

```text
W/S  前进/后退       A/D  左/右横移（仅 Kiwi）
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

该 launch 会启动所选底盘、对应里程计与 EKF、M10P、AMCL、地图服务器、Nav2 和 RViz；履带使用差速模型且不允许横移。不会启动 SLAM Toolbox。导航时地图来源是 `map` 参数指向的 YAML 文件。

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
ros2 topic echo /chassis/odom/raw    # 仅 Kiwi
ros2 topic echo /chassis/odom/laser  # 仅履带
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
- 没有 `/odometry/filtered`：Kiwi 检查 0x3B 轮速反馈；履带检查 `/scan`、`/chassis/odom/laser` 和 IMU，再检查 EKF 参数。
- 地图不更新：同时检查 `/scan`、`map -> odom`、`odom -> base_footprint` 和激光安装 TF。
- 键盘无效：确认键盘节点在前台 TTY 中运行，并且没有 Nav2 同时占用 `/cmd_vel`。
- 方向错误：ROS 约定是 `vx+` 前进、`vy+` 左移；履带无 `vy`。检查电机编号/符号和实车前方标定。

## 9. 主要参数入口

活动开关是 `src/kiwi_platform/config/active_platform.yaml`；每个底盘的文件映射在 `src/kiwi_platform/config/profiles/`。上面的表格列出所有底盘专属参数文件。M10P 串口配置目前共用 `src/kiwi_slam/config/ls_m10p_uart.yaml`；Web 服务配置是 `src/kiwi_web/config/web.yaml`。地图保存在 `$HOME/ros2_ws/maps/`，导航时使用 `map:=绝对路径.yaml`。

使用 `--symlink-install` 时修改 YAML 后重启进程即可；普通安装需要重新构建对应包再加载工作区。实车参数仍须按 [平台架构与待标定项](documents/platform_architecture.md) 核验。
