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

### 3.1 安装 Ubuntu、ROS 2 和构建工具

使用 Ubuntu 22.04（Jammy，`amd64` 或 `arm64`）和 ROS 2 Humble；先用 `locale` 确认系统使用 UTF-8。
以下命令遵循 [ROS 2 Humble 官方 deb 安装步骤](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。
若机器已有 `/opt/ros/humble/setup.bash`，跳过 ROS 2 仓库配置和安装部分。

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools git python3-rosdep python3-vcstool python3-serial usbutils
```

`ros-humble-desktop` 包含 RViz；无图形界面时可改装 `ros-humble-ros-base`，
但本 README 中 `use_rviz:=true` 的命令需要桌面环境。雷达驱动源码直接使用 PCL、Boost、
Eigen、`yaml-cpp` 和 `libpcap`；`kiwi_web` 需要 `aiohttp`、Pillow 和 `psutil`。
安装对应系统包，再由 `rosdep` 补齐其余 ROS 包依赖：

```bash
sudo apt install -y libpcl-dev libboost-thread-dev libeigen3-dev libyaml-cpp-dev libpcap-dev
sudo apt install -y python3-aiohttp python3-pil python3-psutil
source /opt/ros/humble/setup.bash
```

### 3.2 获取项目与依赖源码

以下以新机上的 `~/ros2_ws` 为例；若已有授权的工作区副本，在其根目录切换到 `test` 分支即可。
克隆前，源机器需先把本地 `test` 分支发布到远端：`git push -u origin test`；
Gitee 私有仓库还需配置访问权限。不要复制旧机器的 `build/`、`install/`。

```bash
ROS_WS="$HOME/ros2_ws"
git clone --branch test https://gitee.com/yonheybot/omni-wheel-arm-chassis.git "$ROS_WS"
cd "$ROS_WS"
vcs import src < tools/platform_dependencies.repos
test -f /etc/ros/rosdep/sources.list.d/20-default.list || sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys cmake_modules -y
```

若 `sudo rosdep init` 因 `raw.githubusercontent.com` TLS 超时而失败，先停止后续
`rosdep update/install`。可按[清华 TUNA 的 GitHub Raw 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/github-raw/)
和[ROS Distro 镜像说明](https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/)
手动建立数据源，并同时切换 rosdistro 索引。下面的三个 YAML 是 Ubuntu 所需的
rosdep 通用规则，不再运行会访问 GitHub Raw 的 `sudo rosdep init`：

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

以后在新终端再次执行 `rosdep update` 前，需重新设置上述 `ROSDISTRO_INDEX_URL`；
如果镜像也无法访问，先检查网络和代理，不要用关闭 TLS 校验的方式绕过。

`vcs import` 按 `tools/platform_dependencies.repos` 获取履带激光里程计 RF2O 的固定版本；
它需要访问 GitHub。项目内的 `lslidar_driver`、`lslidar_msgs` 是 M10P 的源码驱动，
无需另装同名 deb 包。RF2O 上游 `package.xml` 中有未被 CMake 使用的旧 `cmake_modules`
键，Humble 下无对应 rosdep 定义，因此上面的命令只跳过该键。若不使用履带底盘，
可以跳过 RF2O 导入，并在构建命令中去掉它。

### 3.3 识别雷达和下位机 USB 串口

连接 M10P 和 C 板，用设备属性辨认端口，**不要依据 `ttyACM0/1` 的顺序判断**：

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/serial/by-path/
for port in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e "$port" ]] || continue
  echo "$port"
  udevadm info --query=property --name="$port" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)='
done
```

当前 M10P USB 转串口芯片为 WCH `1a86:55d4`，通常枚举成 `ttyACM*`。安装稳定名称
`/dev/wheeltec_lidar`；若实测 VID/PID 不同，先按 `lsusb`/`udevadm` 输出调整规则：

```bash
sudo tee /etc/udev/rules.d/99-wheeltec-lidar.rules >/dev/null <<'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="wheeltec_lidar", GROUP="dialout", MODE="0660"
RULE
```

C 板 USB CDC 也通常是 `ttyACM*`。根据 `/dev/serial/by-id/` 和 `udevadm` 输出，
把下面第一行改为**实际 C 板端口**；规则同时匹配该板的 VID、PID 和唯一序列号，
生成 `/dev/robomaster_c`：

```bash
C_BOARD_TTY=/dev/ttyACM1  # TODO：替换为刚确认的 C 板设备，不能照抄编号
C_BOARD_VENDOR_ID="$(udevadm info --query=property --name="$C_BOARD_TTY" | sed -n 's/^ID_VENDOR_ID=//p')"
C_BOARD_PRODUCT_ID="$(udevadm info --query=property --name="$C_BOARD_TTY" | sed -n 's/^ID_MODEL_ID=//p')"
C_BOARD_SERIAL="$(udevadm info --query=property --name="$C_BOARD_TTY" | sed -n 's/^ID_SERIAL_SHORT=//p')"
if [[ -z "$C_BOARD_VENDOR_ID" || -z "$C_BOARD_PRODUCT_ID" || -z "$C_BOARD_SERIAL" ]]; then
  echo 'C 板缺少 VID/PID/序列号；检查设备，或使用 /dev/serial/by-path/ 的稳定名称。' >&2
else
  printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", ATTRS{serial}=="%s", SYMLINK+="robomaster_c", GROUP="dialout", MODE="0660"\n' \
    "$C_BOARD_VENDOR_ID" "$C_BOARD_PRODUCT_ID" "$C_BOARD_SERIAL" | sudo tee /etc/udev/rules.d/99-robomaster-c.rules
fi
```

若 C 板没有 `ID_SERIAL_SHORT`，可直接在底盘 YAML 中把 `device` 改成
`/dev/serial/by-path/` 下对应的稳定链接（换 USB 物理口后需重新核对），不用创建 C 板规则。
同型号 WCH 设备有多个时，雷达规则也需加 `ATTRS{serial}` 或使用对应的 `by-path` 链接。

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo usermod -aG dialout "$USER"
# 注销并重新登录后，在新终端执行：
id -nG | grep -w dialout
ls -l /dev/wheeltec_lidar /dev/robomaster_c
```

若使用 `by-path` 方案，最后一行只检查实际使用的设备。雷达路径配置在
`src/kiwi_slam/config/ls_m10p_uart.yaml` 的 `serial_port`；Kiwi 和履带底盘路径分别配置在
`src/kiwi_base/config/kiwi_base.yaml`、`src/kiwi_base/config/tracked_pwm.yaml` 的 `device`。
M10P 的串口速率 500000 由驱动按型号设置；C 板 USB CDC 不需要设置波特率。

每次新终端先加载环境：

```bash
cd "$HOME/ros2_ws"
source tools/activate_workspace.sh
```

不要在同一终端最后加载含旧 `kiwi_base` 的其他工作区。激活脚本会把当前工作区放到
overlay 最前面，并校验 `kiwi_base`、`kiwi_slam`、`kiwi_navigation` 的实际来源；这可避免
EKF 因误读旧 YAML 而静默回退到没有 `odom0/imu0` 的默认配置。

## 4. 构建和测试

首次或源码变更后，从工作区根目录构建导航链路、M10P 驱动、Web 控制台和履带 RF2O：

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
`chassis:=tracked_pwm mode:=driver`，默认禁用运动，不能直接启动建图或导航。

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
