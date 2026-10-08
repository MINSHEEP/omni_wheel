# MID-360S / FAST-LIO2 部署与现场核验

本文针对 Ubuntu 22.04、ROS 2 Humble 和当前工作区的 **tracked_pwm 履带底盘**。实机外壳和 SDK2 发现报文均确认为 MID-360S（设备类型 35），使用内置 IMU；C 板 IMU 话题和底盘通信接口保留，但不进入这条定位链路。2026-09-27 核查 `a@192.168.66.112`：连接雷达的有线网卡是 `enp2s0`，雷达 IP 是 `192.168.1.117`。原先主机也配置为 `.117`，形成 IP 冲突；2026-09-27 已将主机该网卡改为 `192.168.1.50/24` 并持久保存，Wi‑Fi `wlo1=192.168.66.112/24` 和默认路由未改变。`.50 → .117` 的三次 ICMP 测试全部成功；现场已测得点云约 10 Hz、IMU 约 200 Hz、FAST-LIO2 里程计和二维投影扫描均约 10 Hz。这些是历史静止接口检查，尚未完成底盘运动及地图质量验收。2026-10-08 统一工作区整合按用户确认的“仅上位机、未连接雷达和 C 板”条件验收软件，`enp2s0` 为 DOWN、没有 IPv4 地址，且 `/dev/robomaster_c` 不存在属于预期条件。

## 1. 先确认实机网络和安装条件

Livox [MID-360 用户手册](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_CHS.pdf)说明：出厂静态地址是 `192.168.1.1XX/24`，`XX` 为序列号最后两位；地址可被修改。本站点已从 ARP 和独立 ICMP 验证 `.117` 的确在有线网上，官方配置文件示例 `.12` 不适用。主机连接雷达的网卡须有同子网地址，且不能与雷达、网关或其他设备冲突。优先用独立雷达网卡，保持 SSH/控制网卡原有地址与默认路由。本站点在 2026-09-27 验证过此设置；当前网卡离线时，应先检查网线、雷达供电及既有网络连接配置，再确认地址和路由。下方临时改 IP 命令供维护参考，不应在地址已经正确时重复执行。

```bash
ip -br link
ip -br -4 addr
ip -4 route
# 确认网卡名称及雷达实际地址后，再输入现场实值：
read -rp '雷达网卡名：' LIDAR_IF
read -rp '实际雷达 IP：' LIDAR_IP
read -rp '主机该网卡待添加的地址/前缀：' HOST_CIDR
# 临时添加地址验证；重启后失效，不会替换其他网卡的 IP。
sudo ip addr add "$HOST_CIDR" dev "$LIDAR_IF"
ping -I "$LIDAR_IF" -c 3 "$LIDAR_IP"
ip route get "$LIDAR_IP"
```

永久地址应写入该网卡既有的 NetworkManager/netplan 配置，并在维护窗口核对 SSH 路由；不要把雷达子网的默认网关覆盖现有远程连接。若实机地址已改到其他网段，主机地址须跟随实机网段，而非固定套用 `192.168.1.0/24`。雷达供电要求为 9–27 V DC，以太网口不能接 PoE；线缆、供电和网口连接也要核对用户手册。

## 2. 统一工作区构建

[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)和 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)均由 Livox 提供；旧的 `livox_ros_driver` / Livox-SDK 不适用 MID-360。HKU-MARS 的 [FAST_LIO 主仓库](https://github.com/hku-mars/FAST_LIO)是 ROS 1 包，本工作区使用 [Ericsii/FAST_LIO_ROS2 的 `ros2` 分支](https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2)快照，这是第三方移植。上游来源、固定 commit、许可证及本地构建调整见 [第三方依赖记录](../../third_party/README.md)。

当前远程统一工作区为 `~/ros2_ws_mid360`，目录分工如下：

```text
ros2_ws_mid360/
├── src/
│   ├── livox_ros_driver2/    # ROS 2 消息、共享组件与驱动节点
│   ├── fast_lio/             # FAST-LIO2 ROS 2 移植包
│   └── kiwi_*/               # 平台、底盘、建图、导航与 Web
├── third_party/
│   └── livox_sdk2/           # SDK 源码，含 COLCON_IGNORE
├── maps/                     # YAML/PGM、PCD、.versions、.routes
├── build/                    # colcon 生成目录
├── install/
└── log/
```

`COLCON_IGNORE` 使 colcon 不将 SDK 目录发现为独立包；`buildignore` 不是 colcon 的有效标记。`livox_ros_driver2` 的 CMake 会从 `third_party/livox_sdk2` 编译静态 SDK，并将它链接进驱动共享组件。SDK 构建产物位于 `build/livox_ros_driver2/`，无需 `sudo cmake --install`、`ldconfig` 或 `/usr/local/lib/liblivox_lidar_sdk_shared.so`。ROS 功能包仍按各自类型产生可执行程序、消息库和共享组件；`src/` 表示 ROS 包边界，不能将其中所有代码都视作动态库。

首次构建或部署到新机器时，使用未加载其他 colcon 工作区的终端，安装 Humble 系统依赖后直接构建：

```bash
cd "$HOME/ros2_ws_mid360"
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git cmake build-essential libeigen3-dev libpcl-dev python3-vcstool
if [ ! -d src/rf2o_laser_odometry ]; then
  vcs import src < tools/platform_dependencies.repos
  bash tools/apply_rf2o_patch.sh
fi
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys=cmake_modules
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source tools/activate_workspace.sh
mkdir -p "$KIWI_WORKSPACE_ROOT/maps"
colcon test --packages-select livox_ros_driver2 kiwi_base kiwi_platform lslidar_driver kiwi_slam kiwi_navigation kiwi_web \
  --event-handlers console_direct+
colcon test-result --verbose
```

RF2O 是保留的 M10P 链路依赖，按固定版本导入同一个 `src/`，不需要另一个工作区。
`cmake_modules` 是它的上游清单中不再使用的 ROS 1 字段，当前 CMake 直接查找 Eigen3，
因此 rosdep 跳过此键。其他依赖照常安装。

本仓库已固定 ROS 2 Humble 的 CMake 和包清单，直接使用上述 `colcon build`。
当前 `src/livox_ros_driver2/build.sh humble` 也是非破坏的统一工作区构建入口，只构建驱动及其依赖；
它不会像原上游脚本那样清空整个工作区或替换清单。

加载脚本先清理先前登记的其他 colcon 工作区路径，再加载 Humble 和当前 `install/local_setup.bash`，避免重载旧 overlay。它会核对 `livox_ros_driver2`、`fast_lio`、`kiwi_platform`、`kiwi_base`、`kiwi_slam`、`kiwi_navigation` 的包前缀均位于当前工作区。可独立核对：

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
```

两者必须位于 `/home/a/ros2_ws_mid360/install/`；若显示旧 `livox_ws` 或 `fastlio_ws`，当前终端不能作为统一部署的验证环境。迁移保留旧分离工作区和原 `~/ros2_ws`，运行入口与地图目录使用新布局；不要跨机器复制生成的 `build/`、`install/`、`log/`。

## 3. 配置设备 IP、接收 IP 和 UDP 端口

复制所安装驱动的 [MID360S 配置示例](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/config/MID360s_config.json) 为站点专用文件。实机必须使用区分大小写的 **`Mid360s`** 配置段，不能用 `MID360`；`lidar_configs[0].ip=192.168.1.117`。当前 SDK2 的 `Mid360s.host_net_info` 是数组，包含 `lidar_ip: ["192.168.1.117"]`、`host_ip: "192.168.1.50"` 和下表主机端口；[SDK2 解析器](https://github.com/Livox-SDK/Livox-SDK2/blob/master/sdk_core/parse_cfg_file.cpp)会据此为指定雷达建立通道。旧 `MID360` 配置在本机只绑定发现端口，未收到 ROS 数据。保持 `lidar_summary_info.lidar_type=8`（这是驱动设备族标志，不能改成型号代码 35），单雷达使用 `multi_topic=0`，点云使用 `xfer_format=1` 以输出 `livox_ros_driver2/msg/CustomMsg`。本仓库启动器默认 `lidar_model:=Mid360s` 并生成站点 JSON，无需手改驱动安装目录。

| 数据 | 雷达 UDP 端口 | 主机示例 UDP 端口 |
| --- | ---: | ---: |
| 设备发现 | 56000 | 任意 |
| 控制命令 | 56100 | 56101 |
| 状态推送 | 56200 | 56201 |
| 点云 | 56300 | 56301 |
| 内置 IMU | 56400 | 56401 |
| 日志 | 56500 | 56501 |

端口依据 [Livox MID-360 通信协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)；若主机防火墙开启，允许雷达网卡相应 UDP 流量。`ping` 通只说明 IP 链路可达，不能证明 UDP 点云和 IMU 已抵达。可用 `sudo tcpdump -ni "$LIDAR_IF" 'udp and (port 56301 or port 56401)'` 查收包。不要通过 SSH 所用网卡盲目改 IP、防火墙或默认路由。

## 4. 逐段核验话题与外参

驱动可先按官方 ROS 2 入口 `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 独立启动；该 launch 默认读取其安装目录的 `MID360_config.json`，因此须确认启动日志显示的配置文件正是改过的站点文件。若用外部 JSON，可直接给节点参数 `user_config_path`，关键参数与[官方 launch](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/launch_ROS2/msg_MID360_launch.py)对应：

```bash
source "$HOME/ros2_ws_mid360/tools/activate_workspace.sh"
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p user_config_path:=/绝对路径/MID360_config.json \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame \
  -p cmdline_input_bd_code:=livox0000000001
```

另开终端，加载统一工作区，并在**相同的 ROS_DOMAIN_ID 和 RMW 设置**下核验：

```bash
source "$HOME/ros2_ws_mid360/tools/activate_workspace.sh"
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/imu --once
```

预期 `/livox/lidar` 为 `livox_ros_driver2/msg/CustomMsg`，`/livox/imu` 为 `sensor_msgs/msg/Imu`；典型频率分别约 10 Hz、200 Hz，应以实测时间戳、丢包和硬件工作模式判定。启动 FAST-LIO 前验证两者时间戳同步、没有倒退或秒级偏差；不要用 `time_sync_en=true` 掩盖时钟问题。[上游 MID-360 参数](https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/config/mid360.yaml)使用 `lid_topic=/livox/lidar`、`imu_topic=/livox/imu`、`lidar_type=1`、`timestamp_unit=3`、`feature_extract_enable=false`。`pcd_save_en` 默认开启且 `interval=-1` 可把全部点写入单个 PCD，长时间运行有内存/磁盘压力，应关闭或限定片段。

复制 `fast_lio/config/mid360.yaml` 为站点参数文件，现场核对话题、时间同步和以下外参，先设 `pcd_save_en: false`。驱动已正常推送后，在另一个终端启动：

```bash
source "$HOME/ros2_ws_mid360/tools/activate_workspace.sh"
ros2 launch fast_lio mapping.launch.py \
  config_path:=/站点参数所在目录 config_file:=mid360.yaml rviz:=false
ros2 topic type /Odometry
ros2 topic hz /Odometry
```

上述 `config_path` 和 `config_file` 是该[移植版 launch](https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/launch/mapping.launch.py)提供的参数；终端中的站点参数目录要换成绝对路径。运行中检查 CPU、内存、点云延迟，以及静置时位姿是否异常漂移。

FAST-LIO 的 `extrinsic_T/R` 表示**雷达相对内部 IMU**的位姿；[MID-360S 官方手册](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_zh-cn.pdf)确认 IMU 与点云坐标轴同向，位置为 `(11, 23.29, -44.12)` mm，因此标称值 `[-0.011, -0.02329, 0.04412]` m 和单位旋转适用。这里不是机身到雷达的安装位姿。用户确认雷达原点及朝向与旧 M10P 相同。旧履带配置记录 `base_link → lidar_link = (0.24, 0, 0) m`、`base_footprint → base_link = (0, 0, 0.15) m`，因此结合内部外参，当前 `mid360_mount.yaml` 推得 `base_footprint → body(IMU) = (0.251, 0.02329, 0.10588) m`、旋转全零。这以“雷达光学原点真正重合”为前提，安装验收仍须用静态点云和原地旋转轨迹复核；若换支架或光学中心不同，修改外参并重建地图。不要套用 C 板 IMU 外参。当前 FAST-LIO 参数关闭在线外参估计。

## 5. 三维里程计、二维建图和重定位的边界

FAST-LIO2 已在内部用迭代卡尔曼滤波融合 MID-360S 点云和**同一个内置 IMU**，输出完整三维相对位姿。本模式没有第二个可信的独立里程计，因此不启动 `robot_localization` EKF，也不融合 C 板 IMU 或全零履带电机反馈。C 板 `/chassis/imu/data_raw` 仍发布供诊断和旧入口兼容。

所选 ROS 2 移植版发布完整三维 `/Odometry`，帧为 `camera_init → body`，并直接广播同名 TF（[源码](https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/src/laserMapping.cpp)）。启动器把该 TF 隔离到私有话题，适配器按外参把三维位姿投影成 Nav2 使用的 `/fastlio/odom_base` 和唯一动态 `odom → base_footprint` TF。该移植版的 `publish_odometry()` 在 `publish()` **之后**填充 `pose.covariance`，所以适配器为平面消息填写独立的保守协方差；原始三维位姿仍应单独评估。FAST-LIO 是局部 LIO/增量建图，不自带已知地图的全局重定位或可靠闭环校正；重启后 `camera_init` 原点也会重设。

AMCL 要求已有的**二维 OccupancyGrid 地图**和 `sensor_msgs/msg/LaserScan`；MID-360 的 3D 点云须按机器人高度、地面、顶棚和障碍物范围筛成稳定的 2D 扫描，另制可验收的 2D 栅格图。不能把 FAST-LIO 的 PCD 文件直接交给 AMCL。若选择 ICP 重定位，则需要持久化的**三维点云地图**、当前点云与初始位姿约束，ICP 节点须输出可检验的 `map → odom` 校正；大走廊几何退化、动态物体和错误初值要有失配判据。Nav2 障碍层可直接用 `PointCloud2`，但也须配置高度过滤和清障；参见 [Nav2 传感器与定位指南](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/) 和 [Voxel Layer 参数](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)。

首次现场验收依次检查：网卡路由与 UDP 收包；`CustomMsg`/IMU 类型、频率和时间戳；静置 IMU 重力轴及点云方向；FAST-LIO `/Odometry` 静置漂移和往返闭合误差；适配后 TF 唯一性；2D/3D 地图质量与重定位残差；最后才在低速受控环境测试 Nav2。保留原 M10P/C 板模式以便回退。

## 6. 使用本仓库的 MID360 专用链路

先停止原 M10P/RF2O/旧 EKF/旧 SLAM Toolbox 启动进程及上文的独立驱动调试进程。活动底盘须为 `tracked_pwm`，两个入口不能同时运行。远程原 `~/ros2_ws` 保留；新入口仅使用统一的 `~/ros2_ws_mid360`。按第 2 节构建完成后，每个运行、录包和检查终端只需：

```bash
source "$HOME/ros2_ws_mid360/tools/activate_workspace.sh"
```

本仓库的启动器根据**实机** `lidar_interface`、`host_ip`、`lidar_ip` 生成 Livox JSON，启动前会检查主机该网卡已有对应 IPv4 地址且去往雷达的路由确实经过该网卡。它不修改系统 IP；先执行第 1 节的临时地址测试，并在现场确认后将地址写入该网卡持久配置。`map_pcd` 留空时默认写入统一工作区根目录 `maps/mid360_map.pcd` 并创建 `maps/`；显式覆盖时必须使用可写目录下的绝对路径。FAST-LIO 的 PCD 保存服务写入此路径；二维地图由 SLAM Toolbox 另存。将下方三个值替换为现场读数：

```bash
mkdir -p "$KIWI_WORKSPACE_ROOT/maps"
LIDAR_IF=enp2s0
HOST_IP=192.168.1.50
LIDAR_IP=192.168.1.117
ros2 launch kiwi_navigation mid360_navigation.launch.py \
  mode:=mapping lidar_interface:="$LIDAR_IF" \
  host_ip:="$HOST_IP" lidar_ip:="$LIDAR_IP" \
  map_pcd:="$KIWI_WORKSPACE_ROOT/maps/mid360_3d.pcd" use_rviz:=true

# 建图完成后，在另一个已加载相同环境的终端保存二维地图：
ros2 run nav2_map_server map_saver_cli -f "$KIWI_WORKSPACE_ROOT/maps/mid360_2d"
# 如需保存 FAST-LIO 当前三维点云，调用移植版的服务；长时间建图注意内存。
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

首次建图先缓慢短距离运动，核对 `/livox/lidar`、`/livox/imu`、`/fastlio/odometry`、`/fastlio/odom_base`、`/scan_mid360` 和 `/map` 的频率与时间戳。`/scan_mid360` 的光束原点是 `lidar_link`，但障碍高度在 `base_footprint` 下裁剪为 0.10–1.20 m（可通过 `scan_z_min`/`scan_z_max` 修改）；`LaserScan` 量程裁为 0.5–20 m。检查低矮障碍、地面和顶棚是否正确进入/排除地图，必要时按现场环境调整高度并重建 2D 图。`/tf` 应只有适配器发布 `odom → base_footprint`，建图时只有 SLAM Toolbox 发布 `map → odom`；FAST-LIO 自带 TF 被隔离。

已有通过人工核对的二维地图后，停止建图入口，改用 AMCL 已知图导航：

```bash
ros2 launch kiwi_navigation mid360_navigation.launch.py \
  mode:=localization map:="$KIWI_WORKSPACE_ROOT/maps/mid360_2d.yaml" \
  lidar_interface:="$LIDAR_IF" host_ip:="$HOST_IP" \
  lidar_ip:="$LIDAR_IP" use_rviz:=true
```

在 RViz 设定初始位姿，核对 AMCL 的 `map → odom` 与真实位置吻合后，再下发低速导航目标。定位模式不运行 SLAM Toolbox，故 AMCL 是唯一 `map → odom` 发布者。AMCL 利用二维地图给出全局重定位，FAST-LIO2 持续给出局部三维运动估计；这不等于点云在既有三维地图上的 ICP 重定位。`/cmd_vel` 先经 `mid360_motion_gate`，仅当 LIO 里程计和投影扫描时间戳都在 0.5 秒内才转发至底盘 `/mid360/cmd_vel_safe`，失效时持续发送零速；底盘本身还有 0.2 秒命令看门狗。C 板 IMU 仍可在 `/chassis/imu/data_raw` 查看，但不会影响该定位链。

历史部署记录（以下 `/home/a/maps` 是 2026-09-27 的原始保存路径；当前地图目录已统一为工作区根目录 `maps/`）：

2026-09-27 静止实机检查已完成：有线网卡 `192.168.1.50/24`、雷达 `192.168.1.117`，Livox CustomMsg 约 10 Hz、内置 IMU 约 200 Hz、FAST-LIO2 三维里程计和 `/scan_mid360` 均约 10 Hz。抽查的一帧 720 束二维扫描有 517 束有限距离。SLAM Toolbox 发布 0.05 m 栅格图并成功保存到 `/home/a/maps/mid360_stationary_smoke.yaml` 与 `.pgm`；FAST-LIO `/map_save` 返回成功，生成 `/home/a/maps/mid360_stationary_3d_smoke.pcd`（57,970 点）。用该测试图启动 AMCL，设初始位姿后 `map → odom` 可查询，Nav2 托管节点进入 active。带底盘节点的零速检查确认 `/dev/robomaster_c` 打开、C 板原始 IMU 约 100 Hz、门控输出全零。上述文件仅是**静止接口检查产物**，不能用于实际导航；还须在受控区域完成外参、时间同步、点云/栅格地图质量、行驶漂移与 AMCL 重定位验证后，才下发导航目标。


## 7. 统一工作区整合时的现场条件

2026-10-08 用户确认本次只有上位机，雷达和 C 板均未连接，验收目标为统一工作区的软件构建、测试与 Web 正常使用。远程检查显示 `enp2s0` 为 DOWN、没有 IPv4 地址，`/dev/robomaster_c` 不存在，检查时未运行 ROS 进程，均符合本次测试条件。去往 `192.168.1.117` 的路由经过 Wi-Fi 网关，因此当前 `ping` 返回不能作为雷达直连在线证据。Web 应能启动并如实显示外设未连接，硬件链路的运动、地图质量与定位精度不在本次软件验收范围。后续接回外设后按第 4–6 节核验。

整合完成：本地和远程各 10 个 ROS 包全量编译成功。所选 7 个功能包测试均无错误、
无失败，其中 Livox 内嵌 SDK/消息烟测 3 项、Web 40 项通过。双端 FAST-LIO 无输入
启动、服务发现和 SIGINT 退出通过；日志目录不可写时会关闭日志并正常退出。
固定位置夹具下 Nav2 的 8 个生命周期节点激活、速度路由和 10 个子进程正常退出通过。
原动态 TF 夹具曾触发远程 TF/scan 锁等待，固定位置夹具改用静态 TF 后退出通过，
该结果不代表真实运动中的动态 TF 已验收。远程 Web 五个页面、地图读取/导出通过，
未接外设时启动建图报告网卡缺少主机 IP 并返回空闲模式。旧地图已复制到
`/home/a/ros2_ws_mid360/maps/`，原 `~/maps` 和旧分离工作区保留。

根 README 仅保留当前 MID360S 链路的原生一行命令，不增加统一命令封装层。
