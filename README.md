# MID360S 履带建图与导航

Ubuntu 22.04 / ROS 2 Humble。MID360S 使用内置 IMU，FAST-LIO2 输出三维里程计，
SLAM Toolbox 生成二维地图，AMCL + Nav2 定位导航。C 板负责底盘通信，保留 IMU 接口，
不进入定位融合；无需外部 EKF。

本地工作区：`/home/hi/ros2_ws_old`；远程工作区：`~/ros2_ws_mid360`。
下面命令均在工作区根目录执行，**每行可以直接复制使用**。

## 设备配置

默认雷达 `192.168.1.117`，主机雷达网卡 `enp2s0=192.168.1.50/24`，
C 板串口 `/dev/robomaster_c`。雷达是以太网设备，只有 C 板需要 udev 串口映射。

首次创建雷达静态网络配置，无默认网关；接回网线后自动连接：

```bash
sudo nmcli con add type ethernet con-name mid360-enp2s0 ifname enp2s0 ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect-priority 100
```

已创建过该配置时更新地址：

```bash
sudo nmcli con mod mid360-enp2s0 ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.gateway "" ipv4.never-default yes
```

安装 C 板映射规则和串口权限：

```bash
sudo install -m 644 tools/99-robomaster-c.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty && sudo usermod -aG dialout "$USER"
```

规则使用当前 C 板 `0483:5740`、序列号 `364E30673533`。加入 `dialout` 后重新登录。
没有外设时可完成软件编译并打开 Web，建图与导航需要接回雷达和 C 板。

## 编译

编译前加载 ROS：

```bash
source /opt/ros/humble/setup.bash
```

首次从 Git 检出时补齐依赖源码（已有该目录可跳过）：

```bash
vcs import src < tools/platform_dependencies.repos && bash tools/apply_rf2o_patch.sh
```

安装系统依赖（已安装可跳过）：

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys=cmake_modules
```

全量编译：

```bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

编译完成后加载本工作区；每个运行终端执行一次：

```bash
source /opt/ros/humble/setup.bash && source install/local_setup.bash && export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

单包编译，例如 Web；其他包替换 `kiwi_web`：

```bash
colcon build --symlink-install --packages-select kiwi_web
```

单独编译 SDK 静态库（驱动全量编译会自动完成，无需提前执行）：

```bash
cmake -S third_party/livox_sdk2 -B build/livox_sdk2 -DLIVOX_SDK2_BUILD_SHARED=OFF -DLIVOX_SDK2_BUILD_SAMPLES=OFF -DLIVOX_SDK2_INSTALL=OFF && cmake --build build/livox_sdk2 --target livox_lidar_sdk_static -j2
```

## 建图与导航

启动建图：

```bash
ros2 launch kiwi_navigation mid360_navigation.launch.py start_nav2:=false use_rviz:=true
```

另开已加载工作区环境的终端，启动键盘遥控：

```bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

停车后保存二维地图和三维 PCD：

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/mid360_map" && ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

用 `Ctrl+C` 停止建图和遥控后，启动导航：

```bash
ros2 launch kiwi_navigation mid360_navigation.launch.py mode:=localization map:="$PWD/maps/mid360_map.yaml" use_rviz:=true
```

在 RViz 设置真实初始位姿，定位稳定后发送目标。默认三维点云保存到
`maps/mid360_map.pcd`；AMCL 使用二维 YAML/PGM，PCD 不直接作为 AMCL 地图。
通过 SSH 且没有图形界面时，将 `use_rviz:=true` 改为 `false`。

## Web 前端

启动前端与后端：

```bash
ros2 launch kiwi_web web.launch.py
```

浏览器打开 [http://192.168.66.112:8080](http://192.168.66.112:8080/)。
Web 默认使用根目录 `maps/`，页面内可开始建图、保存地图、启动导航和设置初始位姿。
使用 Web 管理建图或导航前，停止命令行启动的同一套进程。

## 目录

- `src/livox_ros_driver2/`：驱动、消息和共享组件。
- `src/fast_lio/`：FAST-LIO2 与完整 ikd-Tree 源码。
- `src/kiwi_*/`：底盘、建图、导航和 Web。
- `third_party/livox_sdk2/`：SDK 静态库源码，含 `COLCON_IGNORE`。
- `maps/`：二维地图、三维 PCD、地图版本和路线。
- `build/`、`install/`、`log/`：编译与运行产物，不提交。

只使用一个工作区。SDK 由驱动 CMake 自动静态编译并链接，不需要 `/usr/local` SDK
或其他 Livox/FAST-LIO 工作区。第三方版本与许可见[依赖记录](third_party/README.md)。
# omni_wheel
# omni_wheel
