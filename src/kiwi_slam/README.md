# kiwi_slam

`kiwi_slam` 是传感器与建图集成层。主路线使用 Livox Mid-360S 内置 IMU、
`livox_ros_driver2` 和 FAST-LIO2；Pacecat LDS-E110-R + SLAM Toolbox 二维路线完整保留。
两条路线复用同一个底盘、URDF 和 TF 约定，不互相修改驱动源码。

## 1. 架构边界

```text
kiwi_base              USB、运动学、底盘 IMU、轮式 odom、关节状态
kiwi_robot.launch.py   底盘 + 可替换 EKF YAML + URDF 公共层
kiwi_lio.launch.py     Mid-360S + IMU 适配 + FAST-LIO2 主路线
kiwi_slam.launch.py    Pace E110 + SLAM Toolbox 备用二维路线
kiwi_navigation        可替换雷达 + SLAM/AMCL + Nav2 全向导航组合层
```

传感器选择只发生在 EKF YAML：

```text
Mid-360S 主路线  config/ekf_mid360.yaml       /livox/imu/ekf
Pace 备用路线    kiwi_base/config/ekf_chassis.yaml  /chassis/imu/data_raw
```

底盘节点始终发布 C 板 IMU。没有任何雷达节点能够关闭或改写该话题。

## 2. Mid-360S 主路线

```text
Mid-360S --Ethernet/UDP--> livox_ros_driver2
  ├─ /livox/lidar (CustomMsg) ───────────────┐
  └─ /livox/imu (Imu) ───────────────────────┼─> FAST-LIO2
       └─ mid360_imu_adapter                 │    ├─ /cloud_registered
            └─ /livox/imu/ekf ─┐             │    ├─ /Laser_map
                                ├─> EKF       │    ├─ /Odometry
/chassis/odom/raw ──────────────┘             │    └─ /path
  └─ /odometry/filtered + odom -> base_footprint
```

FAST-LIO2 直接读取原始 `/livox/imu`，适配节点只为平面 EKF 复制一条带协方差的
`/livox/imu/ekf` 支路。两条数据互不覆盖。`/Laser_map` 是三维 `PointCloud2`，不是
Nav2 所需的二维 `nav_msgs/OccupancyGrid /map`。

Mid-360S 三维建图和 Pace 二维建图均可独立运行。二维导航由新增的
`kiwi_navigation` 负责：E110 直接提供 `/scan`，Mid-360 通过独立点云投影节点提供相同
的 `/scan`，上层 SLAM Toolbox、AMCL 和 Nav2 不感知雷达型号。Mid-360 的 `/Laser_map`
仍是三维点云，不冒充 Nav2 的二维 `/map`；二维路线使用 `radar_mid360.yaml` 的水平
切片生成 `/scan`。

## 3. Pace 备用路线

```text
LDS-E110-R -> bluesea2 -> /scan -> slam_toolbox
                                      ├─ /map
                                      └─ map -> odom

/chassis/odom/raw + /chassis/imu/data_raw -> EKF
                                               ├─ /odometry/filtered
                                               └─ odom -> base_footprint
```

该路线可直接生成二维 `/map`，用于现有 SLAM Toolbox 联调。切回 Pace 只换启动命令，
不改 Mid-360S、底盘或 URDF 代码。

## 4. 参数与外参入口

```text
config/mid360s_network.json    工控机/雷达 IP 和 Livox UDP 端口（SDK 固定 JSON 格式）
config/livox_mid360s.yaml      Livox 驱动格式、频率和 frame_id
config/mid360_imu.yaml         EKF 支路协方差和角速度死区
config/ekf_mid360.yaml         Mid-360S 主路线的 EKF 输入选择
config/fast_lio_mid360s.yaml   FAST-LIO2 内参、厂内 IMU 外参和地图输出
config/lds_e110.yaml           Pace 串口与 LaserScan 参数
config/slam_toolbox.yaml       Pace 二维建图参数
urdf/kiwi.urdf.xacro           整车结构和传感器相对 base_link 的安装外参
```

安装外参只在 Xacro 维护：

```text
base_link -> chassis_imu_link  C 板 BMI088 安装位姿
base_link -> lidar_link        雷达整机安装位姿
lidar_link -> livox_frame      Livox 驱动坐标系连接
```

`fast_lio_mid360s.yaml` 的 `extrinsic_T/R` 是 Mid-360S 点云原点到内置 IMU 的厂内标定，
不是雷达相对底盘的安装外参，不能与 Xacro 数值混用。

## 5. 调试话题和消息结构

```text
话题                         类型                                  发布者/用途
/chassis/feedback/raw        kiwi_base/msg/FeedbackDebug          C 板 12 字段原始反馈
/chassis/imu/data_raw        sensor_msgs/msg/Imu                   C 板底盘 IMU
/chassis/odom/raw            nav_msgs/msg/Odometry                三轮原始轮式里程计
/joint_states                sensor_msgs/msg/JointState           三轮角度和角速度
/livox/lidar                 livox_ros_driver2/msg/CustomMsg       Mid-360S 原始点和逐点时间
/livox/imu                   sensor_msgs/msg/Imu                   Mid-360S 原始六轴数据
/livox/imu/ekf               sensor_msgs/msg/Imu                   补协方差后的 EKF 支路
/odometry/filtered           nav_msgs/msg/Odometry                平面 EKF 位姿和速度
/cloud_registered            sensor_msgs/msg/PointCloud2          LIO 世界系配准点云
/cloud_registered_body       sensor_msgs/msg/PointCloud2          LIO 机体系点云
/Laser_map                   sensor_msgs/msg/PointCloud2          FAST-LIO2 三维地图
/Odometry                    nav_msgs/msg/Odometry                FAST-LIO2 三维位姿
/path                        nav_msgs/msg/Path                    FAST-LIO2 轨迹
/scan                        sensor_msgs/msg/LaserScan             Pace 路线 SLAM 输入
/map                         nav_msgs/msg/OccupancyGrid           Pace 路线二维栅格地图
```

Livox `CustomMsg` 的主要字段：

```text
header       ROS 时间戳和 livox_frame
timebase     当前数据包的纳秒时间基准
point_num    有效点数量
lidar_id     雷达编号
points[]     每点 offset_time、x/y/z、reflectivity、tag、line
```

`sensor_msgs/msg/Imu` 包含 `orientation`、`angular_velocity`、
`linear_acceleration` 及三组 3x3 协方差。Livox 原始消息不提供有效 orientation；
适配消息将 `orientation_covariance[0]` 设为 `-1` 明确表示无姿态。

`nav_msgs/msg/Odometry` 包含 `header`、`child_frame_id`、带 6x6 协方差的 `pose` 和
`twist`。`/chassis/odom/raw` 是轮速原始量，`/odometry/filtered` 是平面 EKF 输出，
`/Odometry` 是 FAST-LIO2 三维输出，三者不可混用。

## 6. 外部依赖和构建

根仓库固定以下源码版本：

```text
src/livox_ros_driver2  Livox 官方 1.2.6（git submodule）
src/FAST_LIO_ROS2      ros2 分支固定提交（git submodule，含 ikd-Tree）
third_party/Livox-SDK2 v1.3.1（本机系统库，不纳入根仓库）
```

首次拉取根仓库：

```bash
cd ~/ros2_ws
git submodule update --init --recursive

# 官方驱动只跟踪 package_ROS2.xml；复制为 package.xml 后 colcon 才能发现。
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
```

若工控机尚未安装 SDK：

```bash
git clone --branch v1.3.1 --depth 1 \
  https://github.com/Livox-SDK/Livox-SDK2.git third_party/Livox-SDK2
cmake -S third_party/Livox-SDK2 -B third_party/Livox-SDK2/build
cmake --build third_party/Livox-SDK2/build -j$(nproc)
sudo cmake --install third_party/Livox-SDK2/build
sudo ldconfig

# 驱动 CMake 固定从 /usr/local 查找 SDK，安装后应能看到这两个文件。
test -f /usr/local/lib/liblivox_lidar_sdk_shared.so
test -f /usr/local/include/livox_lidar_api.h
```

若构建报 `Could not find LIVOX_LIDAR_SDK_LIBRARY`，表示 SDK 只完成了编译、尚未
完成系统安装；重新执行上面的 `sudo cmake --install ...` 和 `sudo ldconfig`。

安装 ROS 依赖并构建：

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 工作区 colcon.meta 会自动只为 livox_ros_driver2 传入 -DDISTRO_ROS=humble。
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to livox_ros_driver2 fast_lio kiwi_slam
source install/setup.bash
```

日常修改本项目代码时使用单包增量构建即可：

```bash
colcon build --symlink-install --packages-select kiwi_base kiwi_slam
```

只有首次部署、修改消息/依赖、更新外部驱动或清理 `build/ install/ log/` 后，才需要重新执行
全量或按依赖构建。`third_party/Livox-SDK2` 是独立 SDK：首次安装后不会随每次 colcon
重复编译；只有 SDK 源码或安装环境变化时才重新执行 SDK 的 `cmake --build` 和 `cmake --install`。

## 7. SCP 到工控机

仅同步本项目封装（工控机已有 SDK 和两个外部仓库时推荐）：

```bash
scp -r \
  /home/ym/ros2_ws/src/kiwi_base \
  /home/ym/ros2_ws/src/kiwi_slam \
  a@192.168.66.119:~/ros2_ws/src/
```

首次完整同步源码：

```bash
scp -r \
  /home/ym/ros2_ws/src/FAST_LIO_ROS2 \
  /home/ym/ros2_ws/src/livox_ros_driver2 \
  a@192.168.66.119:~/ros2_ws/src/

scp -r /home/ym/ros2_ws/third_party/Livox-SDK2 \
  a@192.168.66.119:~/ros2_ws/third_party/
```

不在文档或脚本保存 SSH 密码；输入密码或使用 SSH key。SCP 后在工控机重新执行本节构建，
不要复制本机 `build/ install/ log/`。

## 8. 启动命令

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Mid-360S 主路线：

```bash
ping -c 3 192.168.1.117
ros2 launch kiwi_slam kiwi_lio.launch.py
```

三维 RViz 的 Fixed Frame 使用 `camera_init`。这是 FAST-LIO2 硬编码的初始世界坐标系，
与摄像头硬件无关；`/cloud_registered` 和 `/Laser_map` 都使用该坐标系。若 RViz 没有点云，
优先确认 `/livox/lidar` 存在发布者，以及 `/cloud_registered` 的 `width` 大于 0。
主路线默认使用硬件 OpenGL；若远程桌面出现黑屏，再加 `rviz_software_rendering:=true` 回退到软件渲染。

FAST-LIO 原生 `camera_init -> body` TF 在启动文件中隔离到 `/fast_lio/tf`，所以它不会出现在
底盘的 `/tf` 树中，也不会自动与 `odom` 相连。三维 RViz 切换到 `odom`、`map` 或
`camera_link` 后，点云没有可用变换就会被隐藏；切回时在 Fixed Frame 输入框手动输入
`camera_init`，或重新加载 `config/kiwi_lio.rviz`，不要选择不存在的 `camera_link`。

无桌面工控机：

```bash
ros2 launch kiwi_slam kiwi_lio.launch.py use_rviz:=false
```

Pace 二维备用路线：

```bash
ros2 launch kiwi_slam kiwi_slam.launch.py use_rviz:=true
```

Pace 建图并同时启动 Nav2：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py
```

切换到 Mid-360 作为二维建图输入：

```bash
ros2 launch kiwi_navigation mapping_navigation.launch.py lidar_type:=mid360
```

已保存地图的 AMCL 导航：

```bash
ros2 launch kiwi_navigation localization_navigation.launch.py \
  map:=$HOME/ros2_ws/maps/kiwi_site.yaml
```

只启动公共底盘、C 板 EKF 和模型：

```bash
ros2 launch kiwi_slam kiwi_robot.launch.py
```

## 9. 实时调试接口

```bash
# Mid-360S 输入
ros2 topic info /livox/lidar --verbose
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/imu --once

# 底盘输入与两种 IMU 对照
ros2 topic hz /chassis/feedback/raw
ros2 topic echo /chassis/feedback/raw
ros2 topic echo /chassis/imu/data_raw
ros2 topic echo /chassis/odom/raw
ros2 topic echo /livox/imu/ekf

# EKF 和 LIO 输出
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 topic hz /cloud_registered
ros2 topic hz /Laser_map
ros2 topic echo /cloud_registered --once
ros2 topic echo /Odometry --once

# TF 所有权和安装外参
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link chassis_imu_link
ros2 run tf2_ros tf2_echo base_link livox_frame

# Pace 路线
ros2 topic hz /scan
ros2 topic hz /map
ros2 service call /kiwi_slam_monitor/check std_srvs/srv/Trigger "{}"

# 接口定义和节点连接
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show nav_msgs/msg/Odometry
rqt_graph
```

Foxglove 可直接选择上述标量字段和点云话题；如已安装 bridge：

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## 10. 测试

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select kiwi_base kiwi_slam --event-handlers console_direct+
colcon test-result --verbose

xacro src/kiwi_slam/urdf/kiwi.urdf.xacro \
  hardware_config:="$(pwd)/src/kiwi_base/config/kiwi_base.yaml" \
  > /tmp/kiwi.urdf
check_urdf /tmp/kiwi.urdf
```

FAST-LIO2 发布的原生 TF 被重映射到 `/fast_lio/tf`，避免与底盘 EKF 争抢 TF。
二维 Nav2 路线必须验证唯一的 `map -> odom -> base_footprint -> base_link` 链路；三维
路线必须在 RViz 使用 `camera_init`，不要将两种 Fixed Frame 混用。
