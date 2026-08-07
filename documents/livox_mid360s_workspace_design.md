# Kiwi 底盘接入 Livox Mid-360S 的工作空间设计

## 1. 目标与版本

运行环境保持 Ubuntu 22.04 与 ROS 2 Humble。雷达由 PaceCat LDS-E110-R 更换为
Livox Mid-360S，采用以下官方组件：

```text
组件                 固定版本   放置位置
Livox-SDK2           v1.3.1     third_party/Livox-SDK2
livox_ros_driver2    1.2.6      src/livox_ros_driver2（下一阶段下载）
```

选择这些版本的原因：两者从上述版本开始明确支持 Mid-360S。`Livox-SDK2` 只提供
C/C++ 通信库，不是 ROS 功能包；真正发布 ROS 2 点云和 IMU 话题的是
`livox_ros_driver2`。

## 2. 推荐工作空间结构

```text
/home/ym/ros2_ws
├── third_party/                         # 不参与 colcon 扫描
│   ├── README.md                        # 固定上游版本和安装方法
│   └── Livox-SDK2/                      # v1.3.1，本地下载且不纳入 Git
├── src/                                 # 只放 ROS 2 功能包
│   ├── base/                            # 现有公共接口
│   ├── kiwi_base/                       # 底盘、轮速、C 板 IMU、EKF
│   ├── livox_ros_driver2/               # 官方 ROS 2 驱动，固定 1.2.6
│   ├── kiwi_lidar/                      # 计划新增：本项目的雷达集成层
│   │   ├── config/
│   │   │   ├── mid360s.example.json     # 网卡、雷达 IP 与 UDP 端口模板
│   │   │   └── pointcloud_filter.yaml   # 机身裁剪、距离和高度范围
│   │   ├── launch/
│   │   │   └── mid360s.launch.py        # 驱动、过滤和诊断统一入口
│   │   ├── rviz/
│   │   └── test/
│   ├── kiwi_description/                # 计划拆分：整车 URDF/Xacro 和静态 TF
│   ├── kiwi_slam/                       # 建图算法与 map->odom，不直接管理硬件
│   └── kiwi_navigation/                 # 后续 Nav2 参数、行为树和任务入口
├── documents/
├── build/                               # colcon 生成目录
├── install/
└── log/
```

`kiwi_lidar` 只保存本项目配置和启动封装，不复制或修改官方驱动源码。当前
`kiwi_slam` 中的 URDF 可以先继续使用；确认 Mid-360S 安装位姿后，再以小步迁移
方式拆出 `kiwi_description`，避免一次改动过多。

## 3. 数据链路

### 3.1 第一阶段：硬件联调

```text
Mid-360S
  └─ Ethernet/UDP
      └─ Livox-SDK2
          └─ livox_ros_driver2
              ├─ /livox/lidar   sensor_msgs/msg/PointCloud2
              └─ /livox/imu     sensor_msgs/msg/Imu
```

第一阶段只验证设备发现、点云、内置 IMU、时间戳、频率和丢包率。RViz Fixed Frame
暂时设置为 `livox_frame`，不接入 SLAM，避免把 TF 或里程计问题误判成雷达问题。

推荐使用带逐点时间戳的 Livox PointCloud2 格式。Mid-360S 内置 IMU 优先供点云
去畸变和 LIO 使用，暂时不要与 C 板 IMU 同时接入现有 EKF，以免重复融合两个
方向、零偏和时间基准尚未标定的 IMU。

### 3.2 第二阶段：保留现有二维建图链路

为了先复用现有 SLAM Toolbox，可以增加临时转换层：

```text
/livox/lidar
  └─ 点云坐标变换与机身裁剪
      └─ pointcloud_to_laserscan
          └─ /scan
              └─ slam_toolbox
                  ├─ /map
                  └─ map -> odom
```

转换时只保留接近水平面的高度带，并裁掉车体自身点云。该方案接入快，但不能充分
利用三维点云，而且 Mid-360S 的非重复扫描会让单帧二维切片稀疏，因此只作为
硬件联调和二维导航过渡方案。

### 3.3 第三阶段：三维 LIO 建图

生产方案应使用支持 Livox 逐点时间戳和内置 IMU 的 LIO 算法，例如经过 Humble
兼容性验证的 FAST-LIO2 接入层：

```text
/livox/lidar + /livox/imu
  └─ LIO 去畸变、里程计与三维建图
      ├─ 三维注册点云
      ├─ LIO 全局位姿
      └─ 二维占据栅格投影 -> /map -> Nav2
```

现有轮速 EKF 继续负责连续、平滑的局部里程计。LIO 提供全局校正时，不允许它和
EKF 同时发布 `odom -> base_footprint`。正确 TF 所有权为：

```text
发布者                    唯一负责的 TF
robot_state_publisher     base_footprint -> base_link -> livox_frame
robot_localization EKF    odom -> base_footprint
SLAM/LIO 适配层           map -> odom
```

若 LIO 原生输出 `map -> livox_frame`，适配层按同一时刻的 `odom -> livox_frame`
计算并发布 `map -> odom`，避免 TF 双发布和模型跳变。

## 4. 坐标系和安装外参

Mid-360S 的固定外参只在 URDF/Xacro 中维护一份：

```text
base_footprint -> base_link -> livox_frame
```

需要实测雷达光学中心相对 `base_link` 的 `x、y、z、roll、pitch、yaw`。不得同时在
Livox JSON、URDF 和 SLAM 配置中重复设置同一组外参，否则会发生二次变换。项目
默认由 URDF 发布外参，驱动只发布 `livox_frame` 下的原始数据。

## 5. 网络边界

Mid-360S 使用以太网 UDP，不再使用 `/dev/ttyUSB0`。工控机需要独立有线网卡或
明确的静态路由，主机 IP 和雷达 IP 必须处于同一子网。实际 IP 应从设备标签、
Livox Viewer 2 或现场网络配置读取，不能直接照抄示例值。

配置文件负责以下内容：

```text
- lidar_ip：实际雷达地址
- host_ip：连接雷达的工控机网卡地址
- 命令、状态、点云、IMU、日志 UDP 端口
- 点云模式与发布频率
```

联调前至少检查：

```bash
ip -br address
ip route
ping -c 3 <lidar_ip>
```

然后观察点云和 IMU：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once --field header
```

## 6. 实施顺序与验收

```text
阶段  内容                                      验收标准
1     编译安装 Livox-SDK2 v1.3.1               动态库可被系统找到
2     下载并编译 livox_ros_driver2 1.2.6       Humble 构建零报错
3     配置工控机网卡和 Mid-360S JSON            可发现设备且 UDP 无持续丢包
4     发布原始点云和内置 IMU                    话题频率、时间戳连续
5     标定 base_link -> livox_frame             静止模型与点云方向正确
6     先接二维转换链路验证现有 SLAM             /scan、/map 和 TF 连续
7     接入 LIO、点云去畸变和二维地图投影        运动时无明显重影和漂移
8     接入 Nav2                                 定位、规划和控制闭环稳定
```

当前只完成 SDK 下载与目录设计，尚未执行系统安装，也未下载 ROS 2 驱动；因此不
影响现有 `kiwi_base`、`bluesea2` 和 `kiwi_slam` 的构建与运行。

