# ROS 2 Humble + Ubuntu 22.04 三轮全向底盘建图导航实施方案

> 版本：2026-08-03  
> 目标平台：Ubuntu 22.04 LTS + ROS 2 Humble  
> 激光雷达：Pacecat LDS-E110 系列（必须先确认后缀 R、R-1 或 R-3）  
> 底盘：三轮全向（Kiwi drive）  
> 文档定位：从电机/编码器、MCU、ROS 2 驱动，到建图、定位、导航和任务层的落地设计。  
> 说明：这是从零搭建的独立工程方案。开源仓库只作为协议、接口和算法参考，不假定现有工程可以直接使用。

## 1. 先给结论

推荐采用下面的基线：

1. 主机运行 Ubuntu 22.04 和 ROS 2 Humble，所有 Humble 软件包固定在同一发行版。
2. 三个轮子的电流环/速度环放在 MCU；ROS 2 只发送机体速度或轮速目标，不直接做 1 kHz 电机闭环。
3. MCU 通过 CAN 优先、USB CDC/UART 作为备选与主机通信；协议必须有序号、时间戳、CRC、使能位和看门狗。
4. 用 ros2_control 的自定义 SystemInterface 管理三路轮子的命令与反馈，再接一个三轮全向控制器。
5. 里程计由轮编码器计算，轮里程计和 IMU 用 robot_localization EKF 融合；EKF 唯一发布 odom 到 base_footprint 的 TF。
6. 建图使用 SLAM Toolbox；保存地图后，定位使用 AMCL，并明确使用 Humble 的 OmniMotionModel。
7. Nav2 第一阶段使用 DWBLocalPlanner，显式打开 Y 方向速度采样；后续可评估 Humble 的 MPPI Omni 模型。
8. 速度链路固定为：Nav2 → velocity_smoother → collision_monitor → 全向控制器 → MCU。硬件急停和通信看门狗独立于这条链路。
9. LDS-E110 先按官方串口协议做独立驱动和 RViz 验证，再接 SLAM/Nav2。不要把雷达 frame_id 写成 map。

最终验收不是“能启动几个 launch”，而是以下闭环都成立：

主机发目标速度 → 三轮实际运动 → 编码器/IMU 回传 → EKF 输出连续 odom/TF → 雷达扫描与 TF 对齐 → SLAM 生成地图 → AMCL 在地图中定位 → Nav2 发全向速度 → 任务层可取消、重试和恢复。

## 2. 系统边界与架构

### 2.1 分层

~~~text
+------------------------------- 任务层 -------------------------------+
| kiwi_mission：任务状态机、航点/动作、重试、取消、恢复、记录         |
+------------------------------- 导航层 -------------------------------+
| Nav2 planner/controller/costmap | smoother | collision monitor      |
+------------------------------- 定位建图层 ---------------------------+
| AMCL（定位） / SLAM Toolbox（建图） | robot_localization EKF       |
+------------------------------- ROS 2 设备层 --------------------------+
| pacecat_lds_e110_driver | kiwi_hardware | kiwi_omni_controller      |
| robot_state_publisher  | diagnostics | ros2_control/controller_mgr |
+------------------------------- 传输层 -------------------------------+
| CAN（首选）或 USB CDC/UART：命令、反馈、故障、急停状态              |
+------------------------------- MCU/执行层 ---------------------------+
| 电机驱动、编码器采集、1 kHz 速度环、通信看门狗、硬件急停             |
+---------------------------------------------------------------------+
| 三个轮毂电机 + 编码器 | IMU | Pacecat LDS-E110 | 电源/BMS/急停      |
~~~

### 2.2 主机与底盘连接

建议的实物链路：

~~~text
Ubuntu 22.04 SBC
  ├─ USB-UART（3.3 V TTL，或带电平转换）── LDS-E110
  ├─ USB-CAN / PCIe-CAN ────────────────── 底盘 MCU
  └─ USB/串口 ──────────────────────────── 调试与升级

底盘 MCU
  ├─ 三路电机驱动器
  ├─ 三路编码器
  ├─ IMU（可接 MCU，也可直接接 SBC）
  ├─ 12/24 V 动力电源、5 V 雷达电源
  └─ 硬件急停输入
~~~

首次联调可让 LDS-E110 直接接主机 USB-UART，减少 MCU 透传变量；量产或线束较长时再评估由 MCU 汇聚传感器。

### 2.3 不负责的部分

- Nav2、任务层不直接写 GPIO、PWM 或电机寄存器。
- AMCL 不替代里程计；它依赖 map、odom、base 和 scan 的 TF 链。
- collision_monitor 不等于功能安全急停；急停必须切断执行器使能或动力回路。
- 示例中的尺寸、轮半径、串口设备名和速度限制均不是实物定值，必须在参数文件中替换。

## 3. 约束、待确认输入和命名

### 3.1 现在已经锁定的约束

| 项目 | 固定值 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| 雷达 | Pacecat LDS-E110 系列 |
| 驱动消息 | sensor_msgs/msg/LaserScan |
| 底盘类型 | 三轮全向 Kiwi drive |
| 定位 | AMCL |
| 建图 | SLAM Toolbox |
| 任务接口 | ROS 2 action，内部用 Nav2 Simple Commander |

### 3.2 开工前必须填写的实物数据

| 数据 | 用途 | 当前状态 |
|---|---|---|
| LDS-E110 后缀（R/R-1/R-3） | 扫描频率、精度和量程参数 | 待确认 |
| 三轮中心到轮接触点距离 L | 逆/正运动学 | 待测量 |
| 轮半径 r、轮宽 | 轮速和 URDF | 待测量 |
| 三轮安装角 gamma、正方向 | 控制器参数 | 待测量 |
| 电机减速比、编码器线数 | MCU 轮速反馈 | 待确认 |
| MCU 型号和电机驱动协议 | kiwi_hardware | 待确认 |
| IMU 型号、安装方向和频率 | EKF | 待确认 |
| 底盘外轮廓 | costmap footprint、碰撞监测 | 待测量 |
| 计算机型号与总线 | CPU 预算、设备名 | 待确认 |

没有这些数据时，可以先做软件仿真和传感器单测，但不能声称运动学已经标定。

## 4. ROS 2 工作空间和包边界

建议工作空间为 ~/kiwi_ws，包名保持稳定，便于后续换电机或雷达：

~~~text
kiwi_ws/
└── src/
    ├── kiwi_description/          # URDF/Xacro、TF、碰撞模型、传感器安装位
    ├── kiwi_hardware/             # ros2_control SystemInterface、MCU 协议、诊断
    ├── kiwi_omni_controller/      # 三轮全向运动学、轮速、轮里程计
    ├── pacecat_lds_e110_driver/   # LDS-E110 生命周期节点、协议解析、LaserScan
    ├── kiwi_bringup/              # 硬件、传感器、状态发布、参数组合启动
    ├── kiwi_localization/         # EKF、AMCL 参数和定位启动
    ├── kiwi_slam/                 # SLAM Toolbox 参数和建图启动
    ├── kiwi_navigation/           # Nav2 参数、地图、速度和安全层启动
    ├── kiwi_mission_interfaces/   # ExecuteMission.action 等接口
    ├── kiwi_mission/              # 任务状态机、BasicNavigator 封装
    └── kiwi_tests/                # 运动学、协议、TF、launch、回放测试
firmware/
└── kiwi_base_mcu/                 # 独立固件工程：驱动、编码器、闭环和安全
~~~

每个包只拥有自己的参数和启动入口；不要把所有参数塞进一个巨大的 bringup 文件。建议用 git 子模块或 vendor 目录锁定外部参考代码，并在 README 记录上游 commit 和许可证。

## 5. TF、坐标系和消息契约

### 5.1 唯一 TF 拓扑

~~~text
map ──(SLAM Toolbox 或 AMCL，二选一)──> odom
odom ──(robot_localization EKF，仅一个发布者)──> base_footprint
base_footprint ──(robot_state_publisher 固定变换)──> base_link
base_link ──> lidar_link
base_link ──> imu_link
base_link ──> wheel_link_1/2/3
~~~

规则：

- 建图运行时只让 SLAM Toolbox 发布 map→odom；定位运行时只让 AMCL 发布 map→odom。
- EKF 发布 odom→base_footprint，控制器的原始 odom 可以发布话题，但配置 enable_odom_tf: false。
- Nav2、AMCL、SLAM Toolbox 统一使用 base_footprint；base_link 只表示机体几何参考。
- lidar_link 的原点和旋转方向必须按安装实测填写，不能靠 RViz 中“看起来差不多”。
- 启动后用 tf2_tools 检查每条链只有一个发布者，不能出现 map→base_link 直连和重复 odom TF。

### 5.2 核心话题

| 话题 | 类型 | 发布者 | 订阅者/用途 |
|---|---|---|---|
| /scan | sensor_msgs/msg/LaserScan | LDS-E110 驱动 | SLAM、AMCL、costmap、collision_monitor |
| /imu/data | sensor_msgs/msg/Imu | IMU 驱动 | EKF |
| /wheel/odom | nav_msgs/msg/Odometry | 全向控制器 | EKF、调试 |
| /odometry/filtered | nav_msgs/msg/Odometry | EKF | Nav2、调试 |
| /joint_states | sensor_msgs/msg/JointState | controller_manager | robot_state_publisher |
| /cmd_vel | geometry_msgs/msg/Twist | Nav2/遥控 | velocity_smoother |
| /cmd_vel_smoothed | geometry_msgs/msg/Twist | velocity_smoother | collision_monitor |
| /cmd_vel_safe | geometry_msgs/msg/Twist | collision_monitor | 全向控制器 |
| /omnidirectional_controller/cmd_vel_unstamped | geometry_msgs/msg/Twist | 速度桥/重映射 | 全向控制器 |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | 各驱动 | 监控和任务前置检查 |

Humble 基线统一使用 geometry_msgs/msg/Twist（非 stamped）。若后续切换 TwistStamped，必须在所有 remap 和控制器参数中同时切换，不能只改 Nav2 一端。

## 6. LDS-E110：硬件、协议和驱动实现

### 6.1 规格和型号差异

以下是 Pacecat 官方页面给出的系列参数；最终以设备铭牌和对应手册为准：

| 型号 | 扫描频率 | 角分辨率（对应频率） | 距离精度 | 强光环境 | 典型功耗 |
|---|---:|---:|---|---:|---:|
| LDS-E110-R | 7 Hz | 0.6° | ±20 mm（<2 m），±30 mm（≥2 m） | >30,000 lux | ≤1 W |
| LDS-E110-R-1 | 10 Hz | 0.83° | ±20 mm（<2 m），±30 mm（≥2 m） | >30,000 lux | ≤1 W |
| LDS-E110-R-3 | 10 Hz | 0.83° | ±10 mm（<2 m），±20 mm（≥2 m） | >80,000 lux | ≤1.5 W |

系列共同信息：905 nm，量程 0.1–12 m（90%反射率条件），UART 3.3 V，供电 5.1 V±0.2 V，尺寸约 38.5×38.5×34 mm，工作温度 -10～45 °C。

### 6.2 接线和电气注意事项

LDS-E110-R V1.5 手册给出的 ZH1.5-4PWB 线序：

| 线色 | 功能 | 连接 |
|---|---|---|
| 白 | TX 输出 | 接 USB-UART/主机 RX |
| 黄 | RX 输入 | 接 USB-UART/主机 TX；不用配置时可悬空 |
| 黑 | GND | 与主机信号地共地 |
| 红 | 电源正 | 独立稳定 5.1 V 电源 |

供电红线范围 4.9–5.3 V，启动电流不超过 500 mA，工作电流约 100–250 mA，纹波要求小于 100 mV。串口为 230400 8N1，TTL 高电平约 2.9–3.5 V。不要把 5 V UART 直接接到 3.3 V 输入；先确认 USB-UART 的电平。

建议：

- 雷达用独立 5 V 稳压支路，并在雷达附近加保险和去耦。
- 主机 USB-UART 用 udev 固定为 /dev/pacecat_e110，不在 launch 中写 ttyUSB0。
- 示波器或逻辑分析仪先确认 230400 波特率和空闲电平，再启动解析器。
- 长线使用屏蔽线、单点接地，避免电机 PWM 干扰雷达串口。

### 6.3 官方测量帧

手册描述的测量帧字段如下，整数均按小端解释：

~~~text
CF FA | N(uint16) | start_angle(uint16, 0.1°)
      | sector_span(uint16, 0.1°)
      | N × [intensity(uint8), distance_mm(uint16)]
      | checksum(uint16)
~~~

- 帧头是线上字节 CF FA，校验不包含帧头。
- checksum = N + start_angle + sector_span + 每个 intensity + 每个 distance，按 16 位累加取模。
- 一个 sector 通常约 18°；不能把单个 sector 当作完整 360°扫描。
- 角度原点为雷达旋转测量中心正前方，角度逆时针增加；转换到 LaserScan 时要与 lidar_link 的 x 轴方向一致。
- 每圈还可能出现 8 字节状态帧：53 54 a b c d 45 44。驱动应统计并暴露状态，但不要在未实测前臆测每个位的语义。

驱动解析器必须使用环形缓冲区，按帧头、长度、边界和校验逐步重同步；串口半帧、粘包、丢字节和重连都属于正常故障路径。把距离转换为米，对无效距离写 NaN 或按 LaserScan 约定过滤，不能用 0 m 伪造障碍物。

### 6.4 驱动路线

**快速验证路线：** 参考 BlueSeaLidar/bluesea-ros2 的 UART 节点，固定上游 commit，修改参数并在真实 E110 上验证。该仓库的通用 0xCE 原始包结构与 E110 的“6 字节头 + N×3 + 校验”形状相近，但型号状态分支没有明确列出 E110，因此不能只凭编译成功就认定协议完全兼容。其 package.xml 许可证字段仍为 TODO，量产前必须向供应方确认授权。

**生产路线：** 新建 pacecat_lds_e110_driver，使用 rclcpp_lifecycle::LifecycleNode 实现：

- configure：打开串口、读取参数、准备 ring buffer；
- activate：启动读取线程和 scan 发布；
- deactivate/cleanup：停止线程、关闭串口；
- parser：解析 CF FA 帧、校验、按角度聚合一整圈；
- scan builder：根据实测频率产生稳定 angle_min/max/increment、range_min/max 和 host timestamp；
- diagnostics：串口状态、帧率、丢帧、校验错误、最后一帧时间、温度/状态（若协议提供）；
- reconnect：设备拔插或串口错误后退避重连，不能让线程静默退出。

LaserScan 发布建议使用 SensorDataQoS；若沿用参考驱动，检查并修正其 QoS，确保 RViz、SLAM Toolbox 和 Nav2 costmap 都能收到数据。

### 6.5 E110 参数起点

以下是起始模板，不代表已验证值：

~~~yaml
pacecat_lds_e110:
  ros__parameters:
    frame_id: lidar_link          # 雷达自身坐标系，不要写 map
    scan_topic: scan
    type: uart
    port: /dev/pacecat_e110
    baud_rate: 230400
    raw_bytes: 3                  # 仅在参考驱动确认该模式对应 E110 后保留
    min_dist: 0.10
    max_dist: 12.0                # 导航阶段可收紧为 11.5
    output_360: true
    with_checksum: true
    from_zero: false              # 先按实际角度方向在 RViz 验证
    publish_rate: 10.0            # R 型改为 7.0；以实测帧率为准
~~~

验收时记录实际后缀、扫描频率、有效角度范围和帧率；若驱动只能输出不等角度数据，先在驱动层重采样，不能把错误的 angle_increment 交给 SLAM。

## 7. 三轮全向底盘运动学

### 7.1 约定

采用 REP-103 机体坐标：x 向前、y 向左、z 向上；角速度 wz 逆时针为正。以轮 1 的几何角 gamma 和轮的正转方向定义运动学，所有正负号必须与 URDF 和电机编码器实测一致。

参考 Humble 三轮控制器使用轮半径 r、底盘中心到轮的距离 L、几何角 gamma（配置单位为度）：

~~~text
wm1 = (-vy + L*wz) / r
wm2 = ( vx*cos(gamma) + vy*sin(gamma) + L*wz ) / r
wm3 = (-vx*cos(gamma) + vy*sin(gamma) + L*wz ) / r

vx = r/(2*cos(gamma)) * (wm2 - wm3)
vy = r/(sin(gamma)+1) * (-wm1 + 0.5*(wm2+wm3))
wz = r/L * [ sin(gamma)/(sin(gamma)+1)*wm1
             + 0.5/(sin(gamma)+1)*(wm2+wm3) ]
~~~

这组公式是参考实现的坐标约定，不是所有三轮布置的通用公式。若三轮以 120° 对称安装但轮 1 不在同一位置，应从轮接触点切向方向重新推导矩阵，或用实测的 3×3 矩阵替换。

### 7.2 控制器参数

三轮控制器可从 mateusmenezes95/omnidirectional_controllers 的 Humble 分支开始 fork，保留 MIT 署名；生产包建议命名为 kiwi_omni_controller。需要声明和验证：

~~~yaml
omnidirectional_controller:
  ros__parameters:
    wheel_names: [wheel_joint_1, wheel_joint_2, wheel_joint_3]
    robot_radius: 0.100       # 示例，替换为实测 L（m）
    wheel_radius: 0.053       # 示例，替换为实测 r（m）
    gamma: 30.0               # 参考控制器按“度”读取，再转弧度
    odom_frame_id: odom
    base_frame_id: base_footprint
    odom_numeric_integration_method: runge_kutta2
    open_loop: false
    enable_odom_tf: false     # TF 交给 EKF
    cmd_vel_timeout: 0.10
    velocity_rolling_window_size: 10
    use_stamped_vel: false
    publish_rate: 50.0
~~~

参考仓库的 publish_rate 声明和参数生命周期需要在 fork 时重新检查；不要未经编译和参数转储验证就直接复制。

### 7.3 符号与标定

为每个轮子保留独立符号 s_i∈{+1,-1}：

1. 轮子离地，低速点动，确认编码器正方向。
2. 机体向 +x、+y、+wz 各执行一次，记录三个轮速的符号和比例。
3. 在控制器中应用 s_i，重新做正向运动学，检查横向和角速度串扰。
4. 以激光或外部测量为参考，标定 r、L 和轮速比例；不要通过修改 gamma 掩盖机械安装误差。

建议的工程初值：纯平移测试中非目标方向速度绝对值小于目标方向的 5%；直行 10 m 的里程计距离误差目标小于 2–3%；原地旋转 360° 的航向误差目标小于 2°。这些是验收目标，不是理论保证。

## 8. MCU、传输协议和安全

### 8.1 推荐协议

CAN 优先，USB CDC/UART 作为调试或低成本方案。应用层帧建议：

~~~text
命令帧：
SOF | version | type=CMD | seq | timestamp_ms | enable
    | wheel_target_1_rad_s | wheel_target_2_rad_s | wheel_target_3_rad_s
    | reserved | CRC16

反馈帧：
SOF | version | type=STATE | seq | timestamp_ms
    | wheel_position_1/2/3 | wheel_velocity_1/2/3
    | current/voltage | fault_bits | estop | CRC16
~~~

字段编码、字节序和量程在接口文档中固定；收到未知版本或 CRC 错误时丢弃并计数。

### 8.2 频率与看门狗

- MCU 电机速度环：约 1 kHz。
- 主机命令：50–100 Hz。
- 编码器反馈：50–200 Hz。
- ROS 控制器命令超时起点：100 ms。
- MCU 命令看门狗：建议 100 ms 内没有新命令则将三轮目标降为零并关闭驱动使能（具体值由制动距离和风险评估确定）。
- 急停输入必须由 MCU/驱动硬件优先处理，ROS 节点掉线不能阻止急停。
- 上电默认禁能；清故障和重新使能需要明确状态转换，不能靠第一帧速度命令隐式开启。

### 8.3 ros2_control 硬件插件

新建 kiwi_hardware，实现 Humble 的 hardware_interface::SystemInterface 生命周期：

~~~text
on_init → on_configure → on_activate
       ↘ on_deactivate → on_cleanup → on_shutdown
       ↘ on_error
export_state_interfaces:  position/velocity（每轮）
export_command_interfaces: velocity（每轮）
read(): 读取 MCU 反馈、更新状态和故障
write(): 发送轮速目标、enable、seq、CRC
~~~

插件 XML 使用 pluginlib 导出，controller_manager 只依赖接口，不依赖具体 MCU 型号。read/write 中不得阻塞无限等待；串口/CAN 线程应有超时和锁定策略。增加一个“加载插件并完成 configure/activate/deactivate”的测试，防止只在真实机器人上发现 XML 或参数错误。

## 9. URDF/Xacro、关节和机器人状态

### 9.1 建议的 Xacro 结构

~~~text
kiwi_description/
├── urdf/
│   ├── kiwi.urdf.xacro
│   ├── materials.xacro
│   └── sensors.xacro
├── config/
│   └── ros2_control.yaml
└── rviz/
    └── model.rviz
~~~

kiwi.urdf.xacro 至少包含：

- base_footprint（地面投影，不参与惯量计算）；
- base_link（机体几何中心）；
- wheel_joint_1/2/3 和对应 wheel_link；
- lidar_link，原点为雷达旋转中心；
- imu_link，方向与 IMU 驱动输出的 frame_id 一致；
- ros2_control 标签，声明三路 velocity command interface、position/velocity state interface。

轮关节的 axis、origin 和 rotation 必须与电机正转测试一致。连续旋转轮关节使用 continuous；如果控制器需要轮角位置，编码器零点在 MCU 或硬件插件中处理，不能在 URDF 中伪造。

### 9.2 ros2_control 配置骨架

~~~yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    omnidirectional_controller:
      type: kiwi_omni_controller/OmnidirectionalController

kiwi_hardware:
  ros__parameters:
    transport: can0             # 或 serial
    device: can0
    command_timeout_ms: 100
    wheel_joints: [wheel_joint_1, wheel_joint_2, wheel_joint_3]
    # 下面的物理量必须与 URDF 和标定记录一致
    wheel_radius_m: 0.053
    robot_radius_m: 0.100
~~~

实际插件参数名称以实现代码为准，启动前用 ros2 param dump 和 controller_manager 的状态接口核对。

## 10. 里程计、IMU 和 EKF

### 10.1 数据流

~~~text
轮编码器 → kiwi_hardware → kiwi_omni_controller → /wheel/odom
IMU ─────────────────────────────────────────────→ /imu/data
/wheel/odom + /imu/data → robot_localization/ekf → /odometry/filtered
                                                     └→ odom→base_footprint TF
~~~

轮里程计保留原始值，便于诊断；Nav2 使用 /odometry/filtered。EKF 参数示例：

~~~yaml
ekf_filter_node_odom:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.20
    two_d_mode: true
    publish_tf: true
    print_diagnostics: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    odom0: /wheel/odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
~~~

数组顺序依次是 x、y、z、roll、pitch、yaw、vx、vy、vz、vroll、vpitch、vyaw、ax、ay、az。只有在 IMU 的 yaw 已经可靠标定并且不会和磁场干扰冲突时才融合绝对 yaw；室内通常先融合 gyro yaw rate（vyaw）和轮速 vx/vy，再由 AMCL/SLAM 提供全局约束。

检查点：

- IMU 时间戳必须来自消息采样时刻，不能每帧写当前时间掩盖延迟。
- covariance 不能全为零；未知量填 -1 或按驱动约定。
- wheel odom 和 IMU 的 frame_id、child_frame_id 与 TF 拓扑一致。
- /odometry/filtered 在机器人静止时速度应接近零，转向时不能出现持续漂移。

## 11. 建图：SLAM Toolbox

### 11.1 建图模式

建图启动顺序：

1. 启动 robot_state_publisher、硬件、雷达、IMU 和 EKF。
2. 确认 odom→base_footprint 和 base_footprint→lidar_link 连续可查。
3. 启动 SLAM Toolbox synchronous 或 asynchronous 节点。
4. 低速遥控机器人覆盖环境，先闭环再保存地图。
5. 用 map_saver_cli 保存 .yaml 与 .pgm；记录分辨率、原点和版本。

SLAM Toolbox 参数起点：

~~~yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    use_sim_time: false
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    scan_topic: /scan
    scan_queue_size: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05
    min_laser_range: 0.10
    max_laser_range: 11.5
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    throttle_scans: 1
    use_scan_matching: true
    use_scan_barycenter: true
    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
~~~

E110-R 只有约 7 Hz 时，应降低机器人平移/旋转速度，避免两帧之间运动过大；R-1/R-3 约 10 Hz 也要以实际帧率为准。地图质量主要受 TF、时间戳、轮里程计和雷达安装刚度影响，不是单纯提高分辨率就能改善。

建图期间不要同时启动 AMCL。保存地图前让机器人停止，等待最后一帧 scan 和 map 更新完成。

## 12. 定位：AMCL（Humble）

### 12.1 启动条件

定位模式加载已有地图，并关闭 SLAM Toolbox。启动 AMCL 前必须已有：

~~~text
map_server 发布 map
EKF 发布 odom→base_footprint
robot_state_publisher 发布 base_footprint→lidar_link
雷达发布 /scan（frame_id=lidar_link）
~~~

AMCL 参数起点：

~~~yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: base_footprint
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: /scan
    robot_model_type: nav2_amcl::OmniMotionModel
    laser_min_range: 0.10
    laser_max_range: 11.5
    laser_likelihood_max_dist: 2.0
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.10
    update_min_a: 0.10
    resample_interval: 1
    transform_tolerance: 0.2
    tf_broadcast: true
    set_initial_pose: false
~~~

这里使用完整插件名 nav2_amcl::OmniMotionModel，适合 Humble 的全向底盘。初始位姿通过 RViz 的 2D Pose Estimate 或任务接口 setInitialPose 设置；不要把 AMCL 的 map→odom TF 和 SLAM Toolbox 同时打开。

调参顺序：

1. 先确认 map、scan、TF 和激光量程；
2. 再调整 alpha1–alpha5（轮里程计/旋转噪声）；
3. 最后调整粒子数、更新距离和角度；
4. 记录不同地面、光照和动态障碍物下的重定位时间。

## 13. Nav2 导航层

### 13.1 第一阶段：DWB 全向基线

Humble 的 Nav2 默认示例面向差速底盘，Y 方向速度通常为零；本项目必须显式设置 Y 方向的限制和采样。建议先用 DWB，便于逐项观察速度、轨迹和代价。

~~~yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: -0.35
      max_vel_x: 0.35
      min_vel_y: -0.30
      max_vel_y: 0.30
      max_vel_theta: 1.2
      min_speed_xy: 0.0
      max_speed_xy: 0.45
      min_speed_theta: 0.0
      acc_lim_x: 0.8
      acc_lim_y: 0.8
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_y: -1.0
      decel_lim_theta: -2.5
      vx_samples: 15
      vy_samples: 15
      vtheta_samples: 20
      sim_time: 1.5
      short_circuit_trajectory_evaluation: true
      stateful: true
~~~

速度限制必须同时满足电机、轮胎打滑、地面摩擦、急停距离和雷达更新率。上面数值只是安全的低速起点。

### 13.2 Planner 与 costmap

~~~yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      update_frequency: 2.0
      publish_frequency: 1.0
      resolution: 0.05
      track_unknown_space: true
      footprint: "[[0.24, 0.00], [0.12, 0.22], [-0.14, 0.22], [-0.24, 0.00], [-0.14, -0.22], [0.12, -0.22]]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 4.0
      height: 4.0
      resolution: 0.05
      footprint: "[[0.24, 0.00], [0.12, 0.22], [-0.14, 0.22], [-0.24, 0.00], [-0.14, -0.22], [0.12, -0.22]]"
      plugins: ["voxel_layer", "inflation_layer"]
~~~

footprint 中的坐标必须替换成实测外轮廓，并给线束、外壳和制动误差留余量。若底盘确实近似圆形可使用 robot_radius，但三轮外壳通常用 polygon 更准确。

### 13.3 速度平滑与碰撞监测

~~~yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.35, 0.30, 1.2]
    min_velocity: [-0.35, -0.30, -1.2]
    max_accel: [0.8, 0.8, 2.0]
    max_decel: [-1.0, -1.0, -2.5]
    odom_topic: /odometry/filtered
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 0.5
    use_realtime_priority: false
    enable_stamped_cmd_vel: false

collision_monitor:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: /cmd_vel_smoothed
    cmd_vel_out_topic: /cmd_vel_safe
    transform_tolerance: 0.2
    source_timeout: 0.5
    stop_pub_timeout: 0.2
    polygons: ["StopPolygon", "SlowPolygon"]
    StopPolygon:
      type: "polygon"
      points: "[[0.30, 0.25], [0.30, -0.25], [-0.25, -0.25], [-0.25, 0.25]]"
      action_type: "stop"
      min_points: 4
      visualize: true
      polygon_pub_topic: "stop_polygon"
    SlowPolygon:
      type: "circle"
      radius: 0.60
      action_type: "slowdown"
      slowdown_ratio: 0.4
      visualize: true
      polygon_pub_topic: "slow_polygon"
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: /scan
      min_height: 0.05
      max_height: 0.50
      enabled: true
~~~

不同 Humble 补丁版本的参数名和 topic remap 需要以已安装包的样例为准。collision_monitor 只过滤 ROS 速度命令，不具备硬实时安全认证；急停仍接 MCU/驱动硬件。

### 13.4 第二阶段：MPPI Omni

当 DWB 已稳定后，可在 Humble 分支评估 nav2_mppi_controller：

~~~yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"
  vx_max: 0.35
  vx_min: -0.35
  vy_max: 0.30
  vy_min: -0.30
  wz_max: 1.2
~~~

MPPI 参数集较大，先用官方 Humble 示例，再逐步加入全向速度、约束和 critic。不要在 DWB 尚未通过基础验收时同时切换 planner、controller 和 EKF。

## 14. 启动文件和运行模式

### 14.1 启动文件职责

~~~text
kiwi_bringup/launch/hardware.launch.py       # controller_manager、硬件、joint_state
kiwi_bringup/launch/sensors.launch.py        # E110、IMU、robot_state_publisher
kiwi_localization/launch/ekf.launch.py        # robot_localization
kiwi_slam/launch/mapping.launch.py            # SLAM Toolbox + RViz（可选）
kiwi_localization/launch/localization.launch.py # map_server + AMCL
kiwi_navigation/launch/navigation.launch.py   # Nav2、smoother、collision monitor
kiwi_mission/launch/mission.launch.py        # 任务 action server
kiwi_bringup/launch/teleop.launch.py          # 键盘/手柄速度源
~~~

各 launch 支持 use_sim_time、namespace、参数文件路径和自动启动 controller 的开关。硬件模式默认 use_sim_time=false；仿真/rosbag 回放才传 true。

### 14.2 建图启动顺序

~~~bash
source /opt/ros/humble/setup.bash
source ~/kiwi_ws/install/setup.bash
ros2 launch kiwi_bringup hardware.launch.py
ros2 launch kiwi_bringup sensors.launch.py
ros2 launch kiwi_slam mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/kiwi_maps/site_01
~~~

每一步都先检查节点、话题、TF 和 diagnostics，再进入下一步。map_saver 生成 site_01.yaml 和 site_01.pgm 后，连同 git commit、雷达后缀和底盘标定参数一起归档。

### 14.3 定位导航启动顺序

~~~bash
source /opt/ros/humble/setup.bash
source ~/kiwi_ws/install/setup.bash
ros2 launch kiwi_bringup hardware.launch.py
ros2 launch kiwi_bringup sensors.launch.py
ros2 launch kiwi_localization localization.launch.py map:=~/kiwi_maps/site_01.yaml
ros2 launch kiwi_navigation navigation.launch.py
~~~

在 RViz 设置初始位姿；确认 AMCL 粒子云稳定后再发送导航目标。

## 15. 任务层设计

### 15.1 包和 action

kiwi_mission_interfaces 定义 ExecuteMission.action，建议内容：

~~~text
# Goal
string mission_id
kiwi_mission_interfaces/WaypointTask[] tasks
---
# Result
bool success
int32 result_code
string message
---
# Feedback
string state
uint32 current_task_index
float32 progress
string detail
~~~

WaypointTask 可包含 pose、frame_id、position_tolerance、yaw_tolerance、timeout、retries、arrival_action、metadata。动作类型用枚举或字符串白名单，不把任意脚本直接执行在导航进程中。

### 15.2 状态机

~~~text
IDLE
 └→ PRECHECK ─失败→ FAILED
       └→ LOCALIZING ─超时→ RECOVERY/FAILED
             └→ NAVIGATING
                    ├到达→ ACTION ─成功→ 下一任务
                    ├取消→ PAUSED/FAILED
                    ├超时→ RECOVERY（限次数）
                    └故障/急停→ PAUSED
最后一个任务完成→ COMPLETED
~~~

前置检查至少包括：E-stop 已释放、MCU heartbeat 正常、雷达帧率正常、EKF 有效、TF 可查、AMCL 粒子有效、电池电压足够。任务层只调用 Nav2 action，不直接发布轮速；手动遥控和任务命令要通过同一速度安全链路仲裁。

### 15.3 Python 任务节点骨架

~~~python
#!/usr/bin/env python3
"""任务层只编排导航和动作，不直接控制电机。"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def run_one_waypoint(navigator: BasicNavigator, pose: PoseStamped) -> bool:
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            # 将反馈转换为 ExecuteMission.action 的 feedback
            pass
    result = navigator.getResult()
    return result == TaskResult.SUCCEEDED
~~~

正式实现要补齐 action server、取消回调、任务持久化、幂等 task_id、超时计时器和恢复策略。恢复行为应有白名单，例如清除代价地图、原地小角度重定位、回退到安全点；不得无限自动重试。

## 16. 安装、构建和设备权限

### 16.1 Ubuntu 22.04/Humble 依赖

~~~bash
sudo apt update
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools \
  ros-humble-teleop-twist-keyboard \
  ros-humble-diagnostic-updater \
  ros-humble-nav2-simple-commander
~~~

velocity smoother、collision monitor 和 MPPI 的包名可能随 Humble 补丁仓库拆分，使用 apt-cache policy 或 rosdep 确认后安装。不要混装 Jazzy/Rolling 二进制包。

### 16.2 构建

~~~bash
source /opt/ros/humble/setup.bash
cd ~/kiwi_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# ROS Humble 的 Python 依赖应使用系统 Python 3.10，避免 conda 覆盖 PATH
export PATH=/usr/bin:/bin:$PATH
which python3
python3 --version
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
~~~

本次验证中，BlueSea 参考雷达仓库在已安装的 Humble 环境下使用系统 Python 构建通过；默认拾取 conda Python 3.14 时，rosidl_adapter 缺少 em 模块而失败。因此把“禁用 conda 覆盖”写入开发机说明和 CI，而不是在代码里绕过依赖。

### 16.3 udev 固定串口名

先读取真实设备属性：

~~~bash
udevadm info -a -n /dev/ttyUSB0 | rg 'idVendor|idProduct|serial'
~~~

再按实际 VID、PID、序列号创建 /etc/udev/rules.d/99-kiwi-devices.rules：

~~~text
SUBSYSTEM=="tty", ATTRS{idVendor}=="<真实VID>", ATTRS{idProduct}=="<真实PID>", ATTRS{serial}=="<真实序列号>", SYMLINK+="pacecat_e110", GROUP="dialout", MODE="0660"
~~~

加入 dialout 后重新登录：

~~~bash
sudo usermod -aG dialout $USER
sudo udevadm control --reload-rules
sudo udevadm trigger
~~~

不要用 chmod 777 或把未确认的 VID/PID 写入提交仓库。

## 17. 分阶段实施计划

### 阶段 0：冻结接口和安全（1–2 天）

- 建立 kiwi_ws、firmware 工程和 git 仓库。
- 确认 E110 后缀、MCU、电机驱动、IMU、供电和急停。
- 写出 CAN/UART 协议、版本号、CRC 和故障码。
- 机械测量 L、r、gamma、底盘 footprint，并拍照记录坐标方向。
- 验收：急停断能、上电默认禁能、协议文档可供两端独立实现。

### 阶段 1：硬件和传感器单测（2–4 天）

- MCU 完成编码器、电机闭环、看门狗、急停和反馈帧。
- E110 接主机，验证 230400 8N1、帧头、校验、角度方向和帧率。
- IMU 发布正确 frame_id、时间戳和 covariance。
- 验收：雷达连续运行 10 分钟无校验错误；急停和拔线都能安全停机。

### 阶段 2：ros2_control 和运动学（3–5 天）

- 完成 kiwi_hardware 生命周期和插件加载测试。
- 接入全向控制器、joint_state_broadcaster 和轮里程计。
- 完成 +x、+y、+wz 三组纯运动标定，保存参数版本。
- 验收：无重复 TF；轮速命令限幅、超时归零；横向串扰小于目标的 5%。

### 阶段 3：EKF 和 TF（1–2 天）

- 接入 robot_localization，关闭控制器 odom TF。
- 用 rosbag 记录轮速/IMU/scan，检查时间同步和延迟。
- 验收：静止漂移、直行误差和旋转误差达到项目目标；TF 无 extrapolation。

### 阶段 4：建图（2–4 天）

- 低速覆盖环境，调 scan 量程、滤波和 SLAM 参数。
- 至少闭环两次，保存地图和参数。
- 验收：地图无明显双墙/重影，重复路线回到同一位置。

### 阶段 5：AMCL + Nav2（3–5 天）

- 加载地图，使用 OmniMotionModel，验证初始位姿和重定位。
- 先 DWB，再 velocity_smoother 和 collision_monitor；最后才评估 MPPI。
- 验收：固定测试点导航成功率、横向移动、障碍物停止和急停均达标。

### 阶段 6：任务层和回归（2–4 天）

- 完成 ExecuteMission action、状态机、反馈、取消和有限重试。
- 用 rosbag/仿真做协议、运动学、TF、导航和任务回归。
- 验收：任务可暂停/取消/恢复，重复 task_id 不会重复执行危险动作。

## 18. 验收与诊断清单

~~~text
系统/版本
□ lsb_release 为 Ubuntu 22.04
□ ros2 doctor 无发行版混用
□ 所有包来自 Humble 或已审计源码

雷达
□ 型号后缀和 7/10 Hz 记录在配置
□ 230400 8N1、CF FA、长度和 checksum 已抓包确认
□ /scan 的 frame_id=lidar_link，角度方向在 RViz 正确
□ 10 分钟校验错误=0、帧率稳定、拔插可重连

底盘
□ 三个 wheel_joint 名称唯一，r/L/gamma 与实物记录一致
□ +x、+y、+wz 纯运动方向正确
□ 命令超时和 MCU watchdog 都能归零
□ 急停不依赖 ROS

TF/定位
□ map→odom 只有 AMCL 或 SLAM Toolbox 一个发布者
□ odom→base_footprint 只有 EKF 一个发布者
□ base_footprint→lidar_link 可查且无时间外推
□ AMCL 使用 nav2_amcl::OmniMotionModel

导航/任务
□ local/global costmap footprint 为实测外轮廓
□ DWB 的 min/max_vel_y、vy_samples 非零
□ smoother、collision monitor、控制器 topic 串接正确
□ ExecuteMission 的取消、超时、重试和急停状态经过实测
~~~

建议每次标定或参数调整都保存：git commit、参数 yaml、rosbag、RViz 截图、测试日期、硬件序列号和结论。这样才能区分“代码回归”与“机械/场地变化”。

## 19. 风险和明确的下一步

### 19.1 主要风险

1. **E110 型号/协议差异**：R、R-1、R-3 的频率和精度不同；通用驱动的状态分支未明确支持 E110，必须抓包确认。
2. **轮子正方向不一致**：运动学公式正确但电机符号错误，会表现为斜行、旋转串扰。
3. **时间戳和 TF 错误**：SLAM/AMCL 的“漂移”经常是延迟或 frame 配错，而不是算法参数。
4. **过度依赖软件安全层**：collision_monitor 或 Nav2 崩溃时，硬件急停和 MCU watchdog 仍需工作。
5. **许可证和版本漂移**：参考雷达仓库许可证字段为 TODO；必须确认授权并固定 commit。
6. **Humble 依赖环境**：conda Python 覆盖系统 Python 会导致 rosidl_adapter/colcon 构建异常。

### 19.2 建议的第一周交付物

- E110 实物后缀、接线照片、原始串口抓包和 parser 单元测试；
- 底盘尺寸/符号表、MCU 协议 v1、急停测试记录；
- kiwi_description 能在 RViz 显示，TF 树无重复；
- 三轮控制器可完成 +x/+y/+wz，输出 /wheel/odom；
- 一份可复现的 Humble 安装和构建脚本；
- 第一张闭环地图及对应参数、rosbag 和验收结论。

## 20. 参考资料

以下链接是方案中实际采用的官方文档或开源仓库；阅读时请优先选择对应的 humble 分支和版本。

- Pacecat LDS-E110 系列官方规格：https://www.pacecat.net/product/lds-e110-series-lidar-sensor/
- Pacecat 中文规格页：https://www.pacecat.com/product/lds-e110-series-lidar-sensor/
- Pacecat LDS-E110-R V1.5 使用说明书（接线与串口协议）：https://www.pacecat.com/shared-files/940/?LDS-E110-R-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6-V1.5.pdf=
- Pacecat 官方下载页：https://www.pacecat.net/downloads/
- BlueSea ROS 2 雷达驱动参考：https://github.com/BlueSeaLidar/bluesea-ros2
- 三轮全向 ROS 2 Humble 控制器参考（MIT）：https://github.com/mateusmenezes95/omnidirectional_controllers/tree/humble
- ros2_control Humble 自定义硬件组件：https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html
- ROS 2 Humble Ubuntu 安装：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Nav2 Humble 默认参数：https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml
- Nav2 AMCL 配置：https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Nav2 Humble MPPI（Omni）：https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller
- Nav2 Humble collision monitor：https://github.com/ros-navigation/navigation2/tree/humble/nav2_collision_monitor
- collision monitor 参数样例：https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/params/collision_monitor_params.yaml
- SLAM Toolbox Humble：https://github.com/SteveMacenski/slam_toolbox/tree/humble
- Nav2 Simple Commander Humble：https://github.com/ros-navigation/navigation2/tree/humble/nav2_simple_commander
- robot_localization Humble：https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel

## 21. 交付前检查记录

本文件生成后应执行以下命令，并把结果作为交付依据：

~~~bash
file -bi /home/ym/MSHEEP/ros2_humble_lds_e110_omni_navigation_plan.md
iconv -f UTF-8 -t UTF-8 /home/ym/MSHEEP/ros2_humble_lds_e110_omni_navigation_plan.md >/dev/null
rg -n 'Ubuntu 22\\.04|ROS 2 Humble|LDS-E110|230400|CF FA|OmniMotionModel|kiwi_hardware|ExecuteMission' \
  /home/ym/MSHEEP/ros2_humble_lds_e110_omni_navigation_plan.md
rg -n '^~~~' /home/ym/MSHEEP/ros2_humble_lds_e110_omni_navigation_plan.md | wc -l
~~~

代码块数量必须为偶数；命令中的示例值仍需在接入实物时替换并经过测试记录。
