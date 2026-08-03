# ROS 2 Humble + Ubuntu 22.04 三轮全向底盘建图导航实施方案

> 版本：2026-08-03
> Git 基线：899a0a4（硬件手册适配前）
> 目标平台：Ubuntu 22.04 LTS + ROS 2 Humble
> 激光雷达：Pacecat LDS-E110-R，依据本地 V1.5 手册
> 底盘控制器：DJI RoboMaster 开发板 C 型（STM32F407）
> 底盘：三轮全向（Kiwi drive）
> 文档定位：从电机/编码器、MCU、ROS 2 驱动，到建图、定位、导航和任务层的落地设计。
> 说明：这是从零搭建的独立工程方案。开源仓库只作为协议、接口和算法参考，不假定现有工程可以直接使用。

## 1. 先给结论

推荐采用下面的基线：

1. 主机运行 Ubuntu 22.04 和 ROS 2 Humble，所有 Humble 软件包固定在同一发行版。
2. RoboMaster C 型板负责底盘实时控制，通过 CAN1 连接三个外置电调；该开发板不是电机功率驱动器。
3. 第一版让 C 型板通过 USB CDC 与主机通信；若实测抗干扰不足，再使用隔离 USB-CAN 接 CAN2。应用协议必须有序号、时间戳、CRC、使能位和看门狗。
4. 用 ros2_control 的自定义 SystemInterface 管理三路轮子的命令与反馈，再接一个三轮全向控制器。
5. 里程计由轮编码器计算，轮里程计和 IMU 用 robot_localization EKF 融合；EKF 唯一发布 odom 到 base_footprint 的 TF。
6. 建图使用 SLAM Toolbox；保存地图后，定位使用 AMCL，并明确使用 Humble 的 OmniMotionModel。
7. Nav2 第一阶段使用 DWBLocalPlanner，显式打开 Y 方向速度采样；后续可评估 Humble 的 MPPI Omni 模型。
8. 速度链路固定为：Nav2 → velocity_smoother → collision_monitor → 全向控制器 → C 型板 → 外置电调。硬件急停和通信看门狗独立于这条链路。
9. LDS-E110-R 直接接主机 USB-UART，并使用独立 5.1 V 电源；不要从 C 型板标注 100 mA 的接口给雷达供电，也不要把雷达 frame_id 写成 map。

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
| USB CDC（首版）或隔离 USB-CAN：命令、反馈、故障、急停状态            |
+------------------------------- MCU/执行层 ---------------------------+
| RoboMaster C 板：闭环、BMI088、ADC、电调通信、看门狗、急停状态       |
+---------------------------------------------------------------------+
| 三个外置电调 + 电机/编码器 | LDS-E110-R | 电源/BMS/硬件急停         |
~~~

### 2.2 主机与底盘连接

建议的实物链路：

~~~text
Ubuntu 22.04 SBC
  ├─ USB-UART（3.3 V TTL）─────────────── LDS-E110-R
  │                                          └─ 独立 5.1 V/≥500 mA 供电
  └─ USB CDC（第一版）──────────────────── RoboMaster C 型板
                                             ├─ CAN1 → 3 个外置电调
                                             ├─ BMI088 六轴 IMU
                                             ├─ IST8310 磁力计（默认不融合航向）
                                             ├─ ADC_BAT 电池电压
                                             └─ 看门狗与急停状态
~~~

LDS-E110-R 保持直连主机，不经过 C 型板转发。若 USB CDC 抗干扰不足，可将主机侧改为隔离 USB-CAN → C 型板 CAN2；CAN1 继续专用于电调。两条 CAN 总线的终端电阻都必须实测，手册没有说明板载 120 Ω 终端。

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
| 雷达 | Pacecat LDS-E110-R，V1.5 手册 |
| MCU | DJI RoboMaster 开发板 C 型，STM32F407 |
| MCU 传感器 | BMI088 六轴 IMU、IST8310 磁力计、ADC_BAT |
| MCU/主机链路 | 第一版 USB CDC；备选隔离 USB-CAN → CAN2 |
| 驱动消息 | sensor_msgs/msg/LaserScan |
| 底盘类型 | 三轮全向 Kiwi drive |
| 定位 | AMCL |
| 建图 | SLAM Toolbox |
| 任务接口 | ROS 2 action，内部用 Nav2 Simple Commander |

### 3.2 开工前必须填写的实物数据

| 数据 | 用途 | 当前状态 |
|---|---|---|
| 三轮中心到轮接触点距离 L | 逆/正运动学 | 待测量 |
| 轮半径 r、轮宽 | 轮速和 URDF | 待测量 |
| 三轮安装角 gamma、正方向 | 控制器参数 | 待测量 |
| 电机、电调型号及 CAN 协议 | C 型板闭环和 kiwi_hardware | 待确认 |
| 电机减速比、编码器线数/反馈单位 | 轮位置和轮速度 | 待确认 |
| BMI088 相对底盘安装方向 | IMU frame 和 EKF | 待测量 |
| STM32F407 完整料号、主频和存储 | 固件资源预算 | 待从芯片丝印/固件工程确认 |
| CAN1/CAN2 是否带终端电阻 | 总线布线 | 待实测 |
| 底盘外轮廓 | costmap footprint、碰撞监测 | 待测量 |
| 计算机型号与总线 | CPU 预算、设备名 | 待确认 |

没有这些数据时，可以先做软件仿真和传感器单测，但不能声称运动学已经标定。

## 4. ROS 2 工作空间和包边界

工作空间固定为 /home/ym/ros2_ws，包名保持稳定，便于后续换电机或电调：

~~~text
ros2_ws/
└── src/
    ├── kiwi_description/          # URDF/Xacro、TF、碰撞模型、传感器安装位
    ├── kiwi_hardware/             # RoboMaster C SystemInterface、USB/CAN 协议、诊断
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
└── robomaster_c_base/             # C 型板固件：CAN 电调、BMI088、协议和安全
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
| /imu/data_raw | sensor_msgs/msg/Imu | C 型板 BMI088 bridge | EKF |
| /battery_state | sensor_msgs/msg/BatteryState | C 型板 ADC_BAT bridge | diagnostics、任务前置检查 |
| /wheel/odom | nav_msgs/msg/Odometry | 全向控制器 | EKF、调试 |
| /odometry/filtered | nav_msgs/msg/Odometry | EKF | Nav2、调试 |
| /joint_states | sensor_msgs/msg/JointState | controller_manager | robot_state_publisher |
| /cmd_vel | geometry_msgs/msg/Twist | Nav2/遥控 | velocity_smoother |
| /cmd_vel_smoothed | geometry_msgs/msg/Twist | velocity_smoother | collision_monitor |
| /cmd_vel_safe | geometry_msgs/msg/Twist | collision_monitor | 全向控制器 |
| /omnidirectional_controller/cmd_vel_unstamped | geometry_msgs/msg/Twist | 速度桥/重映射 | 全向控制器 |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | 各驱动 | 监控和任务前置检查 |

Humble 基线统一使用 geometry_msgs/msg/Twist（非 stamped）。若后续切换 TwistStamped，必须在所有 remap 和控制器参数中同时切换，不能只改 Nav2 一端。

## 6. LDS-E110-R：硬件、协议和驱动实现

### 6.1 本地 V1.5 手册确认参数

型号已经由本地文件 /home/ym/ros2_ws/documents/LDS-E110-R-激光雷达使用说明书-V1.5.pdf 确认为 LDS-E110-R。以下参数以这份手册为准，而不是继续使用早期网页中的系列值：

| 项目 | LDS-E110-R V1.5 参数 | 手册位置 |
|---|---|---|
| 扫描频率 | 典型 7 Hz，即 420 RPM | 正文第 2、7 页 |
| 测量频率 | 4200 点/s | 正文第 3、7 页 |
| 扫描范围/角分辨率 | 360° / 0.6° | 正文第 7 页 |
| 检测距离 | 0.05–12 m@90%反射率；0.05–6 m@10%反射率 | 正文第 7 页 |
| 测距精度 | ±15 mm（<2 m）；±30 mm（≥2 m） | 正文第 7 页 |
| 波长/激光安全 | 905 nm±15 nm；Class 1 | 正文第 2、7 页 |
| 环境光 | >60,000 lux | 正文第 7 页 |
| 功耗/供电 | <1.5 W；5.1 V±0.2 VDC | 正文第 7–8 页 |
| 工作温度 | -10～50 °C | 正文第 7 页 |
| 外形尺寸 | 38.5×38.5×33.7 mm | 正文第 7 页 |

雷达内部电机物理上顺时针旋转，但输出坐标定义为正前方 +X/0°，角度逆时针增大。正装且 lidar_link 的 +Z 向上时与 ROS 坐标约定一致；驱动仍需按包内角度排序，不能直接按串口到达顺序构造递增角度。

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

- 雷达用独立 5.1 V 稳压支路，并在雷达附近加保险和去耦。
- 禁止使用 RoboMaster C 型板 UART2、CAN2 或独立 5 V 接口标注的 5 V@100 mA 支路给雷达供电；其启动电流不满足要求。
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
- 数据包总长度为 10 + 3N 字节；必须先检查 N 和剩余长度，再访问点数据。
- checksum = N + start_angle + sector_span + 每个 intensity + 每个 distance，按 16 位累加取模。手册文字公式漏写 sector_span，但同页数值示例明确加入 0x00B4，因此实现和测试采用数值示例，并用实机抓包再次确认。
- V1.5 示例为每扇区 18°、30 点，标称一圈 20 包、约 600 点；驱动不能硬编码，必须使用包内 N、起始角度和跨度。
- 角度原点为雷达旋转测量中心正前方，角度逆时针增加；转换到 LaserScan 时要与 lidar_link 的 x 轴方向一致。
- 每圈发送一次 8 字节状态帧：53 54 a b c d 45 44。手册只定义 a 的单位、强度、去拖点、滤波、扇区和固定角分辨率位；b/c/d 未定义，驱动只原样记录并发布诊断。

驱动解析器必须使用环形缓冲区，按帧头、长度、边界和校验逐步重同步；串口半帧、粘包、丢字节和重连都属于正常故障路径。协议没有设备时间戳，驱动应根据主机接收时间和实测圈周期生成时间信息。把距离转换为米，对无效距离写 NaN 或按 LaserScan 约定过滤，不能用 0 m 伪造障碍物。

### 6.4 驱动路线

**快速验证路线：** V1.5 手册明确推荐 BlueSeaLidar/bluesea-ros2。当前工作区已将该仓库固定在 commit 206ade9，且已在 ROS 2 Humble 下使用系统 Python 构建通过。不过仓库没有专用 E110 启动参数，源码中的型号状态分支也未明确列出 E110；必须用本地手册的 CF FA 抓包和 checksum 对照验证。其 package.xml 许可证字段仍为 TODO，量产前需向供应方确认授权。

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
    min_dist: 0.05                # V1.5 手册额定最小量程
    max_dist: 12.0                # 90%反射率额定值；导航可实测后收紧
    output_360: true
    with_checksum: true
    from_zero: false              # 先按实际角度方向在 RViz 验证
    publish_rate: 7.0             # LDS-E110-R 标称值；最终以实测圈周期为准
~~~

LaserScan 标称时间起点为 scan_time≈0.142857 s、time_increment≈0.000238095 s、angle_increment≈0.01047198 rad。它们分别来自 1/7、1/4200 和 0.6°换算，只能作为初值；发布时优先使用实测圈周期和包内角度。若驱动只能输出不等角度数据，先在驱动层重采样，不能把错误的 angle_increment 交给 SLAM。

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
    # robot_radius、wheel_radius、gamma 尚未测量。
    # 在三项真实值写入前，bringup 必须拒绝激活该控制器。
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

## 8. RoboMaster C 型板、传输协议和安全

### 8.1 手册确认的板级资源

本节依据 /home/ym/ros2_ws/documents/RoboMaster开发板C型用户手册.pdf。手册明确主控为 STM32F407，但没有给出完整芯片后缀、主频、Flash 或 SRAM 容量，因此这些值不得按常见 F407 配置直接写死。

| 资源 | 手册确认值 | 对本项目的用途 |
|---|---|---|
| 输入与环境 | XT30，DC 8–28 V；0–55 ℃；60×41×16.3 mm；38 g（正文第 17 页） | 接底盘电源，监控温度和电池 |
| 电源保护 | 防反接、缓启动、超过 28 V 时后级关断（正文第 3、6 页） | 仍需外部保险、BMS 和急停 |
| CAN | 两路，最高 1 Mbps（正文第 11–12 页） | CAN1 接三个电调；CAN2 作为主机链路备选 |
| UART | 两路，可配置波特率，支持手册所述 3.3/5 V 电平（正文第 10–11 页） | 调试或备用通信，不接 RS232/RS485 原始电平 |
| USB | USB 2.0 Full Speed，支持通信和 DFU（正文第 8–9 页） | 第一版主机链路使用自定义 USB CDC |
| PWM | 7 路，TIM1_CH1–4、TIM8_CH1–3（正文第 12、18 页） | 仅适合外部 PWM 模块/舵机，不是三相功率驱动 |
| 六轴 IMU | BMI088，SPI 最高 10 MHz，带加热控制（正文第 15–16 页） | 发布角速度和加速度给 EKF |
| 磁力计 | IST8310，I²C 最高 400 kHz，地址 0x0E（正文第 16 页） | 室内磁场验证后再决定是否使用 |
| 电池检测 | PF10/ADC_BAT（正文第 15、18 页） | 上报输入电压和低电压诊断 |

接口细节：

- CAN1：2-pin，1=CANL、2=CANH；STM32 映射 PD1/TX、PD0/RX。
- CAN2：4-pin，1=5V、2=GND、3=CANH、4=CANL；STM32 映射 PB6/TX、PB5/RX。
- 手册未说明板载 CAN 终端电阻，CAN1/CAN2 都必须断电测量，再按总线两端各 120 Ω 的实际拓扑配置。
- 外壳丝印 UART1 实际对应 STM32 UART6（PG14/PG9），丝印 UART2 实际对应 STM32 UART1（PA9/PB7）；固件和线束表必须同时写丝印名与 MCU 外设名。
- VCC_5V_M 是 PWM 外设 5 V 电源，总上限 5 A；VCC_5V 是板载器件 5 V 电源，总上限 1 A。UART2、CAN2 和独立 5 V 接口在电源框图中标为 5 V@100 mA，不能给 LDS-E110-R 供电。
- 手册没有定义专用正交编码器接口，也没有三相电机功率级。三个电机必须使用带反馈的外置电调，或者后续增加独立编码器采集硬件。

### 8.2 推荐硬件链路

~~~text
Ubuntu 22.04 / ROS 2 Humble 主机
  ├─ USB-UART → LDS-E110-R（独立 5.1 V 电源）
  └─ USB CDC → RoboMaster C 型板
                    ├─ CAN1 → 电调 1/2/3 → 电机/编码器
                    ├─ BMI088 → gyro + accel
                    ├─ ADC_BAT → 电池电压
                    └─ GPIO → 急停状态/驱动使能

备选：隔离 USB-CAN → CAN2；CAN1 仍只连接电调
~~~

手册只确认 USB 具备通信能力，不提供本项目需要的 CDC 应用协议；USB 描述符、收发缓冲和 CRC 协议均由 robomaster_c_base 固件实现。USB 给板载 STM32/部分器件供电时，不能同时假定它给 PWM 外设电源供电。

### 8.3 主机与 C 型板协议

应用层帧建议：

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

反馈帧建议至少增加 BMI088 和电池字段；若单帧过长，可拆为同一时间戳下的 WHEEL_STATE、IMU_STATE、POWER_STATE 和 FAULT_STATE：

~~~text
WHEEL_STATE: seq | mcu_time_us | pos[3] | vel[3] | current[3] | fault[3]
IMU_STATE:   seq | mcu_time_us | gyro[3] | accel[3] | temperature
POWER_STATE: seq | mcu_time_us | battery_voltage | estop | enable
~~~

MCU 时间戳要单调递增，主机节点维护 MCU 时间到 ROS 时间的偏移估计。所有消息使用固定字节序和明确的缩放单位，协议版本变化必须可检测。

### 8.4 频率与看门狗

- C 型板到电调的控制与反馈：工程目标 500 Hz–1 kHz，最终由电调 CAN 协议决定。
- 主机命令：50–100 Hz。
- 编码器反馈：50–200 Hz。
- ROS 控制器命令超时起点：100 ms。
- C 型板命令看门狗：建议 100 ms 内没有有效新命令则将三轮目标降为零并关闭电调使能（具体值由制动距离和风险评估确定）。
- 急停输入必须由 C 型板/电调硬件优先处理，ROS 节点掉线不能阻止急停。
- 上电默认禁能；清故障和重新使能需要明确状态转换，不能靠第一帧速度命令隐式开启。
- BMI088 读取建议 500–1000 Hz，向主机发布 100–200 Hz；这些是工程目标，不是手册保证值。

### 8.5 ros2_control 硬件插件

新建 kiwi_hardware，其中硬件类命名为 RoboMasterCSystem，实现 Humble 的 hardware_interface::SystemInterface 生命周期：

~~~text
on_init → on_configure → on_activate
       ↘ on_deactivate → on_cleanup → on_shutdown
       ↘ on_error
export_state_interfaces:  position/velocity（每轮）
export_command_interfaces: velocity（每轮）
read(): 读取 C 型板反馈、更新轮状态和故障
write(): 发送轮速目标、enable、seq、CRC
~~~

插件 XML 使用 pluginlib 导出，controller_manager 只依赖接口。read/write 中不得阻塞等待 USB/CAN；传输线程使用有界队列、超时和重连状态机。BMI088 数据可由同一传输所有者发布 /imu/data_raw，避免两个节点同时打开 C 型板。增加协议 CRC/半帧/重连单测，以及“加载插件并完成 configure/activate/deactivate”的测试。

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
    hardware_plugin: kiwi_hardware/RoboMasterCSystem
    transport: usb_cdc
    device: /dev/robomaster_c
    fallback_transport: socketcan
    fallback_can_interface: can0
    command_timeout_ms: 100
    wheel_joints: [wheel_joint_1, wheel_joint_2, wheel_joint_3]
    protocol_version: 1
    publish_imu_raw: true
    publish_battery_state: true
    # wheel_radius_m、robot_radius_m、gamma_deg 均待实测。
    # 参数缺失时必须配置失败，禁止使用演示尺寸。
~~~

实际插件参数名称以实现代码为准，启动前用 ros2 param dump 和 controller_manager 的状态接口核对。

## 10. 里程计、IMU 和 EKF

### 10.1 数据流

~~~text
C 型板 CAN 电调反馈 → kiwi_hardware → kiwi_omni_controller → /wheel/odom
C 型板 BMI088 ─────→ kiwi_hardware/bridge ─────────────────→ /imu/data_raw
/wheel/odom + /imu/data_raw → robot_localization/ekf → /odometry/filtered
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
    imu0: /imu/data_raw
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: false
~~~

数组顺序依次是 x、y、z、roll、pitch、yaw、vx、vy、vz、vroll、vpitch、vyaw、ax、ay、az。当前 BMI088 基线只融合 gyro yaw rate（vyaw）和轮速 vx/vy。BMI088 本身不提供磁航向；板载 IST8310 位于电机和大电流线附近，完成硬铁/软铁标定及不同电机工况验证前，不融合绝对 yaw。

C 型板手册说明 BMI088 通过最高 10 MHz SPI 连接，并提供 PF6/TIM10_CH1 加热控制。加热恒温点建议高于正常板温 15–20 ℃，但控制参数必须结合实际安装和温升测试；温度、加热占空比和传感器异常均进入 diagnostics。

检查点：

- IMU 时间戳必须来自消息采样时刻，不能每帧写当前时间掩盖延迟。
- orientation 未计算时将 orientation_covariance[0] 设为 -1，不能发布单位四元数冒充有效航向。
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
    min_laser_range: 0.05
    max_laser_range: 12.0
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

E110-R 标称只有 7 Hz，应降低机器人平移/旋转速度，避免两帧之间运动过大。地图质量主要受 TF、时间戳、轮里程计和雷达安装刚度影响，不是单纯提高分辨率就能改善。若底盘或外壳遮挡 0.05 m 近场数据，应做明确的角度/自体过滤并记录，而不是随意修改驱动额定量程。

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
    laser_min_range: 0.05
    laser_max_range: 12.0
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
      footprint: "__MEASURED_FOOTPRINT_REQUIRED__"
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
      footprint: "__MEASURED_FOOTPRINT_REQUIRED__"
      plugins: ["voxel_layer", "inflation_layer"]
~~~

__MEASURED_FOOTPRINT_REQUIRED__ 是故意设置的不可运行哨兵值：底盘外轮廓未测量前，导航 bringup 应失败，而不是套用演示尺寸。测量时给线束、外壳和制动误差留余量；若底盘确实近似圆形可使用 robot_radius，但三轮外壳通常用 polygon 更准确。

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
      points: "__MEASURED_STOP_POLYGON_REQUIRED__"
      action_type: "stop"
      min_points: 4
      visualize: true
      polygon_pub_topic: "stop_polygon"
    SlowPolygon:
      type: "circle"
      radius: "__MEASURED_SLOW_RADIUS_REQUIRED__"
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

两个 __MEASURED_* 安全区参数也必须在实物测量和制动测试后填写。不同 Humble 补丁版本的参数名和 topic remap 需要以已安装包的样例为准。collision_monitor 只过滤 ROS 速度命令，不具备硬实时安全认证；急停仍接 C 型板/电调硬件。

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
source /home/ym/ros2_ws/install/setup.bash
ros2 launch kiwi_bringup hardware.launch.py
ros2 launch kiwi_bringup sensors.launch.py
ros2 launch kiwi_slam mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/kiwi_maps/site_01
~~~

每一步都先检查节点、话题、TF 和 diagnostics，再进入下一步。map_saver 生成 site_01.yaml 和 site_01.pgm 后，连同 git commit、LDS-E110-R 手册版本和底盘标定参数一起归档。

### 14.3 定位导航启动顺序

~~~bash
source /opt/ros/humble/setup.bash
source /home/ym/ros2_ws/install/setup.bash
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

前置检查至少包括：E-stop 已释放、RoboMaster C 型板 heartbeat 正常、三个电调在线、BMI088 正常、雷达帧率正常、EKF 有效、TF 可查、AMCL 粒子有效、电池电压足够。任务层只调用 Nav2 action，不直接发布轮速；手动遥控和任务命令要通过同一速度安全链路仲裁。

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
  ros-humble-nav2-simple-commander \
  can-utils
~~~

velocity smoother、collision monitor 和 MPPI 的包名可能随 Humble 补丁仓库拆分，使用 apt-cache policy 或 rosdep 确认后安装。不要混装 Jazzy/Rolling 二进制包。

### 16.2 构建

~~~bash
source /opt/ros/humble/setup.bash
cd /home/ym/ros2_ws
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

### 16.3 udev 固定设备名

分别读取 E110 USB-UART 和烧录 CDC 固件后的 C 型板属性：

~~~bash
udevadm info -a -n /dev/ttyUSB0 | rg 'idVendor|idProduct|serial'
udevadm info -a -n /dev/ttyACM0 | rg 'idVendor|idProduct|serial'
~~~

再按实际 VID、PID、序列号创建 /etc/udev/rules.d/99-kiwi-devices.rules：

~~~text
SUBSYSTEM=="tty", ATTRS{idVendor}=="<E110转换器VID>", ATTRS{idProduct}=="<E110转换器PID>", ATTRS{serial}=="<E110序列号>", SYMLINK+="pacecat_e110", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<C板CDC VID>", ATTRS{idProduct}=="<C板CDC PID>", ATTRS{serial}=="<C板序列号>", SYMLINK+="robomaster_c", GROUP="dialout", MODE="0660"
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

- 在 /home/ym/ros2_ws 建立 firmware 工程和持续集成。
- 确认三个电机/电调型号、CAN 协议、编码器单位、供电和急停。
- 写出 CAN/UART 协议、版本号、CRC 和故障码。
- 机械测量 L、r、gamma、底盘 footprint，并拍照记录坐标方向。
- 验收：急停断能、上电默认禁能、协议文档可供两端独立实现。

### 阶段 1：硬件和传感器单测（2–4 天）

- RoboMaster C 固件完成 CAN 电调控制、BMI088、ADC_BAT、看门狗、急停和反馈帧。
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
□ 型号为 LDS-E110-R，标称 7 Hz/4200 点每秒记录在配置
□ 230400 8N1、CF FA、长度和 checksum 已抓包确认
□ /scan 的 frame_id=lidar_link，角度方向在 RViz 正确
□ 10 分钟校验错误=0、帧率稳定、拔插可重连

底盘
□ C 型板 USB CDC/CAN 链路、协议版本和 CRC 已验证
□ CAN1 三个电调在线，CAN1/CAN2 终端电阻已实测
□ BMI088 安装方向、时间戳、温度和 covariance 已验证
□ 三个 wheel_joint 名称唯一，r/L/gamma 与实物记录一致
□ +x、+y、+wz 纯运动方向正确
□ 命令超时和 C 型板 watchdog 都能归零
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

1. **E110 驱动兼容性**：型号已确定为 LDS-E110-R，但通用 ROS 2 驱动没有专用启动文件；必须按本地 V1.5 手册抓包确认 CF FA、校验和角度排序。
2. **轮子正方向不一致**：运动学公式正确但电机符号错误，会表现为斜行、旋转串扰。
3. **时间戳和 TF 错误**：SLAM/AMCL 的“漂移”经常是延迟或 frame 配错，而不是算法参数。
4. **过度依赖软件安全层**：collision_monitor 或 Nav2 崩溃时，硬件急停和 C 型板 watchdog 仍需工作。
5. **C 型板能力误读**：它没有三相功率驱动和手册定义的专用正交编码器接口；电调反馈协议未确定前无法完成闭环。
6. **许可证和版本漂移**：参考雷达仓库许可证字段为 TODO；必须确认授权并固定 commit。
7. **Humble 依赖环境**：conda Python 覆盖系统 Python 会导致 rosidl_adapter/colcon 构建异常。

### 19.2 建议的第一周交付物

- LDS-E110-R 接线照片、原始串口抓包和 parser 单元测试；
- 底盘尺寸/符号表、RoboMaster C 协议 v1、急停测试记录；
- kiwi_description 能在 RViz 显示，TF 树无重复；
- 三轮控制器可完成 +x/+y/+wz，输出 /wheel/odom；
- 一份可复现的 Humble 安装和构建脚本；
- 第一张闭环地图及对应参数、rosbag 和验收结论。

## 20. 参考资料

以下链接是方案中实际采用的官方文档或开源仓库；阅读时请优先选择对应的 humble 分支和版本。

- 本地权威参数：/home/ym/ros2_ws/documents/LDS-E110-R-激光雷达使用说明书-V1.5.pdf
- 本地权威参数：/home/ym/ros2_ws/documents/RoboMaster开发板C型用户手册.pdf
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
file -bi /home/ym/ros2_ws/ros2_three_wheel_omni_navigation_plan.md
iconv -f UTF-8 -t UTF-8 /home/ym/ros2_ws/ros2_three_wheel_omni_navigation_plan.md >/dev/null
rg -n 'Ubuntu 22\\.04|ROS 2 Humble|LDS-E110-R|230400|CF FA|RoboMaster|BMI088|OmniMotionModel|ExecuteMission' \
  /home/ym/ros2_ws/ros2_three_wheel_omni_navigation_plan.md
rg -n '^~~~' /home/ym/ros2_ws/ros2_three_wheel_omni_navigation_plan.md | wc -l
git -C /home/ym/ros2_ws status --short --branch
git -C /home/ym/ros2_ws log --oneline -2
~~~

代码块数量必须为偶数；命令中的示例值仍需在接入实物时替换并经过测试记录。
