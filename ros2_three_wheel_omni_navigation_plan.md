# ROS 2 Humble + Ubuntu 22.04 三轮全向底盘建图导航实施方案

> 版本：2026-08-04
> Git 基线：`src` 子仓库 `bc45017`（kiwi_base 通信与反馈工具）；`standard-robot-pro` 仍为独立固件工作区，未纳入本次 ROS 提交。
> 目标平台：Ubuntu 22.04 LTS + ROS 2 Humble
> 激光雷达：Pacecat LDS-E110-R，依据本地 V1.5 手册
> 底盘控制器：DJI RoboMaster 开发板 C 型（STM32F407）
> 底盘：三轮全向（Kiwi drive）
> 文档定位：从电机/编码器、MCU、ROS 2 驱动，到建图、定位、导航和任务层的落地设计。
> 说明：这是从零搭建的独立工程方案。开源仓库只作为协议、接口和算法参考，不假定现有工程可以直接使用。

> 当前实现状态（2026-08-04）：已完成 `kiwi_base` 第一版底盘通信骨架和 MCU 最小改动链路。ROS 节点/键盘、上位机三轮逆运动学、主机限速/斜坡/软件急停、`0x33/len=12/id=0x3A` 三电机 float 命令编解码、轮里程计/IMU/电池/诊断发布、单调时钟安全超时和 USB 断线重连已实现；MCU CDC ring、`ros_bridge`、三路 M3508 速度 PID、连续编码器、100 ms watchdog、WHEEL/IMU 状态帧已实现。M3508/C620/轮子规格已录入，CAN ID、轮符号、速度单位实物确认、物理急停和电流换算仍待实机确认。ROS Humble 构建和无硬件 Python 自检通过；当前仍待 Keil/ARM 实际链接、烧录、USB 枚举、CAN 电机台架、EKF/SLAM/Nav2 联调。

## 1. 先给结论

推荐采用下面的基线：

1. 主机运行 Ubuntu 22.04 和 ROS 2 Humble，所有 Humble 软件包固定在同一发行版。
2. RoboMaster C 型板负责底盘实时控制，通过 CAN1 连接三个外置电调；该开发板不是电机功率驱动器。
3. 第一版让 C 型板通过 USB CDC 与主机通信；若实测抗干扰不足，再使用隔离 USB-CAN 接 CAN2。当前联调命令协议为 `0x33/len=12/id=0x3A/三个电机轴 float32/CRC16`；反馈帧和物理单位仍需抓包确认，watchdog 与硬件急停独立保留。
4. 不采用 `omnidirectional_controllers`，也不把标准控制器管理器放进第一版；新建单一 `kiwi_base_node`，负责三轮逆运动学、上/下位机协议、编码器/IMU解析和原始轮里程计。
5. 里程计由轮编码器计算，轮里程计和 IMU 用 robot_localization EKF 融合；EKF 唯一发布 odom 到 base_footprint 的 TF。
6. 建图使用 SLAM Toolbox；保存地图后，定位使用 AMCL，并明确使用 Humble 的 OmniMotionModel。
7. Nav2 第一阶段使用 DWBLocalPlanner，显式打开 Y 方向速度采样；后续可评估 Humble 的 MPPI Omni 模型。
8. 速度链路固定为：Nav2 → velocity_smoother → collision_monitor → `kiwi_base_node`（限速/斜坡/软件急停）→ C 型板 → 外置电调。硬件急停和通信看门狗独立于这条链路。
9. LDS-E110-R 直接接主机 USB-UART，并使用独立 5.1 V 电源；不要从 C 型板标注 100 mA 的接口给雷达供电，也不要把雷达 frame_id 写成 map。

最终验收不是“能启动几个 launch”，而是以下闭环都成立：

底盘闭环：Nav2/遥控速度 → `kiwi_base_node` → C 型板三轮闭环 → 编码器/IMU → EKF 输出 `odom→base_footprint`；建图模式再由 SLAM Toolbox 输出 `/map` 和 `map→odom`，回环后由 TF 组合出 `map→base_link`。导航模式关闭 SLAM Toolbox，改用 map_server + AMCL，再由 Nav2 发全向速度。

### 1.1 框架候选与最终组合

“换一个框架”不等于把 Nav2 全部替换掉。对本项目，最稳妥的做法是保留 Humble 的 Nav2 导航服务器，只替换底盘适配层，并按需要替换规划插件：

| 能力 | 首选 | 备选 | 本项目决策 |
|---|---|---|---|
| 导航编排 | Nav2 Humble | 自研导航状态机 | 保留 Nav2；任务层通过 BT/Simple Commander 调用 |
| 2D 建图 | SLAM Toolbox | RTAB-Map ROS2、Cartographer ROS2 | E110 纯 2D 激光首选 SLAM Toolbox；RTAB-Map 仅在增加相机/3D 激光后启用 |
| 已知地图定位 | Nav2 AMCL | SLAM Toolbox localization mode | 按用户要求使用 AMCL；两者不能同时发布 `map→odom` |
| 全局规划 | SmacPlanner2D | NavFn、SmacPlannerLattice | 首选 SmacPlanner2D；非圆形外壳或需要运动学可行路径时评估 Lattice |
| 局部控制 | DWB | MPPI Omni、RPP | 先 DWB 做可观测基线，稳定后切换 MPPI Omni 做性能对比 |
| 底盘接口 | 自研 `kiwi_base_node` | 标准硬件接口适配层 | 第一版用单节点降低通信和调试复杂度；未来需要统一硬件接口时再拆分 |
| 复杂任务 | Nav2 BT + Simple Commander | PlanSys2 | 当前航点/动作任务不引入 PlanSys2，任务复杂到需要 PDDL 时再评估 |

官方依据：

- [Nav2 Smac Planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html) 明确支持圆形全向机器人使用 `SmacPlanner2D`，并提供面向全向车的 Lattice 控制集。
- [Nav2 MPPI](https://docs.nav2.org/configuration/packages/configuring-mppic.html) 支持 `mppi::OmniMotionModel`，并提供 `vy` 与横向加速度约束。
- [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) 提供 `nav2_amcl::OmniMotionModel`。
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) 是当前维护的 ROS 2 2D SLAM 方案，并有 mapping/localization 两种模式。
- [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros) 支持 Humble，但主要价值在视觉、3D 激光和多传感器闭环；对当前 E110 单雷达方案属于更重的备选。

暂不选 `slam_gmapping`、`hector_slam` 作为 Humble 基线，也不把 Cartographer 作为第一阶段依赖：前者的 Humble 发布支持不足，后者配置和维护成本更高。若后续需要 3D/视觉建图，单独建立 RTAB-Map 实验分支，不改变 AMCL + Nav2 主链路。

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
| pacecat_lds_e110_driver | kiwi_base_node | diagnostics             |
| robot_state_publisher   | robot_localization/ekf | Nav2 |
+------------------------------- 传输层 -------------------------------+
| USB CDC（首版）或隔离 USB-CAN：命令、反馈、故障、预留急停状态        |
+------------------------------- MCU/执行层 ---------------------------+
| RoboMaster C 板：闭环、BMI088、ADC、电调通信、看门狗、急停 GPIO（待接入）|
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
- 203 mm 轮径和 322.56 mm 中心到轮心已按项目输入写入初始参数；轮符号、有效接触半径、设备名和速度限值仍需实物验证。

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

### 3.2 已提供的底盘规格与当前工程解释

| 部件/数据 | 当前记录值 | 用途与边界 |
|---|---:|---|
| M3508 减速电机套装额定电压 | 24 V | 电机/电调母线额定值，不等于 C 板 USB/逻辑电源 |
| M3508 空载转速 | 482 rpm | 配减速箱输出轴规格；不能再除以 19 |
| M3508 3 N·m 时最大转速 | 469 rpm | 配减速箱输出轴规格；需结合负载、轮胎和供电实测 |
| M3508 持续最大扭矩 | 3 N·m | 机械/热约束，不是可长期忽略的控制电流上限 |
| M3508 使用温度 | 0–50 °C | 需在整机封装后做温升验证 |
| 减速比 | 工程标称 19:1（资料机构比约 19.203，最终按轮速/编码器实测校正） | 主机将轮输出轴目标换算为电机轴目标，MCU 不重复相乘 |
| C620 信号 | CAN/PWM（二者不可同时接入） | 当前工程使用 CAN；具体电流帧仍需抓包确认 |
| C620 最大持续电流 | 20 A | 电调能力上限；电池、保险丝、线束和接插件需另行核算 |
| C620 线长 | 344±15 mm | 布线与应力释放参考 |
| C620 额定电压/重量/尺寸 | 24 V / 35 g / 49.4×25.8×11.5 mm | 机械与供电设计参考 |
| 轮直径/半径 | 203 mm / 0.1015 m | 轮速与 URDF 初始值 |
| 中心到轮心距离 | 322.56 mm / 0.32256 m | 本文暂按径向距离解释；若实际是三角形边长，应换算为 `边长/√3` |

482/469 rpm 是带减速箱的输出轴数据。按当前 `r=0.1015 m、L=0.32256 m`，输出轴空载/额定扭矩角速度约为 50.475/49.114 rad/s；ROS 和 MCU 当前把 30 rad/s 作为保守运行上限，不能将其误写成电机规格极限。

### 3.3 仍需在台架确认的数据

| 数据 | 用途 | 当前状态 |
|---|---|---|
| 三轮实际轮序、切向角和正方向 | Jacobian、URDF、编码器符号 | 待低速点动确认 |
| 三个电机 CAN ID、CAN 总线和终端电阻 | C 型板闭环 | 待抓包/实测 |
| C620 电流 raw 与安培换算、温度保护 | 电流限幅和诊断 | 待用电流计校准 |
| USB 命令帧格式、版本和校验 | `ros_bridge` 与主机 codec | 命令格式已按 0x33/0x3A/三 float 冻结；反馈帧仍待确认 |
| 编码器反馈单位、零点和掉线行为 | 轮位置/速度 | 待实测 |
| BMI088 相对底盘安装方向 | IMU frame 和 EKF | 待测量 |
| STM32F407 完整料号、主频和存储 | 固件资源预算 | 待从芯片丝印/固件工程确认 |
| 底盘外轮廓 | costmap footprint、碰撞监测 | 待测量 |
| 计算机型号与总线 | CPU 预算、设备名 | 待确认 |

没有这些台架数据时，可以先做软件自检和传感器单测，但不能声称运动学符号、电流安全或最终协议已经标定。

## 4. ROS 2 工作空间和包边界

工作空间固定为 /home/ym/ros2_ws，包名保持稳定，便于后续换电机或电调：

~~~text
ros2_ws/
└── src/
    ├── kiwi_description/          # URDF/Xacro、TF、碰撞模型、传感器安装位
    ├── kiwi_base/                 # 已实现：节点、键盘、三轮运动学、USB 协议、轮里程计
    ├── pacecat_lds_e110_driver/   # LDS-E110 生命周期节点、协议解析、LaserScan
    ├── kiwi_bringup/              # 硬件、传感器、状态发布、参数组合启动
    ├── kiwi_localization/         # EKF、AMCL 参数和定位启动
    ├── kiwi_slam/                 # SLAM Toolbox 参数和建图启动
    ├── kiwi_navigation/           # Nav2 参数、地图、速度和安全层启动
    ├── kiwi_mission_interfaces/   # ExecuteMission.action 等接口
    ├── kiwi_mission/              # 任务状态机、BasicNavigator 封装
    └── kiwi_tests/                # 目标：运动学、协议、TF、launch、回放测试
firmware/
└── robomaster_c_base/             # C 型板固件：CAN 电调、BMI088、协议和安全
~~~

每个包只拥有自己的参数和启动入口；不要把所有参数塞进一个巨大的 bringup 文件。建议用 git 子模块或 vendor 目录锁定外部参考代码，并在 README 记录上游 commit 和许可证。

`kiwi_base` 建议先按通信和运动学职责拆成可单测的内部模块，不再拆成控制器插件：

~~~text
kiwi_base/（当前已存在）
├── include/kiwi_base/{kiwi_kinematics,robomaster_protocol}.hpp
├── src/{kiwi_base_node,kiwi_kinematics,robomaster_protocol,kiwi_teleop_keyboard}.cpp
├── config/kiwi_base.yaml
└── test/{test_kinematics,test_protocol}.cpp
~~~

`kiwi_base_node` 的单元测试输入输出不依赖实际串口；硬件联调只替换 transport 层，便于在没有 C 型板时先完成运动学、协议和 EKF 数据链路验证。

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

- 建图运行时只让 SLAM Toolbox 发布 `map→odom`；定位运行时只让 AMCL 发布 `map→odom`。
- `kiwi_base_node` 只发布 `/wheel/odom` 话题，不发布 odom TF；EKF 唯一发布 `odom→base_footprint`。
- Nav2、AMCL、SLAM Toolbox 统一使用 base_footprint；base_link 只表示机体几何参考。
- lidar_link 的原点和旋转方向必须按安装实测填写，不能靠 RViz 中“看起来差不多”。
- SLAM Toolbox 的回环优化会更新 `map→odom`，不会直接发布 `map→base_link`；后者由 TF 自动组合 `map→odom→base_footprint→base_link` 得到。
- 启动后用 tf2_tools 检查每条链只有一个发布者，不能出现 map→base_link 直连和重复 odom TF。

### 5.2 核心话题

| 话题 | 类型 | 发布者 | 订阅者/用途 |
|---|---|---|---|
| /scan | sensor_msgs/msg/LaserScan | LDS-E110 驱动 | SLAM、AMCL、costmap、collision_monitor |
| /imu/data_raw | sensor_msgs/msg/Imu | `kiwi_base_node` BMI088 bridge | EKF |
| /battery_state | sensor_msgs/msg/BatteryState | `kiwi_base_node` ADC_BAT bridge | diagnostics、任务前置检查 |
| /wheel/odom | nav_msgs/msg/Odometry | `kiwi_base_node` | EKF、调试 |
| /odometry/filtered | nav_msgs/msg/Odometry | EKF | Nav2、调试 |
| /joint_states | sensor_msgs/msg/JointState | `kiwi_base_node` | robot_state_publisher |
| /cmd_vel | geometry_msgs/msg/Twist | Nav2/遥控 | velocity_smoother |
| /cmd_vel_smoothed | geometry_msgs/msg/Twist | velocity_smoother | collision_monitor |
| /cmd_vel_safe | geometry_msgs/msg/Twist | collision_monitor | `kiwi_base_node` |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | 各驱动 | 监控和任务前置检查 |

Humble 基线统一使用 geometry_msgs/msg/Twist（非 stamped）。`kiwi_base_node` 订阅 `/cmd_vel_safe`；若后续切换 TwistStamped，必须在 Nav2、安全层和底盘节点三端同时切换。

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

不要把某个开源仓库的三行公式当作通用运动学。自研控制器统一使用轮接触点和驱动切向方向生成 Jacobian。以第 i 个轮的轮心位置
`p_i=(x_i,y_i)`、驱动滚动方向单位向量 `t_i=(tix,tiy)`、轮半径 `r_i` 和编码器符号 `s_i` 表示：

~~~text
omega_i = s_i / r_i * [ tix*vx + tiy*vy
                         + (x_i*tiy - y_i*tix)*wz ]

omega = J * [vx, vy, wz]^T
twist = (J^T J)^-1 J^T * omega       # 3 轮满秩时的正向解
~~~

三轮 120° 对称布置只是该矩阵的一种特例。`kiwi_base_node` 在启动时检查矩阵满秩、轮半径为正、轮心不重合；缺参数或条件不满足时拒绝运行。正向运动学、逆向运动学和轮符号都必须用纯 `+x/+y/+wz` 测试验证。

当前 `src/kiwi_base` 为兼容已经冻结的 C 板联调量，实际发送采用固定的归一化矩阵：

~~~text
w1 = 3 * (-0.5*vx - 0.8660254*vy - wz)
w2 = 3 * ( vx              - wz)
w3 = 3 * (-0.5*vx + 0.8660254*vy - wz)
~~~

这里的 `3` 是联调标定比例，不是轮半径；轮半径、轮心距当前只用于几何有效性检查，
不参与上述固定映射的幅值计算。完成轮序、有效接触半径和低速运动标定后，才能切换为
上面的物理 Jacobian，并同步修改正解、测试和速度单位，避免只替换一侧公式。

### 7.2 `kiwi_base_node` 参数和职责

`kiwi_base_node` 当前负责四件事：

1. 将 `/cmd_vel_safe` 的机体速度通过当前固定 Kiwi 映射转成三个轮子的目标速度；逆解只在上位机执行，C 型板不重复逆解；
2. 按 `0x3A` 命令 codec 向 C 型板发送三个电机轴速度，并接收实际轮速与 IMU；反馈字段仍待协议抓包确认；
3. 从三个轮子的 position/velocity state 计算 `/wheel/odom` 和 `/joint_states`，不发布 odom TF；
4. 执行速度/加速度限幅、`/kiwi/estop` 软件急停、单调时钟命令超时、断线重连和 MCU 电机故障门控；当前 I/O 在单个 wall timer 内完成，没有独立 I/O 线程或 TX 队列。

参数骨架：

~~~yaml
kiwi_base_node:
  ros__parameters:
    transport: usb_cdc
    device: /dev/robomaster_c
    protocol_version: 51       # 0x33，命令联调版本字节
    command_timeout_ms: 100
    feedback_timeout_ms: 150
    reconnect_interval_ms: 1000
    publish_rate: 50.0
    max_wheel_speed_rad_s: 30.0
    max_linear_speed_m_s: 0.6
    max_lateral_speed_m_s: 0.6
    max_angular_speed_rad_s: 1.5
    max_accel_m_s2: 0.8
    max_decel_m_s2: 1.2
    max_yaw_accel_rad_s2: 2.0
    max_yaw_decel_rad_s2: 3.0
    estop_topic: /kiwi/estop
    wheel_names: [wheel_joint_1, wheel_joint_2, wheel_joint_3]
    # 几何参数已合并到 config/kiwi_base.yaml；同名参数可由自定义文件覆盖。
    odom_frame_id: odom
    base_frame_id: base_footprint
    imu_frame_id: imu_link
    cmd_vel_topic: /cmd_vel_safe
    wheel_odom_topic: /wheel/odom
    imu_topic: /imu/data_raw
    joint_states_topic: /joint_states
~~~

已覆盖：Jacobian 满秩/回环、协议 golden vector、半帧/粘包、CRC 错误、IMU 时间戳和 WHEEL 字段偏移。待补：命令超时/断线重连的节点级测试、rosbag odom 连续性、实机故障/急停测试。该节点不依赖 `omnidirectional_controllers` 或 `standard-robot-pro` 头文件。

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
- C620 的 CAN/PWM 只能选一种输入，切换前断电；C620 终端开关按总线拓扑只在两端开启，不能把三个电调都并联成终端。
- CAN1/CAN2 仍需按实际线束断电测量，再确认总线两端 120 Ω 终端配置。
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

手册只确认 USB 具备通信能力，不提供本项目需要的 CDC 应用协议；USB 描述符、收发缓冲和项目 CRC codec 均由 `ros_bridge`/ROS 适配层实现。USB 给板载 STM32/部分器件供电时，不能同时假定它给 PWM 外设电源供电。

### 8.3 主机与 C 型板协议（命令帧冻结格式）

命令帧按你提供的 C 型板解析流程实现；反馈帧布局仍按当前固件结构保留，后续抓包后可只替换反馈 codec，
不改变“上位机逆解、C 板执行速度闭环”的所有权：

~~~text
header = sof(0x33) | payload_len | msg_id | header_crc8
frame  = header | payload | crc16_le

CMD_MOTOR_SPEED(0x3A): payload 12 字节、整帧 18 字节：
motor_target_rad_s[0]:f32@0、motor_target_rad_s[1]:f32@4、
motor_target_rad_s[2]:f32@8。三个值是电机轴 rad/s，小端 IEEE-754 float32。

字节顺序：33 | 0c | 3a | 65 | motor1:f32 | motor2:f32 | motor3:f32 | crc16_le。
轮/电机映射：wheel_joint_1/2/3 -> payload[0/1/2] -> CAN ID 1/2/3。

STATE_WHEEL(0x80): payload 46 字节，state_seq、last_cmd_seq、mcu_time_ms、
position[3]、velocity[3]、current_raw[3]、battery_mV、fault_bits、flags。
STATE_IMU(0x81): payload 34 字节，seq:u16@0、mcu_time_ms:u32@2、
gyro[3]:f32@6、accel[3]:f32@18、temperature:f32@30。
~~~

命令帧没有 seq、timestamp、enable 或 clear_fault 字段。主机安全门控在发送前把禁能/超时目标写成三个零速值；
MCU 对每个 CRC、长度、有限值和量程合法的新帧刷新 100 ms watchdog。`STATE_WHEEL.last_cmd_seq` 是 MCU 本地接受计数，
不是线上回显的主机序号。

本版依据 `CAN_receive.c` 的 M3508 反馈和速度 PID，把三个 float 冻结为电机转子轴 `rad/s`。若后续双方改为 `rpm` 或减速箱输出轴速度，只修改 ROS/MCU 的单位转换层，不能在不升协议版本的情况下改变含义。

收到未知版本、ID、长度或 CRC 错误时必须丢弃并计数；命令速度量程按 `KIWI_MAX_MOTOR_RAD_S` 检查。

MCU 时间戳单位为 `ms`；当前 ROS 消息 header 使用接收时刻，尚未实现 MCU 到 ROS 的偏移估计，后续 EKF 联调前必须补齐。`STATE_POWER=0x82` 仅保留解码接口，当前 MCU 不发送。所有整数小端、float32 IEEE-754 小端，未知版本/长度/CRC 直接丢弃。

### 8.4 上位机与底盘安全保护

安全链路必须逐层失效安全，任何一层失效都不能让电机保持上一条有效速度：

| 层级 | 已实现/要求 | 触发后的动作 |
|---|---|---|
| 上位机输入 | 拒绝 NaN/Inf；`max_*_speed` 绝对限幅；`max_accel/max_decel` 斜坡 | 只允许受限速度进入逆解 |
| 上位机命令 | 100 ms `cmd_vel` 超时；`/kiwi/estop=true` | 立即清零并发送三个零速 float |
| USB/反馈 | 打开失败、拔线、部分写入、150 ms 反馈超时 | 清在线状态、停止发送有效使能帧并重连 |
| MCU 接收 | SOF/长度/ID/CRC/有限值/电机轴速度量程检查 | 非法帧不刷新 watchdog、不更新命令快照 |
| MCU 执行 | USB 未枚举、watchdog 100 ms、电机离线、故障位 | 三路目标归零、PID 清积分、CAN 输出 0 |
| 电流/功率 | C620 命令能力上限 20 A；当前 `KIWI_MAX_COMMAND_RAW=8192` 是约 10 A 命令电流刻度的起始 raw 限制，不等同电调输入电流或电机额定连续电流 | 需用电源侧电流计和温度实测后再调整；16384 仅保留为峰值上限，不默认启用 |
| 硬件层 | 物理急停 GPIO/电调使能/动力回路 | 必须独立切断执行器；尚未接入前项目验收不通过 |

`KIWI_C620_COMMAND_RAW_PER_AMP=819.2` 只描述 C620 CAN 命令映射；`STATE_WHEEL.motor_current_raw` 是 DJI 反馈原始码，未完成标定前禁止在 ROS 中当作安培或据此做热保护。软件急停不是功能安全急停，硬件急停必须在 ROS、USB 和 MCU 全部失效时仍有效。

### 8.5 频率与看门狗

- C 型板到电调的控制与反馈：工程目标 500 Hz–1 kHz，最终由电调 CAN 协议决定。
- 主机命令：50–100 Hz。
- 编码器反馈：50–200 Hz。
- ROS 控制器命令超时起点：100 ms。
- C 型板命令看门狗：当前固定为 100 ms 内没有有效新命令则三路目标归零并清 PID；零速帧不等于硬件断能。
- 物理急停输入尚未接入 C 型板 GPIO；当前只能保证上电禁能、通信 watchdog、电流 raw 限制和电机离线归零，不能把软件状态位当作硬件急停。
- 上电默认零输出；有效帧和三个电机在线后才运行 PID，协议没有线上 enable/clear_fault 位；物理急停另行处理。
- BMI088 读取建议 500–1000 Hz，向主机发布 100–200 Hz；这些是工程目标，不是手册保证值。

### 8.6 ROS 2 底盘控制节点接口

第一版只实现一个 `kiwi_base_node`，不引入控制器管理器。当前实现是单节点 wall timer 内的非阻塞 I/O：

~~~text
cmd_vel subscriber → Kiwi inverse kinematics → nonblocking write → USB CDC
                                              ↑                    ↓
wheel odom publisher ← encoder state ← framed RX parser ← C 型板 feedback
imu publisher        ← BMI088 state   ← framed RX parser
diagnostics          ← heartbeat/fault/timeout/estop
~~~

节点接口固定为：

- 订阅 `/cmd_vel_safe`（`geometry_msgs/msg/Twist`），只接受机体 `vx/vy/wz`；
- 发布 `/wheel/odom`（`nav_msgs/msg/Odometry`）、`/joint_states`、`/imu/data_raw`、`/battery_state` 和 `/diagnostics`；
- 不发布 `odom→base_footprint` TF，由 `robot_localization` EKF 独占；
- USB I/O 使用 wall timer、非阻塞读写和 1 s 断线重连；USB 未配置时 MCU 下一个 2 ms 周期立即停机，重连会清除旧命令会话；当前没有独立线程、socketcan fallback 或 TX 队列；
- 收到 CRC 正确且版本匹配的反馈才更新时间戳和在线状态，命令超时立即发送三个零速电机值。
- `usb_cdc` 传输使用 Linux `termios` 读取 `/dev/robomaster_c`；其他 transport 当前拒绝启动。

后续若需要接入仿真、控制器管理器或多种底盘，只把上述传输和运动学模块拆成标准硬件适配层，不改变协议和话题契约。

### 8.7 `standard-robot-pro` 审计结论

审计对象：`/home/ym/ros2_ws/standard-robot-pro`。结论是“复用底层驱动，重写上层接口与三轮控制”，不是直接接入 ROS 2，也不是把整个仓库推倒重写。

#### 可以保留的底层能力

- `application/robot_cmd/CAN_receive.c`、`CAN_cmd_dji.c`、`motor.h`：DJI 电调 CAN 反馈、当前值发送、通用电机状态结构和 ID/CAN 选择。
- `application/robot_cmd` 下的 PID、故障和电机离线检测骨架；但必须重新确认电机轴/减速输出轴的单位和方向。
- BMI088 SPI 读取、ADC_BAT、电源和急停 GPIO、FreeRTOS 周期任务、CRC8/CRC16 工具。
- `bsp_can.c`、串口 DMA/IDLE 的硬件初始化思路；主机协议仍需单独定义，不能沿用板间竞赛协议。

#### 必须隔离或重写的部分

1. 这是 STM32F407/HAL/FreeRTOS/Keil 固件，本身不是 ROS 2 包；`standard-robot-pro` 没有供 `colcon` 构建的 `package.xml`、ROS 节点或 launch 文件。ROS 侧已在工作区新增 `src/kiwi_base`，通过 USB CDC 连接该固件。
2. `application/typedef_robot/robot_param.h` 已按 `ROBOT_CONTROL_KIWI` 选择 `robot_param_kiwi.h`；未定义宏时仍回到竞赛 profile。M3508/C620/轮径规格宏已记录，CAN ID、方向、有效减速比、轮径和限流策略仍是台架起点。
3. `application/chassis/chassis_omni.h:43-70` 和 `chassis_omni.c:65-79,97-105,253-261` 是四电机数组、初始化、观测和 CAN 发送；`chassis_omni.c:230-232` 只计算 `set[0..2]`，随后 `:234-242` 仍处理未赋值的 `set[3]`。这对三轮 Kiwi 是阻断性问题。
4. 原 `chassis_omni.c:144-157` 的目标量来自 DT7 遥控器，面向上位机的 `GetBoardInfo` 分支被注释；`:212-218` 的离线保命也被注释。Kiwi 路径已新增 ROS 命令接收、本地接受计数、超时、急停和上电禁能，旧 DT7 路径仍不能与其并行写 CAN。
5. `application/typedef/usb_typdef.h` 已新增 18 字节 `ReceiveDataKiwiMotorSpeed_s` 作为帧结构说明；旧 `ReceiveDataRobotCmd_s` 的 pitch/yaw/fire/vx/vy 仅供非 Kiwi 竞赛 USB 任务使用，不能承载底盘 `cmd_vel`。
6. USB 接收路径已改为 CDC 回调写 ring buffer、任务解析完整帧，仅对 CRC 正确、版本/ID/长度匹配、有限值和量程合法的命令刷新 watchdog；旧 `usb_task` 在 Kiwi 宏下不创建。

#### 源码中的硬编码只能作为待核对参考

现有 `robot_param_omni_infantry.h`、`robot_param_omni_mini.h`、`robot_param_omni_sentry.h` 等示例包含 `WHEEL_RADIUS=0.08/0.15`、`WHEEL_CENTER_DISTANCE=0.245/0.295`、4 个 M3508 ID、减速比 19 和方向符号。这些是四轮竞赛底盘示例，且部分注释把半径写成“直径”；在实物轮半径、轮心坐标、切向方向、减速比和编码器单位确认前，不得复制到 YAML 或固件。

#### 建议的抽取边界

~~~text
standard-robot-pro lower layer
  CAN/DJI + PID + BMI088 + ADC + CRC + FreeRTOS
                         │
                         ▼
new firmware/kiwi_base
  3 motors + Kiwi Jacobian (or host wheel-speed mode)
  USB CDC ring buffer + framed protocol + watchdog + estop
                         │ USB CDC / isolated CAN
                         ▼
ROS 2 kiwi_base_node → /wheel/odom + /imu/data_raw → EKF → Nav2
~~~

第一版由 ROS 主机完成 Kiwi 逆运动学，发送三个电机转子轴目标角速度；C 型板只做三个电机的速度闭环、电流输出和反馈。这样可以在 ROS 侧单元测试矩阵，并让 MCU 协议保持稳定。若以后需要脱离 ROS 的遥控运行，再在固件中增加同一矩阵的可选模式。

下位机改造建议在现有工程中新增独立目录，避免污染竞赛模块：

~~~text
standard-robot-pro/application/kiwi_base/kiwi_base.c/.h
standard-robot-pro/application/communication/ros_bridge.c/.h
  ├─ CDC/UART ring buffer 与帧解析
  ├─ 三轮目标角速度仲裁、watchdog、estop
  └─ CAN 电调反馈、BMI088、ADC_BAT 状态打包
~~~

该审计已转为实现记录：`standard-robot-pro` 已新增/修改 `ros_bridge.c`、`kiwi_base.c`、CDC ring、Kiwi 参数和 FreeRTOS 入口。当前 Keil 只有 `StandardRobotPro` target，通过 `ROBOT_CONTROL_KIWI=1` 启用；ARM/Keil 实际链接和烧录仍待执行。

## 9. URDF/Xacro、关节和机器人状态

### 9.1 建议的 Xacro 结构

~~~text
kiwi_description/
├── urdf/
│   ├── kiwi.urdf.xacro
│   ├── materials.xacro
│   └── sensors.xacro
└── rviz/
    └── model.rviz
~~~

kiwi.urdf.xacro 至少包含：

- base_footprint（地面投影，不参与惯量计算）；
- base_link（机体几何中心）；
- wheel_joint_1/2/3 和对应 wheel_link；
- lidar_link，原点为雷达旋转中心；
- imu_link，方向与 IMU 驱动输出的 frame_id 一致；
- 惯量、碰撞几何和三轮关节的 axis/origin；`kiwi_base_node` 负责发布 joint state，不在 URDF 中嵌入控制器标签。

轮关节的 axis、origin 和 rotation 必须与电机正转测试一致。连续旋转轮关节使用 continuous；编码器零点在 MCU 或 `kiwi_base_node` 中处理，不能在 URDF 中伪造。

### 9.2 底盘节点配置骨架

~~~yaml
kiwi_base_node:
  ros__parameters:
    transport: usb_cdc
    device: /dev/robomaster_c
    protocol_version: 51       # 0x33，命令联调版本字节
    command_timeout_ms: 100
    feedback_timeout_ms: 150
    reconnect_interval_ms: 1000
    publish_rate: 50.0
    max_wheel_speed_rad_s: 30.0
    max_linear_speed_m_s: 0.6
    max_lateral_speed_m_s: 0.6
    max_angular_speed_rad_s: 1.5
    max_accel_m_s2: 0.8
    max_decel_m_s2: 1.2
    max_yaw_accel_rad_s2: 2.0
    max_yaw_decel_rad_s2: 3.0
    estop_topic: /kiwi/estop
    wheel_names: [wheel_joint_1, wheel_joint_2, wheel_joint_3]
    # 运行参数和几何参数统一放在 config/kiwi_base.yaml，正常只加载一个文件。
    odom_frame_id: odom
    base_frame_id: base_footprint
    imu_frame_id: imu_link
    cmd_vel_topic: /cmd_vel_safe
    wheel_odom_topic: /wheel/odom
    imu_topic: /imu/data_raw
~~~

启动前用 `ros2 param dump /kiwi_base_node` 核对参数，节点必须在串口/CAN不可用或运动学参数缺失时明确报错并保持禁能。

运行参数、几何参数和安全限值统一位于 `src/kiwi_base/config/kiwi_base.yaml`；节点源码还提供同样的默认值，launch 默认通过包安装路径读取该文件。`kiwi_kinematics.yaml` 仅作历史兼容覆盖文件保留。MCU 侧的电机 ID、CAN 和电流 raw 限制位于 `standard-robot-pro/application/typedef_robot/robot_param_kiwi.h`；减速比为跨 ROS/Keil 的联调参数，必须保持两处一致，不能只改一端。

### 9.3 ROS 包函数职责索引

源码中的函数都带有中文注释，阅读顺序建议如下：

| 文件 | 函数 | 责任与安全边界 |
|---|---|---|
| `src/kiwi_base/src/kiwi_base_node.cpp` | `KiwiBaseNode::KiwiBaseNode` | 读取参数、检查 Jacobian、建立话题和定时器；配置错误保持禁能 |
| 同上 | `open_serial` / `close_serial` / `read_serial` | USB CDC raw 非阻塞 I/O、断线重连、清空半帧和在线状态 |
| 同上 | `limit_command` / `update` | 速度/加速度限幅、命令/反馈 watchdog、软件急停、上位机逆解和发送门控 |
| 同上 | `send_command` / `on_frame` | 发送三电机轴速度命令；接收实际轮速、IMU、电池和故障状态 |
| 同上 | `publish_odom` / `publish_joints` / `publish_imu` / `publish_battery` | 发布原始轮里程计、关节、IMU 和电池话题；不发布 odom TF |
| 同上 | `publish_diagnostics` | 汇总配置、USB、反馈、MCU、急停和 CRC 计数 |
| `src/kiwi_base/src/kiwi_teleop_keyboard.cpp` | `tick` / `publish_zero` | 键盘速度、250 ms 死手超时和退出安全停车 |
| `src/kiwi_base/src/kiwi_kinematics.cpp` | `inverse` / `forward` | 主机 Twist↔三个轮输出轴角速度；不访问硬件 |
| `src/kiwi_base/src/robomaster_protocol.cpp` | `encode_command` / `FrameParser` / `decode_*` | 0x3A 三 float 命令与反馈 codec；协议变化时替换此层 |

Python 台架工具为 `src/kiwi_base/tools/kiwi_serial_test.py`：`--self-test` 不接硬件，`--dry-run` 打印电机轴速度和 18 字节帧，`--enable` 才允许发送非零值，`--raw-hex` 用于反馈抓包/发送。

键盘联调时，`kiwi_teleop_keyboard` 只发布 `/cmd_vel_safe`，`kiwi_base_node` 是唯一 USB 所有者并负责逆解、减速比转换、安全门控和 0x3A 帧发送。两个终端的完整命令见 `src/kiwi_base/config/README.md`，不要同时运行 Python 直连脚本和 `kiwi_base_node` 抢占同一 CDC 设备。

## 10. 里程计、IMU 和 EKF

### 10.1 数据流

~~~text
C 型板 CAN 电调反馈 → kiwi_base_node → /wheel/odom
C 型板 BMI088 ─────→ kiwi_base_node ───→ /imu/data_raw
/wheel/odom + /imu/data_raw → robot_localization/ekf → /odometry/filtered
                                                          └→ odom→base_footprint TF
~~~

轮里程计保留原始值，便于诊断；Nav2 使用 /odometry/filtered。EKF 参数示例：

EKF 只解决局部连续运动估计，不生成 `map`。建图时 SLAM Toolbox 读取 `/scan` 和 `odom→base_footprint`，发布 `/map`（OccupancyGrid）及 `map→odom`；回环检测后更新 `map→odom`，通过 TF 链自动得到 `map→base_link`。保存地图后关闭 SLAM Toolbox，导航模式改由 map_server + AMCL 发布 `map→odom`。

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

当前实现限制和检查点：

- 当前 `kiwi_base_node` 暂用 USB 接收时刻作为 ROS header 时间戳，未做 MCU 时间偏移估计；接入 EKF 前必须用实测延迟和 MCU 时间戳修正。
- orientation 未计算时将 orientation_covariance[0] 设为 -1，不能发布单位四元数冒充有效航向。
- wheel odom 已填保守的非零协方差；IMU orientation 未计算并将 `orientation_covariance[0]` 设为 -1，具体噪声仍需实测标定。
- wheel odom 和 IMU 的 frame_id、child_frame_id 与 TF 拓扑一致。
- /odometry/filtered 在机器人静止时速度应接近零，转向时不能出现持续漂移。

## 11. 建图与回环：SLAM Toolbox

### 11.1 建图模式

建图启动顺序：

1. 启动 robot_state_publisher、硬件、雷达、IMU 和 EKF。
2. 确认 odom→base_footprint 和 base_footprint→lidar_link 连续可查。
3. 启动 SLAM Toolbox synchronous 或 asynchronous 节点。
4. 低速遥控机器人覆盖环境，先闭环再保存地图。
5. 用 map_saver_cli 保存 .yaml 与 .pgm；记录分辨率、原点和版本。

建图时的变换和话题关系如下：

~~~text
kiwi_base_node + EKF  ──发布──> odom→base_footprint
SLAM Toolbox      ──发布──> /map + map→odom
TF2 自动组合      ──得到──> map→odom→base_footprint→base_link
回环检测          ──修正──> map→odom（不改写 odom→base_footprint）
~~~

因此，SLAM Toolbox 不负责发布 `odom→base_link`，也不应与 EKF 争抢这条 TF；它通过里程计和激光匹配建立/优化地图坐标系。需要查看机器人在地图中的位姿时，查询 `map` 到 `base_link` 的 TF 即可。

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

建图期间不要同时启动 AMCL。保存地图前让机器人停止，等待最后一帧 scan、回环优化和 map 更新完成；确认 `/map` 已稳定后再执行 `map_saver_cli`。

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

### 13.2 全局 Planner 与 costmap

三轮 Kiwi 能够任意方向平移，第一阶段全局规划器使用 `SmacPlanner2D`。它对圆形全向机器人提供代价感知的 2D A*；如果实测底盘外形不是近似圆形且需要考虑运动学可行性，再评估 `SmacPlannerLattice`。`NavFn` 保留为依赖缺失或调试时的回退方案。

~~~yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5
      allow_unknown: true
      downsample_costmap: false
      cost_travel_multiplier: 2.0

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
kiwi_bringup/launch/hardware.launch.py       # kiwi_base_node、传感器、joint_state
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

第一版不安装 `ros-humble-ros2-control`/`ros-humble-ros2-controllers` 作为运行依赖；`kiwi_base_node` 直接发布 joint states 和轮里程计。未来若需要接入 controller manager，再单独增加适配包并回归验证。

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

### 阶段 0：命令接口和安全基线（代码已完成，反馈协议/实物待验）

- 已在 `/home/ym/ros2_ws` 保留 ROS 包和固件嵌套仓库；Keil 当前通过 `ROBOT_CONTROL_KIWI=1` 选择 Kiwi profile。
- 命令 codec 已按 `0x33/len=12/id=0x3A/三个电机轴 float32/CRC16` 对齐；反馈帧仍待协议评审和实物抓包确认。
- M3508/C620、203 mm 轮径和 322.56 mm 中心到轮心初值已录入；仍需确认 L 的语义、轮序/符号、CAN ID、有效接触半径和电流反馈单位。
- 主机限速/斜坡/软件急停、MCU watchdog/电流 raw 限制代码已具备；物理急停断能尚未验收，未接入前不能宣称安全完成。

### 阶段 1：硬件和传感器单测（代码骨架已完成，台架待验）

- RoboMaster C 固件已接入 CAN 电调控制、BMI088 快照、ADC_BAT、watchdog 和 WHEEL/IMU 反馈帧；物理急停尚未接入。
- USB CDC 回调已写入环形缓冲区，旧 USB 任务在 Kiwi 模式下不创建；ROS 侧 golden vector、半帧/粘包/CRC/WHEEL 偏移测试已通过。
- E110 接主机，验证 230400 8N1、帧头、校验、角度方向和帧率。
- IMU 发布正确 frame_id、时间戳和 covariance。
- 验收：雷达连续运行、USB 枚举、CAN 电机、急停和拔线安全停机均待实物测试。

### 阶段 2：底盘控制节点和运动学（ROS 代码已完成，实物标定待验）

- 已完成 `kiwi_base_node` 的 USB CDC 0x3A 三 float 命令收发、上位机三轮逆运动学、安全门控、joint states、轮里程计、IMU/电池/诊断发布和断线重连代码；不引入第三方全向控制器。
- CRC/半帧/粘包/WHEEL 偏移测试已通过；节点级超时/重连测试和实物 +x/+y/+wz 标定待补。
- 完成 +x、+y、+wz 三组纯运动标定，保存参数版本。
- 验收：无重复 TF；轮速/机体速度限幅、斜坡、超时归零和软件急停有效；物理急停、横向串扰和电流温升达到目标。

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
- 全局规划先用 SmacPlanner2D，局部控制先 DWB，再 velocity_smoother 和 collision_monitor；最后才评估 MPPI Omni。
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
□ C 型板 USB CDC/CAN 链路、USB 枚举和 0x3A 三 float 命令已完成实物验证（反馈帧仍待确认）
□ CAN1 三个电调在线，CAN1/CAN2 终端电阻已实测
□ BMI088 安装方向、时间戳、温度和 covariance 已完成实物验证（当前温度帧为 0、ROS 用接收时刻）
□ 三个 wheel_joint 名称唯一，r/L/gamma 与实物记录一致
□ +x、+y、+wz 纯运动方向正确
□ 主机限速/斜坡、命令超时、软件急停和 C 型板 watchdog 都能归零（代码已实现，待台架）
□ 物理急停不依赖 ROS（尚未接入，当前不通过）
□ C620 命令电流 raw/A、M3508 温升和保险/线束容量完成实测（当前不通过）
□ 运行时不依赖四轮第三方全向控制器

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
5. **C 型板能力误读**：它没有三相功率驱动和手册定义的专用正交编码器接口；当前代码按 DJI 电调 CAN 反馈闭环，电机/电调型号和 CAN 接线仍需实物确认。
6. **许可证和版本漂移**：参考雷达仓库许可证字段为 TODO；必须确认授权并固定 commit。
7. **Humble 依赖环境**：conda Python 覆盖系统 Python 会导致 rosidl_adapter/colcon 构建异常。
8. **底盘源码边界**：`standard-robot-pro` 的四轮竞赛配置和 USB 演示协议若未隔离，会把错误的轮数、单位和安全策略带入 ROS 链路。
9. **反馈时序**：ROS 当前按反馈到达时刻更新在线状态和轮里程计，尚未基于 `state_seq/mcu_time_ms` 去重或校正批量积压帧；接入 EKF 前需补节点级时序测试与时间偏移处理。

### 19.2 建议的第一周交付物

- LDS-E110-R 接线照片、原始串口抓包和 parser 单元测试；
- 底盘尺寸/符号表、命令 codec 与反馈协议评审记录、急停测试记录；
- kiwi_description 能在 RViz 显示，TF 树无重复；
- 三轮控制器可完成 +x/+y/+wz，输出 /wheel/odom；
- 一份可复现的 Humble 安装和构建脚本；
- 第一张闭环地图及对应参数、rosbag 和验收结论。

## 20. 参考资料

以下链接是方案中实际采用的官方文档或开源仓库；阅读时请优先选择对应的 humble 分支和版本。

- 本地权威参数：/home/ym/ros2_ws/documents/LDS-E110-R-激光雷达使用说明书-V1.5.pdf
- 本地权威参数：/home/ym/ros2_ws/documents/RoboMaster开发板C型用户手册.pdf
- DJI RoboMaster M3508 产品页（含 C620 规格入口）：https://www.robomaster.com/en-US/products/components/general/M3508
- DJI M3508 P19 Brushless DC Gear Motor V1.0：https://rm-static.djicdn.com/tem/17348/RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor%20V1.0.pdf
- C620 User Guide：https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.0.pdf（重点核对 CAN 命令电流范围、终端电阻和热保护）
- 项目输入参数：本轮用户提供的 203 mm 轮径、322.56 mm 中心到轮心距离；不是 DJI 雷达或开发板手册参数，需按 CAD/实测确认。
- Pacecat LDS-E110 系列官方规格：https://www.pacecat.net/product/lds-e110-series-lidar-sensor/
- Pacecat 中文规格页：https://www.pacecat.com/product/lds-e110-series-lidar-sensor/
- Pacecat LDS-E110-R V1.5 使用说明书（接线与串口协议）：https://www.pacecat.com/shared-files/940/?LDS-E110-R-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6-V1.5.pdf=
- Pacecat 官方下载页：https://www.pacecat.net/downloads/
- BlueSea ROS 2 雷达驱动参考：https://github.com/BlueSeaLidar/bluesea-ros2
- ROS 2 Humble Ubuntu 安装：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Nav2 Humble 默认参数：https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml
- Nav2 Smac Planner（2D/Hybrid/Lattice）：https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- Nav2 AMCL 配置：https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Nav2 MPPI Omni 配置：https://docs.nav2.org/configuration/packages/configuring-mppic.html
- Nav2 Humble MPPI（Omni）：https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller
- Nav2 Humble collision monitor：https://github.com/ros-navigation/navigation2/tree/humble/nav2_collision_monitor
- collision monitor 参数样例：https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/params/collision_monitor_params.yaml
- SLAM Toolbox Humble：https://github.com/SteveMacenski/slam_toolbox/tree/humble
- RTAB-Map ROS2（Humble 备选）：https://github.com/introlab/rtabmap_ros/tree/ros2
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
