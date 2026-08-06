# A 路线：机器人平台与状态估计

> 责任目标：向 B 稳定交付 `/scan`、`/odometry/filtered`、
> `odom→base_footprint` 和标准坐标的底盘命令入口。
> 对接契约：[接口契约.md](接口契约.md)

## 1. 完成定义

A 的任务不是“话题能看到数据”，而是让上位机连续、可诊断地知道机器人如何运动，并在
输入或反馈异常时停止底盘。

```text
输入：C 板电机/IMU 反馈、LDS-E110-R、/cmd_vel_safe、/software_estop
输出：/scan、/wheel/odom_raw、/imu/data_raw、/odometry/filtered、
      odom→base_footprint、机器人静态 TF、/diagnostics
```

## 2. 建议代码边界

```text
现有 src/kiwi_base/              USB 协议、三轮运动学、轮里程计、底盘安全入口
现有 src/bluesea-ros2/           雷达驱动；只在项目参数或明确缺陷处做最小修改
新增 src/kiwi_description/       URDF/Xacro、静态 TF、碰撞/可视模型
新增 src/kiwi_state_estimation/  robot_localization 参数、状态估计 launch、测试
新增 src/kiwi_platform_bringup/  底盘、雷达、description、EKF 的组合启动
```

A 不修改 SLAM、AMCL、Nav2 controller 或 costmap 参数。B 若需要传感器安装位或外形变化，
通过接口评审给出数值，A 更新 URDF/Xacro。

## 3. A0：先恢复可信运动学基线

这是所有里程计工作的前置门禁。

1. 选定最终 ROS 坐标为 `x前、y左、wz逆时针`。
2. 低速架空点动三轮，记录电机 ID、轮序、正方向和反馈单位。
3. 同步修改 `inverse()`、`forward()`、Python 台架工具和测试，不能只切换三行逆解。
4. 如果保留固定公式，明确它输出的是物理轮轴 `rad/s` 还是联调比例量。
5. 正式里程计必须引入轮半径和轮心距，保持量纲正确。
6. 确认 C 板反馈是电机轴 rpm 还是 rad/s；转换后 ROS 内只保留 SI 单位。

验收：

```text
纯 +x：里程计 vx>0，|vy| 和 |wz| 接近 0
纯 +y：里程计 vy>0，|vx| 和 |wz| 接近 0
纯 +wz：里程计 wz>0，|vx| 和 |vy| 接近 0
正逆解：随机有限输入 round-trip 测试通过
```

当前 `3882b32` 只切换了逆解轮序，正解与测试仍对应上一套轮序；修复前不得把
`/wheel/odom` 交给 EKF。

## 4. A1：底盘、雷达和原始状态话题

### 4.1 底盘输出

把 `kiwi_base` 参数冻结为：

```yaml
kiwi_base_node:
  ros__parameters:
    cmd_vel_topic: /cmd_vel_safe
    wheel_odom_topic: /wheel/odom_raw
    imu_topic: /imu/data_raw
    joint_states_topic: /joint_states
    odom_frame_id: odom
    base_frame_id: base_footprint
    imu_frame_id: imu_link
```

`/wheel/odom_raw` 要求：

- `header.frame_id=odom`、`child_frame_id=base_footprint`。
- `twist` 使用标准坐标和 SI 单位，协方差不得全零。
- 原始轮里程计不发布 TF，避免与 EKF 竞争。
- 反馈超时后停止更新有效里程计，并在 diagnostics 标记 ERROR。

### 4.2 雷达输出

当前雷达 YAML 的 `frame_id: map` 必须改为 `lidar_link`。A 需要核对：

- `/scan` 约 7 Hz，角度方向和 `lidar_link` 一致。
- `range_min/range_max` 与 LDS-E110-R 实测一致。
- `scan_time`、`time_increment` 和时间戳不会跳变。
- 串口拔插后能重连，校验错误和最后一帧年龄进入 `/diagnostics`。

### 4.3 IMU 输出

- `frame_id=imu_link`，轴方向由 URDF 固定变换表达，不在 EKF 参数里猜轴。
- gyro 为 rad/s，accel 为 m/s²，协方差来自静态采样估计。
- 若 orientation 来自 C 板欧拉角，必须确认角度定义和安装方向；未验证时标为无效。
- 第一阶段 EKF 只融合轮速 `vx/vy` 和 IMU `wz`，不急于融合室内磁航向。

## 5. A2：URDF/Xacro 与 TF

最低目录：

```text
kiwi_description/
├── urdf/kiwi.urdf.xacro
├── urdf/materials.xacro
├── urdf/sensors.xacro
├── rviz/model.rviz
└── test/test_urdf.py
```

最低 link/joint：

```text
base_footprint -> base_link
base_link -> lidar_link
base_link -> imu_link
base_link -> wheel_link_1/2/3（通过 wheel_joint_1/2/3）
```

验收：

1. `check_urdf` 通过，惯量和关节轴不为占位值。
2. `robot_state_publisher` 是所有静态机器人 TF 的唯一来源。
3. `view_frames` 中没有 `map→base_link` 直连、断链或重复发布者。
4. RViz 中雷达点与机器人外形、前后左右方向一致。

## 6. A3：robot_localization EKF

当前系统未安装 `robot_localization`，先安装对应 Humble 包，再新增
`kiwi_state_estimation/config/ekf_odom.yaml`。

第一阶段融合边界：

```text
/wheel/odom_raw：vx、vy，可选 wz；不重复融合由同一轮速积分得到的 pose 和 twist
/imu/data_raw：wz；绝对 yaw 通过标定前不融合
EKF 输出：/odometry/filtered + odom→base_footprint
world_frame：odom
two_d_mode：true
frequency：50 Hz
```

参数原则：

- 只选择每个独立信息源真正测得的维度，避免重复计算导致滤波器过度自信。
- 所有输入必须有合理协方差；全零协方差不进入交付版本。
- EKF 是 `odom→base_footprint` 唯一发布者；`kiwi_base_node` 保持只发消息。
- 静止测试、纯轴运动和长时间运行分别记录残差，不以 RViz“看起来平滑”作为验收。

## 7. A4：命令入口与安全

A 必须把 `/cmd_vel_safe` 当作标准 REP-103 Twist，在 `kiwi_base` 内完成硬件轮序/换轴。

进入 C 板前至少执行：

1. 拒绝 NaN/Inf 和超范围值。
2. 100 ms 命令超时归零。
3. `/software_estop=true` 立即归零并保持停止。
4. USB 断线、反馈超时或任一电机离线时归零。
5. 退出节点和重新连接时先发送零速，不能恢复旧命令。
6. MCU watchdog 和物理急停继续独立工作。

建议 diagnostics 至少包含：

```text
底盘 USB 在线状态、最后命令年龄、最后反馈年龄、CRC 错误计数
三个电机在线状态、反馈单位版本、软件急停状态、MCU watchdog 状态
雷达串口状态、scan 频率、最后 scan 年龄、校验错误计数
IMU 频率、时间戳跳变、gyro/accel 非有限值计数
```

## 8. A5：时间戳和协议升级路线

当前只积分瞬时电机速度，USB 延迟和丢帧会直接进入位置误差。阶段划分：

```text
阶段 1：沿用 0x3B，记录主机接收时间、频率、抖动和丢帧现象
阶段 2：新增协议版本/ID，加入 feedback_seq、mcu_sample_time、累计轮角或编码器计数
阶段 3：主机估计 MCU/ROS 时间偏移，按采样时刻发布，不按到达顺序盲目积分
```

不能改变现有 `0x3B/48` 的字段含义后仍宣称协议兼容。

## 9. A 的测试矩阵

```text
类别        用例                                      通过条件
单元测试    正逆运动学、轮序、符号、单位              全部通过
协议测试    CRC、半帧、粘包、非有限值、反馈超时        全部通过
台架测试    单轮低速、三轮 +x/+y/+wz                  方向和比例正确
TF 测试     静态树、EKF 动态 TF、唯一发布者            无断链/重复
时间测试    scan/IMU/轮反馈频率与时间戳                 无倒退，抖动有记录
安全测试    命令断流、USB 拔线、软件急停、电机离线      规定时间内零输出
实车测试    矩形、横移、360° 原地旋转                  方向正确，误差有量化
```

建议验证命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kiwi_base kiwi_description kiwi_state_estimation kiwi_platform_bringup
colcon test --packages-select \
  kiwi_base kiwi_description kiwi_state_estimation kiwi_platform_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
ros2 run tf2_tools view_frames
ros2 topic hz /scan /wheel/odom_raw /imu/data_raw /odometry/filtered
```

## 10. A 交付给 B 的接口包

每次交付目录或工单至少包含：

```text
commit.txt                  根仓库 commit
parameters/                实际运行 YAML
tf/view_frames.pdf         TF 图
bags/platform_<date>/      原始与滤波话题 rosbag
reports/topic_hz.txt       频率结果
reports/calibration.md     轮序、单位、轮径、IMU 安装方向
reports/safety.md          超时、拔线、急停结果
```

交付 rosbag 至少包含一次前进、左移、逆时针和静止段。B 可以依靠该 bag 完成 SLAM/Nav2
回放，不需要等待实车。

## 11. A 不负责的内容

- 不调 SLAM 回环、AMCL 粒子或 Nav2 critic 权重。
- 不为了让地图“看起来正常”而修改 `map→odom`。
- 不在底盘节点中实现全局/局部规划或障碍物逻辑。
- 不把 collision_monitor 当作物理急停。
