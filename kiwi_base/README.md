# kiwi_base

ROS 2 Humble 三轮 Kiwi 全向底盘驱动。上位机负责 `Twist` 安全门控、三轮逆运动学和 USB CDC 通信；RoboMaster C 型板负责三路 M3508 实时速度闭环、编码器与 IMU 反馈。

协议交接文档：[USB_CDC_PROTOCOL.md](USB_CDC_PROTOCOL.md)

参数说明：[config/README.md](config/README.md)

## 运动学与电机控制链路

### 1. 上位机逆运动学

`kiwi_base_node` 只接收机体速度，不直接接收三个电机目标。输入来自
`/cmd_vel_safe`：`linear.x=vx`（前进为正，m/s）、`linear.y=vy`（左移为正，m/s）、
`angular.z=wz`（逆时针为正，rad/s）。节点在
`src/kiwi_base/src/kiwi_base_node.cpp` 的 `update()` 中按以下顺序处理：

```text
/cmd_vel_safe
    -> 命令超时检查（100 ms）
    -> 机体 vx/vy/wz 限幅
    -> 加速度/减速度斜坡
    -> KiwiKinematics::inverse()
    -> 三路轮速统一限幅（max_wheel_speed_rad_s）
    -> 乘 19:1 减速比，得到电机转子轴 rad/s
    -> encode_command() 组装 0x3A 帧
```

当前联调先把 ROS 机体速度转换为固件预定义控制输入：

```text
u_x  = -vy       （ROS 左移正方向对应控制输入 -x）
u_y  = -vx       （ROS 前进正方向对应控制输入 -y）
u_wz =  wz       （旋转方向不变）
```

再按轮序 1、2、3 使用固定映射：

```text
w1 = 3 * (-0.5 * ux - 0.8660254 * uy - uwz)
w2 = 3 * ( ux              - uwz)
w3 = 3 * (-0.5 * ux + 0.8660254 * uy - uwz)
```

`3` 是当前 C 板联调标定比例，不能当作轮半径或减速比；`motor_reduction_ratio`
只在 `kiwi_base_node.cpp:465` 附近乘一次。`wheel_sign[i]` 会对对应一路整体乘以
`+1/-1`，用于低速点动时修正电机正方向。当前只检查几何参数有限、轮半径为正和方向符号合法，
不做 Jacobian 满秩检查；轮半径和轮心距尚未参与这套冻结的归一化映射。完成实车标定后，如需切换为物理 Jacobian，应同时更新
逆解、正解、测试和 MCU 目标单位，不能只改一行符号。

### 2. USB 到电机的下发链路

```text
KiwiKinematics::inverse()
    -> wheel_speed[0..2]（轮序 1/2/3）
    -> * motor_reduction_ratio[0..2]（默认 19）
    -> CommandFrame.motor_rad_s[0..2]
    -> encode_command(): 0x33 / len=12 / id=0x3A / 3 个 little-endian float32 / CRC16
    -> write(fd, 18 bytes) -> RoboMaster C 板 USB CDC
    -> RosBridge_Process(): SOF、长度、ID、CRC8、CRC16、有限值和量程检查
    -> command_snapshot.motor_target_rad_s[0..2]
    -> KiwiBase_Update(): 每路速度 PID（实测速度 vs 目标速度）
    -> KiwiBase_Send(): C620 CAN 电流命令
    -> M3508 减速电机
```

协议没有线上 `enable` 字段。上位机禁能、命令超时、软件急停或串口断开时，
`encode_command()` 将三个 float 全部编码为 0；C 板 100 ms watchdog、USB 未枚举、
任一电机离线或 PID 故障时也会清零并清除积分。两端都做归零是为了防止任一层保留旧速度。

### 3. 反馈和里程计回路

```text
M3508/C620 CAN 反馈 + BMI088
    -> C 板编码器累计、速度和 IMU 快照
    -> USB 0x3B（12 个 float32）
    -> FrameParser + decode_motor_imu_feedback()
    -> 电机轴速度 / 19 = 轮输出轴速度
    -> KiwiKinematics::forward()
    -> /wheel/odom、/joint_states、/imu/data_raw
    -> robot_localization EKF（后续）
```

当前 C++ 节点的 `send_command()` 是 USB 唯一写入点；C 板的
`RosBridge_Process()` 是 USB 唯一命令解析点，`KiwiBase_Update()` 是速度闭环入口，
`KiwiBase_Send()` 是 CAN 输出入口。键盘节点只发布 `/cmd_vel_safe`，不应直接打开串口。

## 常用命令

以下命令默认在 `/home/ym/ros2_ws` 执行。

### 构建和加载环境

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kiwi_base
source install/setup.bash
```

### 启动底盘节点

默认读取已安装的 `config/kiwi_base.yaml`，默认设备为 `/dev/robomaster_c`：

```bash
ros2 launch kiwi_base kiwi_base.launch.py
```

覆盖设备路径：

```bash
ros2 launch kiwi_base kiwi_base.launch.py device:=/dev/ttyACM0
```

默认配置 `require_feedback_for_motion: false`，节点无需等待 `0x80` 反馈或
`MOTOR_ONLINE` 标志即可发送非零 `0x3A` 指令。将其设为 `true` 可恢复反馈联锁。
直接下发模式仅用于联调，下位机的 100 ms 命令看门狗和物理急停必须保持有效。

### 键盘控制

底盘节点保持运行，在另一个终端执行：

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run kiwi_base kiwi_teleop_keyboard
```

按键为 `w/s` 前后、`a/d` 横移、`q/e` 旋转、空格或 `x` 停止。键盘 250 ms 没有新按键时自动发送零速。

### 查看反馈和诊断

```bash
ros2 topic echo /diagnostics
ros2 topic echo /joint_states
ros2 topic echo /wheel/odom
ros2 topic echo /imu/data_raw
ros2 topic echo /battery_state
ros2 topic hz /wheel/odom
ros2 topic hz /imu/data_raw
```

确认当前反馈联锁配置：

```bash
ros2 param get /kiwi_base_node require_feedback_for_motion
```

### 软件急停

触发急停：

```bash
ros2 topic pub --once /kiwi/estop std_msgs/msg/Bool "{data: true}"
```

解除急停：

```bash
ros2 topic pub --once /kiwi/estop std_msgs/msg/Bool "{data: false}"
```

解除后必须重新发送新鲜速度命令。软件急停不能代替动力回路物理急停。

### USB 和协议自检

检查设备：

```bash
ls -l /dev/robomaster_c
udevadm info -q property -n /dev/robomaster_c
```

无硬件协议自检：

```bash
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
```

只生成帧、不打开 USB：

```bash
python3 src/kiwi_base/tools/kiwi_serial_test.py \
  --motor-rad-s 1 -2 3 --enable --dry-run --print-hex
```

只接收下位机返回的 `0x3B` 十二浮点组合反馈：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py \
  --port /dev/robomaster_c --print-hex
```

收到 10 帧后自动退出：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py \
  --port /dev/robomaster_c --count 10
```

无硬件验证 `0x3B` 帧、CRC、半帧和粘包解析：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```

实机已确认 12 个小端 `float32` 的顺序为：三个电机实际速度、`roll/pitch/yaw`、
角速度 `xyz`、线加速度 `xyz`。`kiwi_base_node` 会解析 `0x3B`，发布轮里程计、
关节状态和 `/imu/data_raw`；电机反馈单位仍由 `feedback_motor_scale_to_rad_s` 修正。
接收脚本优先使用 `pyserial`，未安装时会自动使用 Ubuntu/Linux 标准库串口实现。

直接操作真实 USB 前必须停止 `kiwi_base_node`，保证同一时刻只有一个进程占用设备。

### 实时查看节点收发帧

联调时通过 launch 参数开启串口逐帧日志：

```bash
ros2 launch kiwi_base kiwi_base.launch.py log_serial_frames:=true
```

终端会逐帧显示：

```text
CMD enabled=true twist=[vx=0.100000 vy=0.000000 wz=0.000000] motor_rad_s=[-2.850000 5.700000 -2.850000]
TX[18] 33 0c 3a 65 ...
RX[54] 33 30 3b 5b ...
```

已有后台节点占用串口时，不要再启动第二个节点。可动态开启日志并跟踪当前日志文件：

```bash
ros2 param set /kiwi_base_node log_serial_frames true
tail -f /tmp/kiwi_base_launch.log
```

查看完成后按 `Ctrl+C` 退出 `tail`，再关闭高频帧日志：

```bash
ros2 param set /kiwi_base_node log_serial_frames false
```

该参数默认关闭，避免长期运行时产生高频日志开销。若设备名不是
`/dev/robomaster_c`，可同时传入 `device:=/dev/ttyACM0`。

### 运行测试

```bash
cd /home/ym/ros2_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select kiwi_base
colcon test-result --verbose
```

首次带电联调必须架空车轮、限制电源电流并准备物理断电。
