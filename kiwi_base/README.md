# kiwi_base

ROS 2 Humble 三轮 Kiwi 全向底盘驱动。上位机负责 `Twist` 安全门控、三轮逆运动学和 USB CDC 通信；RoboMaster C 型板负责三路 M3508 实时速度闭环、编码器与 IMU 反馈。

协议交接文档：[USB_CDC_PROTOCOL.md](USB_CDC_PROTOCOL.md)

参数说明：[config/README.md](config/README.md)

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
