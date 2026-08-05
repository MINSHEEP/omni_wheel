# Kiwi Base USB CDC 通信协议

> 日期：2026-08-05  
> 适用范围：ROS 2 Humble 上位机 `kiwi_base` 与 RoboMaster C 型板 Kiwi 固件  
> 当前状态：主机命令使用 `0x3A`；下位机组合反馈使用实机确认的 `0x3B`。
> C++ 节点和 Python 工具均已支持 `0x3B` 的十二个 float32 字段。

## 1. 结论和约束

USB CDC 按连续字节流处理，不保证一次 USB 读取恰好得到一帧。收发两端必须支持半帧、粘包和错误字节后的重新同步。

所有线上数据必须遵守以下规则：

- 多字节整数使用 little-endian；
- 浮点数使用 IEEE-754 binary32（C 语言 `float`），little-endian；
- `len` 只表示 payload 长度，不包含 4 字节帧头和 2 字节 CRC16；
- 总帧长固定为 `len + 6`；
- 禁止直接发送未压缩的 C 结构体，必须按本文偏移逐字段写入，避免填充和对齐差异；
- 收到 NaN、Inf、错误长度、未知 ID 或 CRC 错误时丢弃该帧；
- 当前轮序固定为 `motor_1/motor_2/motor_3`，对应 ROS `wheel_joint_1/2/3` 和 CAN ID `1/2/3`。

## 2. 通用帧格式

```text
字节偏移       长度   类型     字段          说明
0              1      u8       sof           固定 0x33，同时作为当前协议版本
1              1      u8       len           payload 字节数
2              1      u8       id            数据 ID
3              1      u8       crc8          只校验字节 0..2
4              len    bytes    payload       由 id 决定
4 + len        1      u8       crc16_low     CRC16 低字节
5 + len        1      u8       crc16_high    CRC16 高字节
```

CRC16 的输入范围包括 `sof、len、id、crc8、payload`，即字节 `0..(3+len)`。CRC16 自身两个字节不参与计算。

当前数据 ID：

```text
ID     方向          payload   总帧长   名称
0x3A   主机 -> MCU   12 B      18 B     MOTOR_COMMAND
0x3B   MCU -> 主机   48 B      54 B     MOTOR_IMU_FEEDBACK
0x80   MCU -> 主机   46 B      52 B     WHEEL_STATE（旧 ROS 适配格式）
0x81   MCU -> 主机   34 B      40 B     IMU_STATE（旧 ROS 适配格式）
0x82   MCU -> 主机   10 B      16 B     POWER_STATE（预留，当前不发送）
```

## 3. CRC 定义

### 3.1 CRC8

```text
输入范围：frame[0..2]
初始值：  0xFF
反射多项式：0x8C
结果位置：frame[3]
最终异或：无
```

### 3.2 CRC16

```text
输入范围：frame[0..3+len]
初始值：  0xFFFF
反射多项式：0x8408
结果位置：frame[4+len]、frame[5+len]
字节顺序：低字节在前
最终异或：无
```

下位机现有 DJI CRC 库可直接使用：

```c
frame[3] = get_CRC8_check_sum(frame, 3U, 0xFFU);
append_CRC16_check_sum(frame, total_frame_length);
```

## 4. 主机发送：MOTOR_COMMAND（0x3A）

### 4.1 Payload 布局

```text
payload 偏移   帧偏移   长度   类型    字段                    单位
0              4        4      f32     motor_1_target_rad_s    电机转子轴 rad/s
4              8        4      f32     motor_2_target_rad_s    电机转子轴 rad/s
8              12       4      f32     motor_3_target_rad_s    电机转子轴 rad/s
```

关键约定：

- 这是电机转子轴速度，不是减速箱输出轴或轮轴速度；
- 上位机已经把轮轴速度乘以 `19.0` 减速比，MCU 不得再次乘除减速比；
- 当前 MCU 接收范围是 `[-570, +570] rad/s`；
- payload 不包含 `enable`、序号或时间戳；上位机内部同名字段不在线传输；
- 三个目标均为 `0.0F` 表示软件停机，MCU 应清零输出并清除 PID 积分；
- 任意一路为 NaN/Inf 或超出范围时整帧拒绝，且不得刷新命令 watchdog。

### 4.2 发送和 watchdog

上位机当前按 50 Hz，也就是约每 20 ms 发送一帧。MCU watchdog 为 100 ms：只有长度、ID、CRC 和三个速度值全部有效的 `0x3A` 帧才能刷新 watchdog。超过 100 ms 未收到有效命令时，MCU 必须立即发送三路零电流命令并清除 PID。

### 4.3 Golden frame

三个目标均为 `0.0 rad/s`：

```text
33 0c 3a 65 00 00 00 00 00 00 00 00 00 00 00 00 02 fd
```

目标依次为 `1.0、-2.0、3.0 rad/s`：

```text
33 0c 3a 65 00 00 80 3f 00 00 00 c0 00 00 40 40 64 8e
```

下位机应先用第二个向量验证：

```text
motor_1_target_rad_s =  1.0F
motor_2_target_rad_s = -2.0F
motor_3_target_rad_s =  3.0F
```

## 5. MCU 发送：MOTOR_IMU_FEEDBACK（0x3B）

实机抓包确认该帧依次返回三个电机实际速度、计算后的欧拉角、IMU 角速度和线加速度。
电机反馈的原始单位仍需台架确认，主机通过 `feedback_motor_scale_to_rad_s` 转为电机轴 rad/s。

### 5.1 Payload 布局

```text
payload 偏移   帧偏移   长度   类型    字段
0              4        4      f32     motor_1_actual
4              8        4      f32     motor_2_actual
8              12       4      f32     motor_3_actual
12             16       4      f32     roll_rad
16             20       4      f32     pitch_rad
20             24       4      f32     yaw_rad
24             28       4      f32     gyro_x_rad_s
28             32       4      f32     gyro_y_rad_s
32             36       4      f32     gyro_z_rad_s
36             40       4      f32     accel_x_m_s2
40             44       4      f32     accel_y_m_s2
44             48       4      f32     accel_z_m_s2
```

payload 固定为 48 字节，完整帧固定为 54 字节，CRC16 位于偏移 `52、53`。

### 5.2 Golden frame

以下是用户实机抓到并通过 CRC8/CRC16 校验的静止反馈帧：

```text
33 30 3b 5b 00 00 00 00 00 00 00 00 00 00 00 00
00 40 47 c0 5c e2 70 3f 95 3f b1 bc e7 ae bc 3b
58 a0 8b 3a 5a c0 42 3b 12 73 fe c0 b9 33 55 3d
46 4f ba c0 ec 74
```

## 6. 旧 ROS 适配：WHEEL_STATE（0x80）

### 6.1 Payload 布局

```text
payload 偏移   帧偏移   长度   类型     字段                    单位/说明
0              4        2      u16      state_seq               每生成一帧加 1，自然回绕
2              6        2      u16      last_cmd_seq            MCU 本地有效命令计数
4              8        4      u32      mcu_time_ms              HAL_GetTick()，自然回绕
8              12       4      f32      wheel_1_position_rad     轮输出轴累计 rad
12             16       4      f32      wheel_2_position_rad     轮输出轴累计 rad
16             20       4      f32      wheel_3_position_rad     轮输出轴累计 rad
20             24       4      f32      wheel_1_velocity_rad_s   轮输出轴 rad/s
24             28       4      f32      wheel_2_velocity_rad_s   轮输出轴 rad/s
28             32       4      f32      wheel_3_velocity_rad_s   轮输出轴 rad/s
32             36       2      i16      motor_1_current_raw      DJI/C620 反馈原始值
34             38       2      i16      motor_2_current_raw      DJI/C620 反馈原始值
36             40       2      i16      motor_3_current_raw      DJI/C620 反馈原始值
38             42       2      u16      battery_mv               毫伏
40             44       4      u32      fault_bits              故障位
44             48       2      u16      flags                   状态位
```

完整帧 CRC16 位于偏移 `50、51`。当前 MCU 以 100 Hz 发送该帧。

`last_cmd_seq` 不是上位机序号。`0x3A` 没有线上序号，因此 MCU 每接受一帧完整有效命令，就把本地 16 位计数器加 1，并在这里回显。

`motor_current_raw` 暂时不能直接解释为安培；完成 C620 反馈码标定前，上位机只把它作为诊断原始量。

### 6.2 fault_bits

```text
bit    掩码          当前定义
0      0x00000001    电机 1/CAN 反馈离线
1      0x00000002    电机 2/CAN 反馈离线
2      0x00000004    电机 3/CAN 反馈离线
3..31  -             保留，发送 0
```

上位机始终把 `fault_bits` 发布到 `/diagnostics`。默认直接下发模式只报警、不阻塞
`0x3A`；启用 `require_feedback_for_motion` 后，任意非零故障位都会阻止非零目标。

### 6.3 flags

```text
bit    掩码     名称           当前定义
0      0x0001   ENABLED        电机闭环当前允许输出非零 CAN 命令
1      0x0002   ESTOP          急停已触发；当前硬件尚未接入，预留
2      0x0004   USB_ONLINE     USB CDC 已完成枚举
3      0x0008   MOTOR_ONLINE   三个电机 CAN 反馈均在线
4..15  -        RESERVED       保留，发送 0
```

启动时即使 `ENABLED=0`，MCU 也应持续发送 `WHEEL_STATE`。上位机默认配置
`require_feedback_for_motion=false`，不等待 `USB_ONLINE`、`MOTOR_ONLINE` 或 `fault_bits`
即可直接发送非零目标，这些字段只用于诊断。将该参数设为 `true` 后，上位机才要求
`USB_ONLINE=1、MOTOR_ONLINE=1、fault_bits=0` 且反馈新鲜，作为非零命令的启动联锁。

### 6.4 Golden frame

以下示例表示序号和轮数据全为 0、电池 24000 mV、`USB_ONLINE | MOTOR_ONLINE`：

```text
33 2e 80 79 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 c0 5d 00 00 00 00
0c 00 b2 a2
```

## 7. 旧 ROS 适配：IMU_STATE（0x81）

### 7.1 Payload 布局

```text
payload 偏移   帧偏移   长度   类型    字段             单位/说明
0              4        2      u16     seq              每生成一帧加 1，自然回绕
2              6        4      u32     mcu_time_ms      HAL_GetTick()，自然回绕
6              10       4      f32     gyro_x           rad/s
10             14       4      f32     gyro_y           rad/s
14             18       4      f32     gyro_z           rad/s
18             22       4      f32     accel_x          m/s^2
22             26       4      f32     accel_y          m/s^2
26             30       4      f32     accel_z          m/s^2
30             34       4      f32     temperature_c    摄氏度
```

完整帧 CRC16 位于偏移 `38、39`。当前 MCU 以 100 Hz 发送该帧，并与 `WHEEL_STATE` 错开发送。

当前注意事项：

- 三轴方向仍是板载 BMI088 当前输出顺序，装车后必须确认与 ROS `imu_link` 坐标一致；
- 当前固件未从 IMU API 取得温度，因此暂时发送 `0.0F`；
- 所有 7 个浮点数都必须是有限值，否则上位机会拒绝整帧。

### 7.2 Golden frame

序号、时间和全部 IMU 数据均为 0：

```text
33 22 81 6a 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 5f 2b
```

## 8. POWER_STATE（0x82，预留）

上位机当前能解析该帧，但 MCU 未实现发送，而且 `WHEEL_STATE` 已包含电池电压和 flags。当前联调阶段不要依赖或周期发送 `0x82`。

若后续决定启用，payload 固定如下：

```text
payload 偏移   帧偏移   长度   类型   字段
0              4        2      u16    seq
2              6        4      u32    mcu_time_ms
6              10       2      u16    battery_mv
8              12       2      u16    flags
```

## 9. 下位机接收状态机要求

推荐按以下顺序处理主机字节流：

1. 搜索 `0x33`；找不到时逐字节丢弃。
2. 累积满 4 字节帧头。
3. 校验 CRC8，并检查 `id=0x3A、len=12`。
4. 根据 `total_len=len+6` 等待完整帧，不得把半帧判为错误。
5. 校验整帧 CRC16。
6. 逐偏移读取三个 little-endian float32。
7. 检查三个值均有限且绝对值不大于 `570 rad/s`。
8. 原子更新三路命令快照、本地命令计数和 watchdog 时间。
9. 任一步失败都不能更新控制目标或 watchdog；重新搜索后续 `0x33`。

USB 接收解析不能长期阻塞 2 ms 电机控制周期。当前固件每周期最多处理 128 字节，剩余数据留在 CDC ring buffer 中下一周期处理。

## 10. 上位机接收行为

`kiwi_base_node` 和 `kiwi_feedback_receiver.py` 对 `0x3B` 的解析顺序为：

1. 在缓存中搜索 `0x33`；
2. 检查 `id=0x3B、len=48`；
3. 校验 CRC8 和 CRC16；
4. 按 little-endian 解码十二个 `float32`；
5. 检查所有值均为有限值；
6. 分发三电机反馈、欧拉角、角速度和线加速度。

`kiwi_base_node` 是运行时唯一串口所有者，同时发送 `0x3A`、接收 `0x3B`。Python 工具
只用于独占串口台架测试，不能与节点同时运行。

默认直接下发模式用于首轮 USB/CAN 联调，不能代替安全闭环。MCU 必须保留 100 ms 命令
watchdog，现场必须具备可直接切断动力的物理急停。

## 11. 联调验收

先架空车轮并准备物理断电，再按顺序验证：

1. 下位机用两个 `0x3A` golden frame 验证帧头、CRC 和 float 解码。
2. 下位机发送本节 `0x3B` golden frame，节点和 Python 脚本应输出十二个对应值。
3. 分段发送同一帧和连续发送多帧，确认脚本能处理半帧与粘包。
4. 篡改 payload 或 CRC，确认脚本增加 `crc_errors` 且不输出该帧。
5. 低速发送三路互不相同的目标，逐一核对反馈轮序、符号、单位和 CAN ID。
6. 主机持续发送零速，确认 MCU watchdog 保持在线但电机不使能。
7. 拔掉 USB 或停止主机进程，确认 MCU 在 100 ms 内清零三路 CAN 输出。

上位机无硬件协议自检：

```bash
cd /home/ym/ros2_ws
python3 src/kiwi_base/tools/kiwi_serial_test.py --self-test
```

打印 `1、-2、3 rad/s` 命令帧但不打开 USB：

```bash
python3 src/kiwi_base/tools/kiwi_serial_test.py \
  --motor-rad-s 1 -2 3 --enable --dry-run --print-hex
```

## 12. 代码对应关系

```text
上位机协议定义：src/kiwi_base/include/kiwi_base/robomaster_protocol.hpp
上位机编解码：  src/kiwi_base/src/robomaster_protocol.cpp
上位机 USB I/O：src/kiwi_base/src/kiwi_base_node.cpp
上位机测试：    src/kiwi_base/test/test_protocol.cpp
Python 联调工具：src/kiwi_base/tools/kiwi_serial_test.py
0x3B 接收工具： src/kiwi_base/tools/kiwi_feedback_receiver.py

下位机协议定义：standard-robot-pro/application/communication/ros_bridge.h
下位机编解码：  standard-robot-pro/application/communication/ros_bridge.c
下位机状态来源：standard-robot-pro/application/kiwi_base/kiwi_base.c
下位机任务周期：standard-robot-pro/Src/freertos.c
下位机 CRC 库： standard-robot-pro/components/support/CRC8_CRC16.c
```

修改任意 ID、字段偏移、单位或长度时，必须同时更新上下位机、本文、C++ 单元测试和 Python golden frame。禁止只修改 C 结构体而不更新逐字节序列化代码。


PID=$(pgrep -n kiwi_base_node)
sudo strace -f -xx -s 512 \
  -e trace=read,write \
  -p "$PID" 2>&1 |
  grep --line-buffered '\\x33'
