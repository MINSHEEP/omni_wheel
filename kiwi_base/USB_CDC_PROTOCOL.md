# Kiwi Base USB CDC 通信协议

本文档只描述当前 RoboMaster C 板与 ROS 2 主机已经联调的两类数据帧：

```text
主机 -> C 板：0x33 / len=12 / id=0x3A，三个电机目标速度
C 板 -> 主机：0x33 / len=48 / id=0x3B，三个电机实际速度、欧拉角和六轴 IMU
```

协议实现位于 `src/kiwi_base/src/robomaster_protocol.cpp`，声明位于
`src/kiwi_base/include/kiwi_base/robomaster_protocol.hpp`。所有整数和
`float32` 均为 little-endian。

## 1. 通用帧格式

每帧总长度为 `4 + len + 2` 字节：

```text
偏移  长度  字段
0     1     sof       固定 0x33，同时作为协议版本
1     1     len       payload 字节数
2     1     id        数据方向和类型
3     1     crc8      CRC8(sof、len、id)
4     len   payload   数据区
4+len 2     crc16     CRC16(偏移 0 到 3+len)，低字节在前
```

当前 ID 只有两个：

```text
ID     方向          payload  总长度  含义
0x3A   主机 -> C 板  12 B     18 B    三电机目标角速度
0x3B   C 板 -> 主机  48 B     54 B    三电机实际速度 + RPY + 六轴 IMU
```

主机接收解析器只接受 `sof=0x33`、`id=0x3B`、`len=48` 的帧；控制帧不应回传给
接收解析器。收到未知 ID、错误长度、CRC 错误或非有限浮点值时，丢弃当前字节并继续
搜索下一个 `0x33`，这样可以从粘包和半帧中恢复。

## 2. CRC

CRC8 使用 DJI 反射算法：`init=0xff`，多项式反射值 `0x8c`，覆盖前三个字节。
CRC16 使用 DJI/CRC-CCITT 反射算法：`init=0xffff`，多项式反射值 `0x8408`，覆盖
`sof` 到 payload 末尾。CRC16 在线上按低字节、高字节发送。

两段校验的调用位置：

```text
encode_command()          生成 CRC8 和 CRC16
FrameParser::consume()    验证 CRC8 和 CRC16
decode_feedback()         只解码已通过两段 CRC 的 payload
```

## 3. 主机发送 0x3A

### 3.1 Payload

```text
payload 偏移  帧偏移  长度  类型  字段
0             4       4     f32   motor_1_target_rad_s
4             8       4     f32   motor_2_target_rad_s
8             12      4     f32   motor_3_target_rad_s
```

目标值是 M3508 转子轴角速度 `rad/s`。上位机先按固定 Kiwi 公式得到轮输出轴速度，
再乘 `motor_reduction_ratio`（默认 19）后填入这三个字段。协议没有 enable 字段；
停止或禁能时三个值全部发送 `0.0f`。

### 3.2 示例

零速命令的完整 18 字节 golden frame：

```text
33 0c 3a 65 00 00 00 00 00 00 00 00 00 00 00 00 02 fd
```

代码入口：`kiwi_base::encode_command()`；节点发送入口：
`KiwiBaseNode::send_command()`。

## 4. C 板发送 0x3B

### 4.1 Payload 布局

该帧固定 48 字节，由 12 个 little-endian `float32` 组成。下位机只发送这一种反馈包，
不再拆分轮状态、IMU 或电源包。

```text
payload 偏移  帧偏移  长度  类型  字段
0             4       4     f32   motor_1_actual
4             8       4     f32   motor_2_actual
8             12      4     f32   motor_3_actual
12            16      4     f32   roll_rad
16            20      4     f32   pitch_rad
20            24      4     f32   yaw_rad
24            28      4     f32   gyro_x_rad_s
28            32      4     f32   gyro_y_rad_s
32            36      4     f32   gyro_z_rad_s
36            40      4     f32   accel_x_m_s2
40            44      4     f32   accel_y_m_s2
44            48      4     f32   accel_z_m_s2
```

`motor_*_actual` 的单位必须和双方约定一致；当前 ROS 默认将其视为电机转子轴 `rad/s`，
再除以 19 得到轮输出轴速度。若实测单位是 rpm，只修改
`feedback_motor_scale_to_rad_s`，不修改帧布局。

### 4.2 接收和 ROS 发布

```text
USB read -> FrameParser::feed()
         -> FrameParser::FeedbackCallback
         -> KiwiBaseNode::on_feedback()
         -> decode_feedback()
         -> /wheel/odom、/joint_states、/imu/data_raw
```

`/wheel/odom` 使用三个实际轮速按固定公式正解并积分；节点只发布消息，不发布
`odom -> base_footprint` TF。`/imu/data_raw` 将 roll/pitch/yaw 转为四元数，同时原样
发布角速度和加速度。当前反馈帧没有电压、故障位或温度字段，因此节点不发布
`sensor_msgs/msg/BatteryState` 或旧的独立状态话题。

### 4.3 抓包示例

```text
33 30 3b 5b ... 48 字节 payload ... ec 74
```

整帧长度是 54 字节；`0x5b` 是前三个字节 `33 30 3b` 的 CRC8，末尾 `ec 74` 是
little-endian CRC16。Python 接收工具会同时打印解码值和原始帧：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py \
  --port /dev/robomaster_c --print-hex
```

## 5. 实现文件职责

```text
include/kiwi_base/robomaster_protocol.hpp  常量、ID、CommandFrame、FeedbackFrame、解析器接口
src/robomaster_protocol.cpp                 CRC、0x3A 组帧、0x3B 解析和 12 浮点解码
src/kiwi_base_node.cpp                       USB 所有权、Twist 逆解、反馈发布
tools/kiwi_serial_test.py                    只发送/打印 0x3A 命令帧
tools/kiwi_feedback_receiver.py              只接收/解码/打印 0x3B 反馈帧
test/test_protocol.cpp                        命令 golden、反馈字段、分片粘包和 CRC 测试
```

## 6. 联调约束

1. 同一 USB CDC 设备只能由一个进程打开；运行 ROS 节点时不要同时运行两个串口工具。
2. ROS 节点参数 `log_serial_frames=true` 会输出 `TX[18]` 和 `RX[54]` 原始帧。
3. 下位机必须保证每个反馈包严格为 `0x33 0x30 0x3B` 开头、48 字节 payload，并按
   本文档计算 CRC8/CRC16。
4. 任何单位变更只修改单位换算参数或上层转换，不能改变 12 个字段的顺序和长度。
