# Kiwi Base USB CDC 通信协议

RoboMaster C 板接收 0x3A 速度命令，并独立异步上报 0x3B 遥测。0x3A 没有 ACK，
两类帧不能按请求/应答关系处理：

```text
主机 -> C 板：0x33 / len=12 / id=0x3A，三个电机目标速度
C 板 -> 主机：0x33 / len=48 / id=0x3B，电机/欧拉角/IMU 异步遥测
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

代码中保留两个 ID：

```text
ID     方向          payload  总长度  含义
0x3A   主机 -> C 板  12 B     18 B    三电机目标角速度
0x3B   C 板 -> 主机  48 B     54 B    异步电机/IMU 遥测（不是命令 ACK）
```

0x3A 没有 ACK，主机从不等待 0x3B。节点在每个控制周期对同一非阻塞串口做有界读取，
`feedback_framed` 决定是否校验 0x3B 帧封装；没有遥测或持续遥测都不能饿死命令 TX。

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

## 4. 异步 0x3B 遥测

### 4.1 Payload 布局

该帧固定 48 字节，由 12 个 little-endian `float32` 组成。它独立于 0x3A 发送周期，
不能作为命令执行确认。

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
先乘 `feedback_motor_scale_to_rad_s`，再经过一阶低通、静止死区并除以 19，得到三个
轮输出轴速度。若实测单位是 rpm，只修改倍率，不修改帧布局或运动学公式。

### 4.2 当前反馈里程计运行方式

```text
/cmd_vel -> 限幅与轮速等比例饱和 -> 0x3A USB write
异步 0x3B -> 非阻塞解析 -> 电机倍率/低通/死区/减速比 -> 正运动学与积分
                                                        -> /chassis/odom/raw
                                                        -> /joint_states
                         -> IMU 处理 -> /chassis/imu/data_raw
                         -> 原始字段 -> /chassis/feedback/raw
```

`/chassis/odom/raw` 使用三个电机实测速度正解并积分；节点只发布消息，不发布
`odom -> base_footprint` TF。EKF 融合反馈正解的 `vx/vy/wz`，并发布
`/odometry/filtered` 和该 TF。反馈断流期间不使用 `/cmd_vel` 补算；恢复首帧重置低通状态
且积分 `dt=0`，防止把断流前旧速度或未知时间跨度计入位移。

`/chassis/imu/data_raw` 和 `/chassis/feedback/raw` 保持既有 Topic/Frame/QoS；只有收到并
通过 CRC/有限值校验的 0x3B 才发布。遥测断流不阻塞 0x3A 命令发送，但 odom/joint_states
也不会在没有实测速度时伪造更新。

### 4.3 兼容帧示例

```text
33 30 3b 5b ... 48 字节 payload ... ec 74
```

整帧长度是 54 字节；`0x5b` 是前三个字节 `33 30 3b` 的 CRC8，末尾 `ec 74` 是
little-endian CRC16。离线工具可用于协议自检：

```bash
python3 src/kiwi_base/tools/kiwi_feedback_receiver.py --self-test
```

## 5. 实现文件职责

```text
include/kiwi_base/robomaster_protocol.hpp  常量、ID、CommandFrame、FeedbackFrame、解析器接口
src/robomaster_protocol.cpp                 CRC、0x3A 组帧、0x3B 解析和 12 浮点解码
src/kiwi_base_node.cpp                       USB 所有权、Twist 逆解、反馈里程计、异步遥测
tools/kiwi_serial_test.py                    只发送/打印 0x3A 命令帧
tools/kiwi_feedback_receiver.py              不占用设备的 0x3B 离线解析自检工具
test/test_protocol.cpp                        命令 golden、反馈字段、分片粘包和 CRC 测试
```

## 6. 联调约束

1. 同一 USB CDC 设备只能由一个进程打开；运行 ROS 节点时不要同时运行两个串口工具。
2. ROS 节点参数 `log_serial_frames=true` 只用于输出命令映射；不得把缺少反馈记录为 TX 故障。
3. 里程计唯一使用经过低通和死区处理的遥测电机值；禁止回退为 `/cmd_vel` 开环积分。
4. 修改 0x3B 格式或单位前必须先冻结两端协议，并同步协议、PTY 与字段顺序测试。
