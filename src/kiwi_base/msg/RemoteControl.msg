# 当前 0x3C 原始状态，不是运动命令。没有滚轮 ch[4] 和接收机在线信息。
# 节点按控制周期发布；header.stamp 始终为最后一帧 CRC 合格数据的 ROS 接收时间。
# 启动未收到数据时 stamp=0；超时后保留旧值用于诊断，绝不能当作新指令。
std_msgs/Header header
uint32 mcu_timestamp_ms             # MCU 打包时间，不是遥控接收时间
int16[4] channels                   # ch[0..3]，回中约 0，未做死区或归一化
uint8 SWITCH_UP=1
uint8 SWITCH_DOWN=2
uint8 SWITCH_MID=3
uint8 sw_right
uint8 sw_left
bool frame_fresh                   # 最近收到 CRC 合格 0x3C，按主机单调时钟超时
bool controls_valid                # 通道在 [-700,700]，拨杆在 [1,3]，仅数值检查
# 即使两个标志均为 true，也不能证明遥控器在线；消费者还必须检测本话题断流。
