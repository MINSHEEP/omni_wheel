# C 板 0x3B 反馈帧的可视化调试消息。
# 所有数组长度固定为 3，单位分别写在字段名中；数值均为解码后的原始值。
std_msgs/Header header
float32[3] motor_actual_rad_s
float32[3] euler_rad
float32[3] gyro_raw_rad_s
float32[3] accel_m_s2
