# C 板 0x3B 反馈帧的可视化调试消息。
# 12 个数值分别定义为标量，Foxglove 可以直接按字段名选择曲线。
std_msgs/Header header
float32 motor_1_actual_rad_s
float32 motor_2_actual_rad_s
float32 motor_3_actual_rad_s
float32 roll_rad
float32 pitch_rad
float32 yaw_rad
float32 gyro_x_raw_rad_s
float32 gyro_y_raw_rad_s
float32 gyro_z_raw_rad_s
float32 accel_x_m_s2
float32 accel_y_m_s2
float32 accel_z_m_s2
