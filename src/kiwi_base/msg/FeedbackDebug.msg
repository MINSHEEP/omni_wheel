# C 板 0x3B 反馈帧的可视化调试消息；字段顺序与 48 字节 payload 一致。
std_msgs/Header header              # 接收时间戳；frame_id 为 chassis_imu_link
float32 motor_1_actual_rad_s        # 电机 1 转子轴实际角速度，单位 rad/s
float32 motor_2_actual_rad_s        # 电机 2 转子轴实际角速度，单位 rad/s
float32 motor_3_actual_rad_s        # 电机 3 转子轴实际角速度，单位 rad/s
float32 roll_rad                    # C 板解算横滚角，单位 rad
float32 pitch_rad                   # C 板解算俯仰角，单位 rad
float32 yaw_rad                     # C 板解算航向角，逆时针为正，单位 rad
float32 gyro_x_raw_rad_s            # C 板原始 X 轴角速度，未做 ROS 符号修正
float32 gyro_y_raw_rad_s            # C 板原始 Y 轴角速度，未做 ROS 符号修正
float32 gyro_z_raw_rad_s            # C 板原始 Z 轴角速度，原协议逆时针为负
float32 accel_x_m_s2                # C 板原始 X 轴线加速度，单位 m/s^2
float32 accel_y_m_s2                # C 板原始 Y 轴线加速度，单位 m/s^2
float32 accel_z_m_s2                # C 板原始 Z 轴线加速度，单位 m/s^2
