/**
 * @file kiwi_kinematics.hpp
 * @brief 三轮 Kiwi 全向底盘的运动学数据结构与正逆解接口。
 */
#pragma once

#include <array>

namespace kiwi_base
{

/** 机体平面速度，单位分别为 m/s、m/s 和 rad/s。 */
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/** 按 vx、vy、wz 各自的绝对值上限限制速度；非有限输入直接清零。 */
Twist2D limit_twist(const Twist2D &input, const Twist2D &limits);

#if 0
/** 历史物理模型参数，当前固定公式不读取。 */
struct WheelGeometry
{
  double x{0.0};
  double y{0.0};
  double tangent_angle_rad{0.0};
  double radius{0.0};
  double sign{1.0};
};
#endif

/**
 * 三轮 Kiwi 底盘的逆/正运动学。
 *
 * 坐标遵循 ROS 平面约定：+x 前进、+y 左移、+z 逆时针。
 * 轮序固定为实车电机 1/2/3：纯 +x 时轮速符号为 [+ 0 -]，
 * 纯 +y 时为 [+ - +]，纯 +wz 时为 [- - -]。
 * 该类不依赖 ROS 或串口，便于在没有硬件时单独验证轮序、方向和速度映射。
 *
 * 运动学输出为轮输出轴 rad/s；主机编码 0x3A 帧前再单独乘 M3508 的
 * 19:1 减速比，得到电机转子轴 rad/s。
 */
class KiwiKinematics
{
public:
  /** 使用实车轮半径和中心到轮心距离创建运动学对象。 */
  explicit KiwiKinematics(
    double wheel_radius_m = 0.1015,
    double wheel_center_distance_m = 0.32256);

  /** 将机体 Twist 逆解为轮 1/2/3 的输出轴角速度，单位为 rad/s。 */
  std::array<double, 3> inverse(const Twist2D &twist) const;

  /** 将三个轮子的实测速度按同一矩阵正解为机体 Twist。 */
  Twist2D forward(const std::array<double, 3> &wheel_rad_s) const;

private:
  double wheel_radius_m_{0.1015};
  double wheel_center_distance_m_{0.32256};

#if 0
  // 历史限幅接口和物理 Jacobian 字段，当前不参与运行。
  double max_wheel_speed(const Twist2D &twist) const;
  std::array<WheelGeometry, 3> wheels_{};
  double j_[3][3]{};
  double pinv_[3][3]{};
#endif
};
}  // namespace kiwi_base
