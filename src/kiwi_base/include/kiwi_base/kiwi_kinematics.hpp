/**
 * @file kiwi_kinematics.hpp
 * @brief 三轮 Kiwi 全向底盘的运动学数据结构与正逆解接口。
 */
#pragma once

#include <array>
#include <limits>

namespace kiwi_base
{

/** 机体平面速度，单位分别为 m/s、m/s 和 rad/s。 */
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/** 逆运动学结果：轮速与硬件实际可实现的等比例 Twist。 */
struct InverseResult
{
  std::array<double, 3> wheel_rad_s{};
  Twist2D achievable_twist{};
};

/** 按 vx、vy、wz 各自的绝对值上限限制速度；非有限输入直接清零。 */
Twist2D limit_twist(const Twist2D &input, const Twist2D &limits);

// 停用原因：实车轮序、方向和几何公式已固定，当前不需要通用轮几何矩阵。
// struct WheelGeometry
// {
//   double x{0.0};
//   double y{0.0};
//   double tangent_angle_rad{0.0};
//   double radius{0.0};
//   double sign{1.0};
// };

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
  /** 使用实车几何和轮输出轴速度上限创建运动学对象。 */
  explicit KiwiKinematics(
    double wheel_radius_m, double wheel_center_distance_m,
    double max_wheel_speed_rad_s = std::numeric_limits<double>::max());

  /** 将机体 Twist 逆解为轮速；超限时三个轮速保持比例一起缩放。 */
  std::array<double, 3> inverse(const Twist2D &twist) const;

  /** 同时返回饱和轮速与对应的可实现 Twist，供控制节点记录真实指令。 */
  InverseResult inverse_with_twist(const Twist2D &twist) const;

  /** 将三个轮子的实测速度按同一矩阵正解为机体 Twist。 */
  Twist2D forward(const std::array<double, 3> &wheel_rad_s) const;

private:
  double wheel_radius_m_;
  double wheel_center_distance_m_;
  double max_wheel_speed_rad_s_;
};
}  // namespace kiwi_base
