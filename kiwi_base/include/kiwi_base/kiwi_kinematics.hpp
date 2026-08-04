/**
 * @file kiwi_kinematics.hpp
 * @brief 三轮 Kiwi 全向底盘的运动学数据结构与正逆解接口。
 */
#pragma once

#include <array>
#include <string>

namespace kiwi_base
{

/** 机体平面速度，单位分别为 m/s、m/s 和 rad/s。 */
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/** 单个全向轮的几何与编码器符号；切向角以 base_link 的 +x 为 0。 */
struct WheelGeometry
{
  double x{0.0};
  double y{0.0};
  double tangent_angle_rad{0.0};
  double radius{0.0};
  double sign{1.0};
};

/**
 * 三轮 Kiwi 底盘的逆/正运动学。
 *
 * 构造时建立轮速 Jacobian 及其正向解。该类不依赖 ROS 或串口，便于在
 * 没有硬件时单独验证轮序、方向和几何参数。
 */
class KiwiKinematics
{
public:
  /** 根据三个轮子的几何参数构造并检查 Jacobian 是否满秩。 */
  explicit KiwiKinematics(const std::array<WheelGeometry, 3> &wheels);

  /** 返回构造时的参数/矩阵校验结果。 */
  bool valid() const noexcept { return valid_; }

  /** 返回最近一次校验失败的可读原因；校验成功时为空字符串。 */
  const std::string &error() const noexcept { return error_; }

  /** 将机体速度逆解为三个轮子的输出轴角速度，单位 rad/s。 */
  std::array<double, 3> inverse(const Twist2D &twist) const;

  /** 将三个轮子的实测输出轴角速度正解为机体速度。 */
  Twist2D forward(const std::array<double, 3> &wheel_rad_s) const;

  /** 返回给定机体速度对应的最大轮速绝对值，用于限幅。 */
  double max_wheel_speed(const Twist2D &twist) const;

private:
  std::array<WheelGeometry, 3> wheels_{};
  double j_[3][3]{};
  double pinv_[3][3]{};
  bool valid_{false};
  std::string error_;
};
}  // namespace kiwi_base
