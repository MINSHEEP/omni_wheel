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

/**
 * 单个全向轮的几何与编码器符号。
 * `x/y` 和 `tangent_angle_rad` 用于启动时检查底盘几何是否可用；
 * `sign` 是该路电机正方向相对 ROS 约定的整体修正，只允许 +1/-1。
 */
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
 * 构造时校验几何参数。当前控制使用已经过底盘验证的固定方向矩阵；该类不依赖
 * ROS 或串口，便于在没有硬件时单独验证轮序、方向和机体速度映射。
 *
 * 注意：当前 C 板联调协议使用固定的归一化 Kiwi 矩阵，代码中的 3.0 是
 * 标定比例；它不等于轮半径，也不等于 M3508 的 19:1 减速比。主机节点
 * 在编码 0x3A 帧前才会单独乘减速比。
 */
class KiwiKinematics
{
public:
  /** 根据三个轮子的几何参数构造并检查有限值、半径和方向符号。 */
  explicit KiwiKinematics(const std::array<WheelGeometry, 3> &wheels);

  /** 返回构造时的参数/矩阵校验结果。 */
  bool valid() const noexcept { return valid_; }

  /** 返回最近一次校验失败的可读原因；校验成功时为空字符串。 */
  const std::string &error() const noexcept { return error_; }

  /** 将机体 Twist 逆解为轮 1/2/3 的联调速度，单位按当前协议约定为 rad/s。 */
  std::array<double, 3> inverse(const Twist2D &twist) const;

  /** 将三个轮子的实测速度按同一矩阵正解为机体 Twist。 */
  Twist2D forward(const std::array<double, 3> &wheel_rad_s) const;

  /** 返回给定机体速度对应的最大轮速绝对值，用于限幅。 */
  double max_wheel_speed(const Twist2D &twist) const;

private:
  std::array<WheelGeometry, 3> wheels_{};
  // 物理几何 Jacobian 及其伪逆代码当前在 .cpp 中以 #if 0 保留，不参与运行。
  // 当前 inverse()/forward() 使用的固定联调映射见 kiwi_kinematics.cpp 的 kCommandMap。
  double j_[3][3]{};
  double pinv_[3][3]{};
  bool valid_{false};
  std::string error_;
};
}  // namespace kiwi_base
