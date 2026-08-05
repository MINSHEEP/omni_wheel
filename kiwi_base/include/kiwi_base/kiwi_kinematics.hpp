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
 * 当前控制使用已经过底盘验证的固定方向公式；输入坐标约定为前进=-y、左移=-x。
 * 该类不依赖 ROS 或串口，便于在没有硬件时单独验证轮序、方向和速度映射。
 *
 * 注意：当前 C 板联调协议使用固定的归一化 Kiwi 矩阵，代码中的 3.0 是
 * 标定比例；它不等于轮半径，也不等于 M3508 的 19:1 减速比。主机节点
 * 在编码 0x3A 帧前才会单独乘减速比。
 */
class KiwiKinematics
{
public:
  /** 创建无状态固定公式运动学对象。 */
  KiwiKinematics() = default;

  /** 将机体 Twist 逆解为轮 1/2/3 的联调速度，单位按当前协议约定为 rad/s。 */
  std::array<double, 3> inverse(const Twist2D &twist) const;

  /** 将三个轮子的实测速度按同一矩阵正解为机体 Twist。 */
  Twist2D forward(const std::array<double, 3> &wheel_rad_s) const;

private:
#if 0
  // 历史限幅接口和物理 Jacobian 字段，当前不参与运行。
  double max_wheel_speed(const Twist2D &twist) const;
  std::array<WheelGeometry, 3> wheels_{};
  double j_[3][3]{};
  double pinv_[3][3]{};
#endif
};
}  // namespace kiwi_base
