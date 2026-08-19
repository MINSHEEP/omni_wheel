/**
 * @file kiwi_kinematics.cpp
 * @brief 实现三轮全向底盘的固定三行逆/正运动学公式。
 */
#include "kiwi_base/kiwi_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace kiwi_base
{
  namespace
  {
    // 0/120/240 度布置中的 sin(60°)。保留足够精度，避免正逆解出现可见偏差。
    constexpr double kSin60 = 0.866025403784;

  }

  Twist2D limit_twist(const Twist2D &input, const Twist2D &limits)
  {
    const auto limit = [](double value, double max_abs)
      { return std::isfinite(value) ? std::clamp(value, -max_abs, max_abs) : 0.0; };
    return {
      limit(input.vx, limits.vx),
      limit(input.vy, limits.vy),
      limit(input.wz, limits.wz)};
  }

  /** 保存实际几何参数；非法尺寸直接阻止节点带错误比例运行。 */
  KiwiKinematics::KiwiKinematics(
    double wheel_radius_m, double wheel_center_distance_m,
    double max_wheel_speed_rad_s)
  : wheel_radius_m_(wheel_radius_m),
    wheel_center_distance_m_(wheel_center_distance_m),
    max_wheel_speed_rad_s_(max_wheel_speed_rad_s)
  {
    if (!std::isfinite(wheel_radius_m_) || wheel_radius_m_ <= 0.0 ||
        !std::isfinite(wheel_center_distance_m_) || wheel_center_distance_m_ <= 0.0 ||
        !std::isfinite(max_wheel_speed_rad_s_) || max_wheel_speed_rad_s_ <= 0.0)
      throw std::invalid_argument(
        "wheel geometry and max speed must be positive; geometry must be finite");
  }

  /**
   * 将 ROS 机体速度转换为三个轮输出轴角速度。
   *
   * 坐标遵循 ROS 平面约定：vx 向前、vy 向左、wz 逆时针；
   * R 为轮半径，L 为中心到轮心距离：
   *   w1 = ( sin60*vx + 0.5*vy - L*wz) / R
   *   w2 = (            -vy         - L*wz) / R
   *   w3 = (-sin60*vx + 0.5*vy - L*wz) / R
   */
  std::array<double, 3> KiwiKinematics::inverse(const Twist2D &t) const
  {
    return inverse_with_twist(t).wheel_rad_s;
  }

  InverseResult KiwiKinematics::inverse_with_twist(const Twist2D &t) const
  {
    InverseResult result{};
    if (!std::isfinite(t.vx) || !std::isfinite(t.vy) || !std::isfinite(t.wz))
      return result;

    // 先用扩展精度完成逆解，避免极大但仍有限的输入在缩放前溢出成 Inf，
    // 随后出现 Inf * 0 并把 NaN 送入底盘命令。
    const long double vx = static_cast<long double>(t.vx);
    const long double vy = static_cast<long double>(t.vy);
    const long double wz = static_cast<long double>(t.wz);
    const long double wheel_radius = static_cast<long double>(wheel_radius_m_);
    const long double rotation_speed =
      static_cast<long double>(wheel_center_distance_m_) * wz;
    const std::array<long double, 3> raw_wheel_speed{
      (static_cast<long double>(kSin60) * vx + 0.5L * vy - rotation_speed) / wheel_radius,
      (-vy - rotation_speed) / wheel_radius,
      (-static_cast<long double>(kSin60) * vx + 0.5L * vy - rotation_speed) /
        wheel_radius};

    const long double peak_wheel_speed = std::max({
      std::abs(raw_wheel_speed[0]), std::abs(raw_wheel_speed[1]),
      std::abs(raw_wheel_speed[2])});
    const long double effective_limit = std::min(
      static_cast<long double>(max_wheel_speed_rad_s_),
      static_cast<long double>(std::numeric_limits<double>::max()));
    long double alpha = 1.0L;
    if (peak_wheel_speed > effective_limit)
    {
      // 同一个 alpha 同时作用于轮速和 Twist，避免改变合成运动方向。
      alpha = effective_limit / peak_wheel_speed;
    }

    for (std::size_t index = 0; index < result.wheel_rad_s.size(); ++index)
      result.wheel_rad_s[index] = static_cast<double>(raw_wheel_speed[index] * alpha);
    result.achievable_twist.vx = static_cast<double>(vx * alpha);
    result.achievable_twist.vy = static_cast<double>(vy * alpha);
    result.achievable_twist.wz = static_cast<double>(wz * alpha);
    return result;
  }

  /**
   * 将三个轮子的实测速度恢复为机体速度。
   * 该函数使用 inverse() 同一套固定公式的解析逆变换。
   */
  Twist2D KiwiKinematics::forward(const std::array<double, 3> &w) const
  {
    Twist2D t{};
    // forward() 是可独立调用的公共边界；不能假设输入一定经过串口协议层校验。
    if (!std::isfinite(w[0]) || !std::isfinite(w[1]) || !std::isfinite(w[2]))
      return t;
    // 与上面的 w1/w2/w3 逆解严格对应，直接恢复 ROS 机体速度。
    t.vx = wheel_radius_m_ * (w[0] - w[2]) / (2.0 * kSin60);
    t.vy = wheel_radius_m_ * (w[0] + w[2] - 2.0 * w[1]) / 3.0;
    t.wz = -wheel_radius_m_ * (w[0] + w[1] + w[2]) /
      (3.0 * wheel_center_distance_m_);
    return t;
  }
} // namespace kiwi_base
