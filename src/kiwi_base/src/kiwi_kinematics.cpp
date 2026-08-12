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

    // 停用原因：三行解析公式比矩阵索引更直观，旧矩阵只保留为方向对照。
    // constexpr double kCommandMap[3][3] = {
    //   {-0.5, -kSin60, -1.0},
    //   {1.0, 0.0, -1.0},
    //   {-0.5, kSin60, -1.0}};
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

  // 停用原因：下面的通用 Jacobian/伪逆实现增加调试负担，且实车只使用固定三轮布局。
  // 原实现不删除，保留公式来源；重新启用时应先恢复 WheelGeometry 和对应单元测试。
  // KiwiKinematics::KiwiKinematics(const std::array<WheelGeometry, 3> &wheels)
  // {
  //   (void)wheels;
  //   for (const auto &w : wheels_)
  //     if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.tangent_angle_rad) ||
  //         !(w.radius > 0.0) || !std::isfinite(w.radius) || !std::isfinite(w.sign) ||
  //         std::abs(w.sign) != 1.0)
  //     {
  //       error_ = "wheel geometry must be finite; radius positive and sign +/-1";
  //       return;
  //     }
  //   for (size_t i = 0; i < 3; ++i)
  //   {
  //     const auto &w = wheels_[i];
  //     const double tx = std::cos(w.tangent_angle_rad);
  //     const double ty = std::sin(w.tangent_angle_rad);
  //     j_[i][0] = w.sign * tx / w.radius;
  //     j_[i][1] = w.sign * ty / w.radius;
  //     j_[i][2] = w.sign * (w.x * ty - w.y * tx) / w.radius;
  //   }
  //   // 原高斯-约旦求伪逆流程停用；固定布局的解析正解位于 forward()。
  //   double a[3][3]{};
  //   for (int r = 0; r < 3; ++r)
  //     for (int c = 0; c < 3; ++c)
  //       for (int k = 0; k < 3; ++k)
  //         a[r][c] += j_[k][r] * j_[k][c];
  //   double aug[3][6]{};
  //   for (int r = 0; r < 3; ++r)
  //   {
  //     for (int c = 0; c < 3; ++c)
  //       aug[r][c] = a[r][c];
  //     aug[r][r + 3] = 1.0;
  //   }
  //   for (int col = 0; col < 3; ++col)
  //   {
  //     int p = col;
  //     for (int r = col + 1; r < 3; ++r)
  //       if (std::abs(aug[r][col]) > std::abs(aug[p][col]))
  //         p = r;
  //     if (std::abs(aug[p][col]) < 1e-10)
  //     {
  //       error_ = "wheel Jacobian is rank deficient";
  //       return;
  //     }
  //     for (int c = 0; c < 6; ++c)
  //       std::swap(aug[col][c], aug[p][c]);
  //     const double d = aug[col][col];
  //     for (int c = 0; c < 6; ++c)
  //       aug[col][c] /= d;
  //     for (int r = 0; r < 3; ++r)
  //       if (r != col)
  //       {
  //         const double q = aug[r][col];
  //         for (int c = 0; c < 6; ++c)
  //           aug[r][c] -= q * aug[col][c];
  //       }
  //   }
  //   double inv[3][3]{};
  //   for (int r = 0; r < 3; ++r)
  //     for (int c = 0; c < 3; ++c)
  //       inv[r][c] = aug[r][c + 3];
  //   for (int r = 0; r < 3; ++r)
  //     for (int c = 0; c < 3; ++c)
  //       for (int k = 0; k < 3; ++k)
  //         pinv_[r][c] += inv[r][k] * j_[c][k];
  // }
  /** 保存实际几何参数；非法尺寸直接阻止节点带错误比例运行。 */
  KiwiKinematics::KiwiKinematics(double wheel_radius_m, double wheel_center_distance_m)
  : wheel_radius_m_(wheel_radius_m),
    wheel_center_distance_m_(wheel_center_distance_m)
  {
    if (!std::isfinite(wheel_radius_m_) || wheel_radius_m_ <= 0.0 ||
        !std::isfinite(wheel_center_distance_m_) || wheel_center_distance_m_ <= 0.0)
      throw std::invalid_argument("wheel radius and center distance must be finite and positive");
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
    std::array<double, 3> out{};
    // 停用原因：输入有限值处理和三轴限幅已集中在 limit_twist()，避免重复判断。
    // if (!std::isfinite(t.vx) || !std::isfinite(t.vy) || !std::isfinite(t.wz))
    //   return out;
    const double rotation_speed = wheel_center_distance_m_ * t.wz;
    out[0] = (kSin60 * t.vx + 0.5 * t.vy - rotation_speed) / wheel_radius_m_;
    out[1] = (-t.vy - rotation_speed) / wheel_radius_m_;
    out[2] = (-kSin60 * t.vx + 0.5 * t.vy - rotation_speed) / wheel_radius_m_;
    return out;
  }

  /**
   * 将三个轮子的实测速度恢复为机体速度。
   * 该函数使用 inverse() 同一套固定公式的解析逆变换。
   */
  Twist2D KiwiKinematics::forward(const std::array<double, 3> &w) const
  {
    Twist2D t;
    // 停用原因：decode_feedback() 已拒绝非有限 float，运动学层不重复校验协议数据。
    // if (!std::isfinite(w[0]) || !std::isfinite(w[1]) || !std::isfinite(w[2]))
    //   return t;
    // 与上面的 w1/w2/w3 逆解严格对应，直接恢复 ROS 机体速度。
    t.vx = wheel_radius_m_ * (w[0] - w[2]) / (2.0 * kSin60);
    t.vy = wheel_radius_m_ * (w[0] + w[2] - 2.0 * w[1]) / 3.0;
    t.wz = -wheel_radius_m_ * (w[0] + w[1] + w[2]) /
      (3.0 * wheel_center_distance_m_);
    return t;
  }
  // 停用原因：当前只限制机体 Twist，不在轮速层做二次缩放，避免改变运动方向比例。
  // double KiwiKinematics::max_wheel_speed(const Twist2D &t) const
  // {
  //   const auto w = inverse(t);
  //   return std::max({std::abs(w[0]), std::abs(w[1]), std::abs(w[2])});
  // }
} // namespace kiwi_base
