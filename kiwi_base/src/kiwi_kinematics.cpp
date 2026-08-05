/**
 * @file kiwi_kinematics.cpp
 * @brief 实现三轮全向底盘的 Jacobian 构建、逆运动学和正运动学。
 */
#include "kiwi_base/kiwi_kinematics.hpp"
#include <algorithm>
#include <cmath>
namespace kiwi_base
{
  namespace
  {
    constexpr double kSin60 = 0.866025;
    constexpr double kMotorCommandScale = 3.0;
  }

  /**
   * 根据轮心位置、滚动切向和轮半径构造 J；每一行描述一个轮子的角速度。
   * 同时计算 (J^T J)^-1 J^T，若半径、符号或矩阵秩不满足要求则保持 invalid。
   */
  KiwiKinematics::KiwiKinematics(const std::array<WheelGeometry, 3> &wheels) : wheels_(wheels)
  {
    for (const auto &w : wheels_)
      if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.tangent_angle_rad) ||
          !(w.radius > 0.0) || !std::isfinite(w.radius) || !std::isfinite(w.sign) ||
          std::abs(w.sign) != 1.0)
      {
        error_ = "wheel geometry must be finite; radius positive and sign +/-1";
        return;
      }
    // J 的列依次对应 vx、vy、wz，输出单位为轮输出轴 rad/s。
    for (size_t i = 0; i < 3; ++i)
    {
      const auto &w = wheels_[i];
      const double tx = std::cos(w.tangent_angle_rad), ty = std::sin(w.tangent_angle_rad);
      j_[i][0] = w.sign * tx / w.radius;
      j_[i][1] = w.sign * ty / w.radius;
      j_[i][2] = w.sign * (w.x * ty - w.y * tx) / w.radius;
    }
    // 三轮满秩时用高斯-约旦消元求 J 的正向伪逆，避免引入 Eigen 运行依赖。
    double a[3][3]{};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        for (int k = 0; k < 3; ++k)
          a[r][c] += j_[k][r] * j_[k][c];
    double aug[3][6]{};
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
        aug[r][c] = a[r][c];
      aug[r][r + 3] = 1.0;
    }
    for (int col = 0; col < 3; ++col)
    {
      int p = col;
      for (int r = col + 1; r < 3; ++r)
        if (std::abs(aug[r][col]) > std::abs(aug[p][col]))
          p = r;
      if (std::abs(aug[p][col]) < 1e-10)
      {
        error_ = "wheel Jacobian is rank deficient";
        return;
      }
      for (int c = 0; c < 6; ++c)
        std::swap(aug[col][c], aug[p][c]);
      const double d = aug[col][col];
      for (int c = 0; c < 6; ++c)
        aug[col][c] /= d;
      for (int r = 0; r < 3; ++r)
        if (r != col)
        {
          const double q = aug[r][col];
          for (int c = 0; c < 6; ++c)
            aug[r][c] -= q * aug[col][c];
        }
    }
    double inv[3][3]{};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        inv[r][c] = aug[r][c + 3];
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        for (int k = 0; k < 3; ++k)
          pinv_[r][c] += inv[r][k] * j_[c][k];
    valid_ = true;
  }
  /** 按固定轮序和当前联调比例，将机体 Twist 分别转换为三个电机速度。 */
  std::array<double, 3> KiwiKinematics::inverse(const Twist2D &t) const
  {
    std::array<double, 3> out{};
    if (!valid_)
      return out;
    out[0] = kMotorCommandScale * (-0.5 * -t.vx - kSin60 * t.vy - t.wz);
    out[1] = kMotorCommandScale * (-t.vx - t.wz);
    out[2] = kMotorCommandScale * (-0.5 * -t.vx + kSin60 * t.vy - t.wz);
    return out;
  }
  /** 使用当前固定三轮公式的解析逆变换恢复机体速度；无效配置返回零。 */
  Twist2D KiwiKinematics::forward(const std::array<double, 3> &w) const
  {
    Twist2D t;
    if (!valid_)
      return t;
    const double denominator = 3.0 * kMotorCommandScale;
    t.vx = (2.0 * w[1] - w[0] - w[2]) / denominator;
    t.vy = (w[2] - w[0]) / (2.0 * kSin60 * kMotorCommandScale);
    t.wz = -(w[0] + w[1] + w[2]) / denominator;
    return t;
  }
  /** 取逆解后三个轮速绝对值的最大值，用于主机命令限幅。 */
  double KiwiKinematics::max_wheel_speed(const Twist2D &t) const
  {
    const auto w = inverse(t);
    return std::max({std::abs(w[0]), std::abs(w[1]), std::abs(w[2])});
  }
} // namespace kiwi_base
