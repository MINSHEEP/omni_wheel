/**
 * @file kiwi_kinematics.cpp
 * @brief 实现三轮全向底盘的固定三行逆/正运动学公式。
 */
#include "kiwi_base/kiwi_kinematics.hpp"
#include <cmath>
namespace kiwi_base
{
  namespace
  {
    // 0/120/240 度布置中的 sin(60°)。保留足够精度，避免正逆解出现可见偏差。
    constexpr double kSin60 = 0.866025403784;

    // 旧的矩阵写法保留在此处供对照；当前 inverse() 直接写出三行公式。
#if 0
    constexpr double kCommandMap[3][3] = {
      {-0.5, -kSin60, -1.0},
      {1.0, 0.0, -1.0},
      {-0.5, kSin60, -1.0}};
#endif
  }

#if 0
  /** 历史几何构造函数，仅保留供物理 Jacobian 恢复时参考。 */
  KiwiKinematics::KiwiKinematics(const std::array<WheelGeometry, 3> &wheels)
  {
    (void)wheels;
#if 0
    // 旧版本的几何参数校验和 Jacobian 计算保留在此处，不参与当前固定公式控制。
    for (const auto &w : wheels_)
      if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.tangent_angle_rad) ||
          !(w.radius > 0.0) || !std::isfinite(w.radius) || !std::isfinite(w.sign) ||
          std::abs(w.sign) != 1.0)
      {
        error_ = "wheel geometry must be finite; radius positive and sign +/-1";
        return;
      }
    // 旧的物理 Jacobian/伪逆路径暂时保留，不参与当前控制，也不做满秩检查。
#if 0
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
#endif
#endif
    // 当前控制方向和比例已经由底盘联调确定，不依赖 WheelGeometry。
    valid_ = true;
  }
#endif
  /**
   * 将机体速度转换为三个轮子的联调目标速度。
   *
   * 输入：当前已标定控制坐标的 vx/vy（m/s）和 wz（rad/s）；前进为 vy<0、左移为 vx<0。
   * 输出：轮 1/2/3 的输出轴角速度，随后由 kiwi_base_node 乘以 19:1
   *       减速比编码为 C 板所需的电机转子轴 rad/s。
   *
   * 直接使用当前实车确认的方向公式，不再经过换轴变量或矩阵循环：
   *   w1 =  sin60*vx + 0.5*vy - wz
   *   w2 = -vy              - wz
   *   w3 = -sin60*vx + 0.5*vy - wz
   */
  std::array<double, 3> KiwiKinematics::inverse(const Twist2D &t) const
  {
    std::array<double, 3> out{};
#if 0
    // 联调阶段不做输入限幅/有限值门控；恢复保护时在节点统一处理。
    if (!std::isfinite(t.vx) || !std::isfinite(t.vy) || !std::isfinite(t.wz))
      return out;
#endif
    // 公式直接对应当前底盘控制坐标；不再额外执行 control_velocity 换轴。
    out[0] = kSin60 * t.vx + 0.5 * t.vy - t.wz;
    out[1] = -t.vy - t.wz;
    out[2] = -kSin60 * t.vx + 0.5 * t.vy - t.wz;
    return out;
  }

  /**
   * 将三个轮子的实测速度恢复为机体速度。
   * 该函数使用 inverse() 同一套固定公式的解析逆变换。
   */
  Twist2D KiwiKinematics::forward(const std::array<double, 3> &w) const
  {
    Twist2D t;
#if 0
    // 联调阶段不做反馈有限值门控；协议层仍负责 CRC 和帧长度。
    if (!std::isfinite(w[0]) || !std::isfinite(w[1]) || !std::isfinite(w[2]))
      return t;
#endif
    // 与上面三行逆解公式对应的解析正解。
    t.vx = (w[0] - w[2]) / (2.0 * kSin60);
    t.vy = (w[0] + w[2] - 2.0 * w[1]) / 3.0;
    t.wz = -(w[0] + w[1] + w[2]) / 3.0;
    return t;
  }
#if 0
  /** 历史限幅接口，当前阶段不参与运行。 */
  double KiwiKinematics::max_wheel_speed(const Twist2D &t) const
  {
    const auto w = inverse(t);
    return std::max({std::abs(w[0]), std::abs(w[1]), std::abs(w[2])});
  }
#endif
} // namespace kiwi_base
