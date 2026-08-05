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
    // 0/120/240 度布置中的 sin(60°)。保留足够精度，避免正逆解出现可见偏差。
    constexpr double kSin60 = 0.866025403784;

    // 当前 C 板联调已经冻结的“归一化轮速”比例。
    // 它是项目标定系数，不是轮半径或减速比；减速比在 kiwi_base_node 中单独处理。
    constexpr double kMotorCommandScale = 3.0;

    // 固定轮序 [轮1, 轮2, 轮3] 的底盘控制输入映射：列依次为 [u_x, u_y, u_wz]。
    // 当前固件的方向约定是 u_x=-ROS vy、u_y=-ROS vx、u_wz=ROS wz。
    // w = kMotorCommandScale * K * [u_x, u_y, u_wz].
    constexpr double kCommandMap[3][3] = {
      {-0.5, -kSin60, -1.0},
      {1.0, 0.0, -1.0},
      {-0.5, kSin60, -1.0}};
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
    // 这里仍按实物几何检查满秩，确保参数不是重复轮心或平行轮；
    // 当前 USB 联调的数值映射使用下面的固定 K 矩阵，以兼容已烧录的 C 板。
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
  /**
   * 将机体速度转换为三个轮子的联调目标速度。
   *
   * 输入：ROS base_link 坐标系的 vx/vy（m/s）和 wz（rad/s）。
   * 输出：轮 1/2/3 的归一化输出轴速度，随后由 kiwi_base_node 乘以 19:1
   *       减速比编码为 C 板所需的电机转子轴 rad/s。
   *
   * 先按固件约定换轴：u_x=-vy、u_y=-vx、u_wz=wz；再代入固定矩阵：
   *   w1 = 3 * ( sin60*vx + 0.5*vy - wz)
   *   w2 = 3 * (-vy              - wz)
   *   w3 = 3 * (-sin60*vx + 0.5*vy - wz)
   * `wheel_sign[i]` 是该路电机/编码器的整体方向修正，只能取 +1 或 -1。
   */
  std::array<double, 3> KiwiKinematics::inverse(const Twist2D &t) const
  {
    std::array<double, 3> out{};
    if (!valid_ || !std::isfinite(t.vx) || !std::isfinite(t.vy) || !std::isfinite(t.wz))
      return out;
    // 不在键盘节点或 MCU 再做一次换轴，轴变换只在这里发生一次。
    const double control_velocity[3] = {-t.vy, -t.vx, t.wz};
    for (size_t i = 0; i < 3; ++i)
    {
      double command = 0.0;
      for (size_t k = 0; k < 3; ++k)
        command += kCommandMap[i][k] * control_velocity[k];
      out[i] = wheels_[i].sign * kMotorCommandScale * command;
    }
    return out;
  }

  /**
   * 将三个轮子的实测速度恢复为机体速度。
   * 该函数使用 inverse() 同一套固定矩阵的解析逆变换；先撤销 wheel_sign，
   * 因此“电机反向”只会影响对应一路，不会改变其它两路的符号。
   */
  Twist2D KiwiKinematics::forward(const std::array<double, 3> &w) const
  {
    Twist2D t;
    if (!valid_ || !std::isfinite(w[0]) || !std::isfinite(w[1]) || !std::isfinite(w[2]))
      return t;
    const double signed_w0 = wheels_[0].sign * w[0];
    const double signed_w1 = wheels_[1].sign * w[1];
    const double signed_w2 = wheels_[2].sign * w[2];
    const double denominator = 3.0 * kMotorCommandScale;
    // 先求固件控制坐标 [u_x,u_y,u_wz]，再还原到 ROS [vx,vy,wz]。
    const double control_x = (2.0 * signed_w1 - signed_w0 - signed_w2) / denominator;
    const double control_y = (signed_w2 - signed_w0) / (2.0 * kSin60 * kMotorCommandScale);
    t.vx = -control_y;
    t.vy = -control_x;
    t.wz = -(signed_w0 + signed_w1 + signed_w2) / denominator;
    return t;
  }
  /** 取逆解后三个轮速绝对值的最大值，用于主机命令限幅。 */
  double KiwiKinematics::max_wheel_speed(const Twist2D &t) const
  {
    const auto w = inverse(t);
    return std::max({std::abs(w[0]), std::abs(w[1]), std::abs(w[2])});
  }
} // namespace kiwi_base
