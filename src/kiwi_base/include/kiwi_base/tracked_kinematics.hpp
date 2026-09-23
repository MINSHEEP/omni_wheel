#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace kiwi_base
{

// 固件 motor1/motor2 是左右履带命令；第三路固定为零。
class TrackedKinematics
{
public:
  TrackedKinematics(double separation_m, double command_per_m_s, double max_command)
  : separation_m_(separation_m), command_per_m_s_(command_per_m_s), max_command_(max_command)
  {
    if (!std::isfinite(separation_m_) || separation_m_ <= 0.0 ||
        !std::isfinite(command_per_m_s_) || command_per_m_s_ <= 0.0 ||
        !std::isfinite(max_command_) || max_command_ <= 0.0)
      throw std::invalid_argument("tracked geometry and command calibration must be positive");
  }

  std::pair<float, float> command(double vx, double wz) const
  {
    if (!std::isfinite(vx) || !std::isfinite(wz))
      return {0.0F, 0.0F};
    double left = (vx - 0.5 * separation_m_ * wz) * command_per_m_s_;
    double right = (vx + 0.5 * separation_m_ * wz) * command_per_m_s_;
    const double largest = std::max(std::abs(left), std::abs(right));
    if (largest > max_command_)
    {
      const double scale = max_command_ / largest;
      left *= scale;
      right *= scale;
    }
    return {static_cast<float>(left), static_cast<float>(right)};
  }

private:
  double separation_m_;
  double command_per_m_s_;
  double max_command_;
};

}  // namespace kiwi_base
