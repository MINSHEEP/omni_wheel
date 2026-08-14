/**
 * @file measurement_processing.cpp
 * @brief 实现三轴测量的符号修正、静止死区、陀螺零漂估计与低通。
 */
#include "kiwi_base/measurement_processing.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace kiwi_base
{

Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband)
{
  return correct_vector(raw, sign, deadband, Vector3{1.0, 1.0, 1.0});
}

Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband, const Vector3 &scale)
{
  Vector3 output{};
  for (std::size_t i = 0; i < output.size(); ++i)
  {
    const double corrected = raw[i] * sign[i];
    output[i] = std::isfinite(corrected) && std::isfinite(scale[i]) &&
      scale[i] > 0.0 && std::abs(corrected) > deadband[i] ? corrected * scale[i] : 0.0;
  }
  return output;
}

GyroBiasFilter::GyroBiasFilter(const GyroFilterConfig &config)
: config_(config)
{
}

Vector3 GyroBiasFilter::process(const Vector3 &corrected, bool stationary, double dt)
{
  // dt 异常（首帧、串口断流恢复）时不推进任何随时间的状态，只做零漂扣除。
  const bool dt_valid = std::isfinite(dt) && dt > 0.0 && dt < 1.0;

  if (stationary)
  {
    if (!calibrated())
    {
      // 标定阶段：静止样本的滑动均值即当前零漂，样本数够了就冻结为初值。
      ++calibration_count_;
      for (std::size_t i = 0; i < bias_.size(); ++i)
      {
        calibration_sum_[i] += corrected[i];
        bias_[i] = calibration_sum_[i] / static_cast<double>(calibration_count_);
      }
    }
    else if (config_.bias_time_constant_s > 0.0 && dt_valid)
    {
      // 在线跟踪温漂：EMA 时间常数远大于运动周期，静止误判的影响被稀释。
      const double alpha = dt / std::max(config_.bias_time_constant_s, dt);
      for (std::size_t i = 0; i < bias_.size(); ++i)
        bias_[i] += alpha * (corrected[i] - bias_[i]);
    }
    if (config_.max_bias_rad_s > 0.0)
    {
      for (std::size_t i = 0; i < bias_.size(); ++i)
        bias_[i] = std::clamp(bias_[i], -config_.max_bias_rad_s, config_.max_bias_rad_s);
    }
  }

  Vector3 output{};
  for (std::size_t i = 0; i < output.size(); ++i)
    output[i] = corrected[i] - bias_[i];

  if (config_.lowpass_cutoff_hz > 0.0)
  {
    if (!lowpass_initialized_ || !dt_valid)
    {
      lowpass_state_ = output;
      lowpass_initialized_ = true;
    }
    else
    {
      const double rc = 1.0 / (2.0 * M_PI * config_.lowpass_cutoff_hz);
      const double alpha = dt / (dt + rc);
      for (std::size_t i = 0; i < output.size(); ++i)
        lowpass_state_[i] += alpha * (output[i] - lowpass_state_[i]);
    }
    output = lowpass_state_;
  }
  return output;
}

}  // namespace kiwi_base
