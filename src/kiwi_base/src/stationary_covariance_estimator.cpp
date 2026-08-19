/**
 * @file stationary_covariance_estimator.cpp
 * @brief 实现连续静止区间内数值稳定的 IMU 在线方差估计。
 */
#include "kiwi_base/stationary_covariance_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kiwi_base {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

StationaryCovarianceEstimator::StationaryCovarianceEstimator(
    const StationaryCovarianceConfig& config)
    : config_(config) {
  // yaw 线性回归包含截距和斜率两个自由度，至少三个样本才有残差自由度。
  if (config_.minimum_samples < 3)
    throw std::invalid_argument("stationary covariance minimum_samples must be at least 3");
  if (!std::isfinite(config_.yaw_variance_floor) || config_.yaw_variance_floor <= 0.0)
    throw std::invalid_argument("yaw variance floor must be finite and positive");
  if (!std::isfinite(config_.gyro_z_variance_floor) || config_.gyro_z_variance_floor <= 0.0)
    throw std::invalid_argument("gyro_z variance floor must be finite and positive");
}

void StationaryCovarianceEstimator::update(double yaw_rad, double gyro_z_rad_s, bool stationary,
                                           double dt) {
  const bool continuous = std::isfinite(dt) && dt > 0.0 && dt < 1.0;
  if (!stationary || !continuous || !std::isfinite(yaw_rad) || !std::isfinite(gyro_z_rad_s)) {
    // 任一异常值或长断流都使整段观测失去“连续且有效”的保证，不能只跳过一帧。
    reset();
    return;
  }

  if (!have_previous_yaw_) {
    previous_yaw_ = yaw_rad;
    yaw_unwrapped_ = yaw_rad;
    have_previous_yaw_ = true;
  } else {
    // 只用相邻差分完成跨 ±pi 展开；方差本身由下面的去趋势绝对 yaw 计算。
    const double yaw_delta = std::remainder(yaw_rad - previous_yaw_, kTwoPi);
    previous_yaw_ = yaw_rad;
    if (!std::isfinite(yaw_delta)) {
      reset();
      return;
    }
    yaw_unwrapped_ += yaw_delta;
  }

  elapsed_s_ += dt;
  add_sample(yaw_, elapsed_s_, yaw_unwrapped_);
  add_sample(gyro_z_, gyro_z_rad_s);
  if (!std::isfinite(yaw_.mean_time) || !std::isfinite(yaw_.mean_value) ||
      !std::isfinite(yaw_.time_sum_squared_deviation) ||
      !std::isfinite(yaw_.value_sum_squared_deviation) ||
      !std::isfinite(yaw_.time_value_sum_deviation) || !std::isfinite(gyro_z_.mean) ||
      !std::isfinite(gyro_z_.sum_squared_deviation)) {
    // 有限极值仍可能让中间运算溢出；清窗使节点继续使用保守启动方差。
    reset();
  }
}

void StationaryCovarianceEstimator::notify_motion() {
  reset();
}

void StationaryCovarianceEstimator::begin_next_window() {
  reset();
}

bool StationaryCovarianceEstimator::calibrated() const noexcept {
  return yaw_.count >= config_.minimum_samples && gyro_z_.count >= config_.minimum_samples;
}

std::size_t StationaryCovarianceEstimator::sample_count() const noexcept {
  return std::min(yaw_.count, gyro_z_.count);
}

double StationaryCovarianceEstimator::yaw_variance() const noexcept {
  if (yaw_.count < config_.minimum_samples || yaw_.count < 3) return config_.yaw_variance_floor;

  if (yaw_.time_sum_squared_deviation <= 0.0) return config_.yaw_variance_floor;

  // SSE = Syy - Sxy²/Sxx；除以 n-2 是含截距、斜率的一元回归残差样本方差。
  const double explained = yaw_.time_value_sum_deviation * yaw_.time_value_sum_deviation /
                           yaw_.time_sum_squared_deviation;
  double residual_sum_squared = yaw_.value_sum_squared_deviation - explained;
  if (!std::isfinite(residual_sum_squared)) return config_.yaw_variance_floor;
  // 完全线性或浮点舍入时 SSE 可能出现极小负数，按零处理再应用安全下限。
  residual_sum_squared = std::max(0.0, residual_sum_squared);
  const double residual_variance = residual_sum_squared / static_cast<double>(yaw_.count - 2);
  if (!std::isfinite(residual_variance)) return config_.yaw_variance_floor;
  return std::max(residual_variance, config_.yaw_variance_floor);
}

double StationaryCovarianceEstimator::gyro_z_variance() const noexcept {
  return variance_with_floor(gyro_z_, config_.gyro_z_variance_floor);
}

void StationaryCovarianceEstimator::reset() noexcept {
  have_previous_yaw_ = false;
  previous_yaw_ = 0.0;
  yaw_unwrapped_ = 0.0;
  elapsed_s_ = 0.0;
  yaw_ = OnlineLinearRegression{};
  gyro_z_ = OnlineVariance{};
}

void StationaryCovarianceEstimator::add_sample(OnlineVariance& state, double value) noexcept {
  // Welford 算法避免“平方均值减均值平方”在小噪声、大偏置下的消减误差。
  ++state.count;
  const double delta = value - state.mean;
  state.mean += delta / static_cast<double>(state.count);
  const double delta_after_mean = value - state.mean;
  state.sum_squared_deviation += delta * delta_after_mean;
}

void StationaryCovarianceEstimator::add_sample(OnlineLinearRegression& state, double time_s,
                                               double value) noexcept {
  // 二元 Welford/Pébay 更新同时维护 Sxx、Syy、Sxy，避免直接平方和的消减误差。
  ++state.count;
  const double time_delta = time_s - state.mean_time;
  const double value_delta = value - state.mean_value;
  state.mean_time += time_delta / static_cast<double>(state.count);
  state.mean_value += value_delta / static_cast<double>(state.count);
  state.time_sum_squared_deviation += time_delta * (time_s - state.mean_time);
  state.value_sum_squared_deviation += value_delta * (value - state.mean_value);
  state.time_value_sum_deviation += time_delta * (value - state.mean_value);
}

double StationaryCovarianceEstimator::variance_with_floor(const OnlineVariance& state,
                                                          double floor) const noexcept {
  if (state.count < config_.minimum_samples || state.count < 2) return floor;

  const double variance = state.sum_squared_deviation / static_cast<double>(state.count - 1);
  // 浮点舍入可能产生极小负数；非有限结果同样退回安全下限。
  if (!std::isfinite(variance) || variance < 0.0) return floor;
  return std::max(variance, floor);
}

}  // namespace kiwi_base
