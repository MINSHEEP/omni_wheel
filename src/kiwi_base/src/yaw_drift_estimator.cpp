/**
 * @file yaw_drift_estimator.cpp
 * @brief 实现 yaw 展开和只基于连续静止区间的零漂补偿。
 */
#include "kiwi_base/yaw_drift_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kiwi_base {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDriftObservationIntervalS = 1.0;

}  // namespace

YawDriftEstimator::YawDriftEstimator(const YawDriftEstimatorConfig& config)
    : config_(config), calibrated_(config.calibration_samples == 0) {
  if (!std::isfinite(config_.time_constant_s) || config_.time_constant_s <= 0.0) {
    throw std::invalid_argument("yaw drift time constant must be finite and positive");
  }
  if (!std::isfinite(config_.max_rate_rad_s) || config_.max_rate_rad_s <= 0.0) {
    throw std::invalid_argument("yaw drift max rate must be finite and positive");
  }
}

double YawDriftEstimator::update(double yaw_raw, bool stationary, double dt) {
  // 协议层通常已拒绝 NaN；此处仍保留最后输出，避免异常值污染展开状态。
  if (!std::isfinite(yaw_raw)) {
    clear_stationary_reference();
    return last_output_;
  }

  if (have_previous_yaw_) {
    const double delta = yaw_raw - previous_yaw_raw_;
    if (delta > kPi) {
      yaw_offset_ -= kTwoPi;
    } else if (delta < -kPi) {
      yaw_offset_ += kTwoPi;
    }
  }
  const double yaw_unwrapped = yaw_raw + yaw_offset_;
  previous_yaw_raw_ = yaw_raw;
  have_previous_yaw_ = true;

  if (!config_.enabled) {
    clear_stationary_reference();
    last_output_ = yaw_unwrapped;
    return last_output_;
  }

  const bool dt_valid = std::isfinite(dt) && dt > 0.0 && dt < 1.0;

  // 已知漂移在运动期间同样继续累积，但运动数据绝不参与漂移率估计。
  if (calibrated_ && dt_valid) {
    compensation_rad_ += drift_rate_rad_s_ * dt;
  }

  if (!stationary) {
    // 运动一开始就清除参考点，阻断跨越运动阶段的错误观测区间。
    clear_stationary_reference();
    last_output_ = yaw_unwrapped - compensation_rad_;
    return last_output_;
  }

  if (!have_stationary_reference_ || !dt_valid) {
    set_stationary_reference(yaw_unwrapped);
    last_output_ = yaw_unwrapped - compensation_rad_;
    return last_output_;
  }

  if (!calibrated_) {
    // 相邻两帧都已确认静止，当前增量才可进入初始标定。
    const double instant_rate = (yaw_unwrapped - stationary_reference_yaw_) / dt;
    calibration_rate_sum_ += instant_rate;
    ++calibration_count_;
    set_stationary_reference(yaw_unwrapped);

    if (calibration_count_ >= config_.calibration_samples) {
      drift_rate_rad_s_ = calibration_rate_sum_ / static_cast<double>(calibration_count_);
      clamp_drift_rate();
      calibrated_ = true;
    }
  } else {
    stationary_elapsed_s_ += dt;
    if (stationary_elapsed_s_ >= kDriftObservationIntervalS) {
      // 参考点只可能在连续静止期间存活，因此整个 EMA 观测区间均为静止。
      const double observed_rate =
          (yaw_unwrapped - stationary_reference_yaw_) / stationary_elapsed_s_;
      const double alpha = std::min(stationary_elapsed_s_ / config_.time_constant_s, 1.0);
      drift_rate_rad_s_ = (1.0 - alpha) * drift_rate_rad_s_ + alpha * observed_rate;
      clamp_drift_rate();
      set_stationary_reference(yaw_unwrapped);
    }
  }

  last_output_ = yaw_unwrapped - compensation_rad_;
  return last_output_;
}

void YawDriftEstimator::notify_motion() {
  clear_stationary_reference();
}

void YawDriftEstimator::clear_stationary_reference() {
  have_stationary_reference_ = false;
  stationary_elapsed_s_ = 0.0;
}

void YawDriftEstimator::set_stationary_reference(double yaw_unwrapped) {
  have_stationary_reference_ = true;
  stationary_reference_yaw_ = yaw_unwrapped;
  stationary_elapsed_s_ = 0.0;
}

void YawDriftEstimator::clamp_drift_rate() {
  drift_rate_rad_s_ =
      std::clamp(drift_rate_rad_s_, -config_.max_rate_rad_s, config_.max_rate_rad_s);
}

}  // namespace kiwi_base
