/**
 * @file yaw_drift_estimator.hpp
 * @brief 提供可独立测试的 yaw 展开与静止零漂估计。
 */
#pragma once

#include <cstddef>

namespace kiwi_base {

/** yaw 零漂估计配置；时间量单位为秒，漂移率单位为 rad/s。 */
struct YawDriftEstimatorConfig {
  bool enabled{false};
  double time_constant_s{0.0};
  double max_rate_rad_s{0.0};
  std::size_t calibration_samples{0};
};

/**
 * 展开原始 yaw，并仅用完整静止区间估计和补偿零漂。
 *
 * 运动开始时会立即丢弃静止参考点，因此运动角度不会在下次停车后被误算为漂移。
 * 标定完成后以至少一秒的连续静止区间更新 EMA；已估计的漂移率在运动期间仍会
 * 随时间累计补偿。
 */
class YawDriftEstimator {
public:
  explicit YawDriftEstimator(const YawDriftEstimatorConfig& config);

  /**
   * 处理一帧 yaw。
   * @param yaw_raw 原始包角 yaw，通常位于 [-pi, pi]。
   * @param stationary 当前帧是否已经通过底盘静止确认。
   * @param dt 距上一帧的时间；非有限值、<=0 或 >=1 秒时不推进时间状态。
   * @return 展开并扣除累计零漂后的连续 yaw。
   */
  double update(double yaw_raw, bool stationary, double dt);

  /**
   * 通知估计器底盘已经开始运动。
   * 即使运动发生在相邻两帧姿态反馈之间，也立即丢弃旧静止参考。
   */
  void notify_motion();

  bool calibrated() const {
    return calibrated_;
  }
  double drift_rate_rad_s() const {
    return drift_rate_rad_s_;
  }
  double compensation_rad() const {
    return compensation_rad_;
  }

private:
  void clear_stationary_reference();
  void set_stationary_reference(double yaw_unwrapped);
  void clamp_drift_rate();

  YawDriftEstimatorConfig config_;

  bool have_previous_yaw_{false};
  double previous_yaw_raw_{0.0};
  double yaw_offset_{0.0};
  double last_output_{0.0};

  bool calibrated_{false};
  std::size_t calibration_count_{0};
  double calibration_rate_sum_{0.0};
  double drift_rate_rad_s_{0.0};
  double compensation_rad_{0.0};

  bool have_stationary_reference_{false};
  double stationary_reference_yaw_{0.0};
  double stationary_elapsed_s_{0.0};
};

}  // namespace kiwi_base
