/**
 * @file stationary_covariance_estimator.hpp
 * @brief 提供仅基于连续静止窗口的 IMU yaw/gyro_z 方差估计。
 */
#pragma once

#include <cstddef>

namespace kiwi_base {

/** 静止协方差估计配置；方差下限必须是有限正数。 */
struct StationaryCovarianceConfig {
  std::size_t minimum_samples{3};
  double yaw_variance_floor{0.0};
  double gyro_z_variance_floor{0.0};
};

/**
 * 使用数值稳定的在线算法估计 yaw 与 gyro_z 方差。yaw 先跨 ±pi 连续展开，
 * 再对“时间-yaw”做一元线性回归；回归残差方差代表去除线性零漂后的角度
 * 抖动。这样不再假设相邻 yaw 误差彼此独立，避免差分方差严重低估绝对航向
 * 的不确定度。gyro_z 使用 Welford 样本方差。
 *
 * 运动、断流或异常输入会清空当前窗口，确保一次估计中的全部样本来自同一段
 * 连续静止区间。窗口完成后调用方读取结果并调用 begin_next_window()，从而让
 * 每次提交使用互不重叠的观测，避免同一批样本被高频重复加权。
 */
class StationaryCovarianceEstimator {
public:
  explicit StationaryCovarianceEstimator(const StationaryCovarianceConfig& config);

  /**
   * 输入一组同步测量；非静止、非有限值或不连续时间会重置当前窗口。
   * dt 必须位于 (0, 1) 秒，首帧可由调用方传入 0 以显式建立新窗口边界。
   */
  void update(double yaw_rad, double gyro_z_rad_s, bool stationary, double dt);

  /** 命令开始运动时立即清除窗口，不等待下一帧遥测。 */
  void notify_motion();

  /** 当前结果提交后开始一个不重叠的新窗口；不会保存或修改调用方已提交方差。 */
  void begin_next_window();

  bool calibrated() const noexcept;
  std::size_t sample_count() const noexcept;
  double yaw_variance() const noexcept;
  double gyro_z_variance() const noexcept;

private:
  struct OnlineVariance {
    std::size_t count{0};
    double mean{0.0};
    double sum_squared_deviation{0.0};
  };

  /** 在线一元回归所需的中心化二阶矩，避免直接累加大时间戳造成消减误差。 */
  struct OnlineLinearRegression {
    std::size_t count{0};
    double mean_time{0.0};
    double mean_value{0.0};
    double time_sum_squared_deviation{0.0};
    double value_sum_squared_deviation{0.0};
    double time_value_sum_deviation{0.0};
  };

  void reset() noexcept;
  static void add_sample(OnlineVariance& state, double value) noexcept;
  static void add_sample(OnlineLinearRegression& state, double time_s, double value) noexcept;
  double variance_with_floor(const OnlineVariance& state, double floor) const noexcept;

  StationaryCovarianceConfig config_;
  bool have_previous_yaw_{false};
  double previous_yaw_{0.0};
  double yaw_unwrapped_{0.0};
  double elapsed_s_{0.0};
  OnlineLinearRegression yaw_;
  OnlineVariance gyro_z_;
};

}  // namespace kiwi_base
