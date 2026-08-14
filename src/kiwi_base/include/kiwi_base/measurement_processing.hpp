/**
 * @file measurement_processing.hpp
 * @brief 提供传感器符号修正、静止死区、陀螺零漂估计与一阶低通处理。
 */
#pragma once

#include <array>
#include <cstddef>

namespace kiwi_base
{

using Vector3 = std::array<double, 3>;

/**
 * 逐轴应用符号和死区：result = sign * raw，绝对值不超过 deadband 时输出零。
 * 非有限输入也输出零，避免异常反馈进入里程计积分或 EKF。
 */
Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband);

/**
 * 逐轴应用符号、死区和比例：先过滤未缩放的静止噪声，再对有效测量应用 scale。
 * scale 只用于传感器量程修正，必须为正数，不能替代坐标符号 sign。
 */
Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband, const Vector3 &scale);

/** 陀螺零漂估计与低通滤波配置；时间量单位为秒，频率为 Hz。 */
struct GyroFilterConfig
{
  /** 初始零漂标定需要的静止样本数；0 表示跳过标定阶段直接在线更新。 */
  std::size_t calibration_samples{0};
  /** 静止时零漂 EMA 时间常数；<= 0 关闭标定完成后的在线更新。 */
  double bias_time_constant_s{0.0};
  /** 零漂绝对值上限；估计值逐轴钳制在 ±max_bias_rad_s 内。 */
  double max_bias_rad_s{0.0};
  /** 一阶低通截止频率；<= 0 关闭低通，测量原样通过。 */
  double lowpass_cutoff_hz{0.0};
};

/**
 * 陀螺角速度零漂估计 + 一阶低通。
 *
 * 输入必须是完成符号/比例修正后的 ROS 系角速度（未做死区）；死区应在
 * 本滤波之后由调用方应用，这样阈值作用在去零漂后的残余噪声上。
 *
 * 零漂只在调用方判定静止时更新：先累计 calibration_samples 个静止样本取
 * 均值作为初始零漂，之后每个静止样本按 EMA 缓慢跟踪温漂。运动样本从不
 * 影响零漂。所有输出均已减去当前零漂估计。
 */
class GyroBiasFilter
{
public:
  explicit GyroBiasFilter(const GyroFilterConfig &config);

  /**
   * 处理一帧测量并返回去零漂、低通后的角速度。
   * @param corrected 符号/比例修正后的角速度，各轴须为有限值。
   * @param stationary 调用方的静止判定；true 时该样本参与零漂估计。
   * @param dt 距上一帧的时间；<= 0 或 >= 1 s 视为无效，跳过 EMA/低通状态推进。
   */
  Vector3 process(const Vector3 &corrected, bool stationary, double dt);

  /** 初始标定是否已完成（calibration_samples 为 0 时视为已完成）。 */
  bool calibrated() const { return calibration_count_ >= config_.calibration_samples; }

  /** 当前零漂估计，供日志与调试输出。 */
  const Vector3 & bias() const { return bias_; }

private:
  GyroFilterConfig config_;
  Vector3 bias_{};
  Vector3 calibration_sum_{};
  std::size_t calibration_count_{0};
  Vector3 lowpass_state_{};
  bool lowpass_initialized_{false};
};

}  // namespace kiwi_base
