/**
 * @file test_yaw_drift_estimator.cpp
 * @brief 验证 yaw 展开和连续静止区间零漂估计。
 */
#include "kiwi_base/yaw_drift_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using kiwi_base::YawDriftEstimator;
using kiwi_base::YawDriftEstimatorConfig;

YawDriftEstimatorConfig make_config(std::size_t calibration_samples) {
  YawDriftEstimatorConfig config{};
  config.enabled = true;
  config.time_constant_s = 1.0;
  config.max_rate_rad_s = 1.0;
  config.calibration_samples = calibration_samples;
  return config;
}

TEST(YawDriftEstimator, CalibratesFromSustainedStationarySamples) {
  YawDriftEstimator estimator(make_config(4));

  estimator.update(0.0, true, 0.1);
  for (int sample = 1; sample <= 4; ++sample) estimator.update(0.001 * sample, true, 0.1);

  EXPECT_TRUE(estimator.calibrated());
  EXPECT_NEAR(estimator.drift_rate_rad_s(), 0.01, 1e-12);
}

TEST(YawDriftEstimator, RotationStartAndStopClearsStationaryReference) {
  YawDriftEstimator estimator(make_config(0));

  // 先建立静止参考，随后发生一弧度真实转动。
  estimator.update(0.0, true, 0.1);
  estimator.update(1.0, false, 0.1);

  // 再次静止的首帧只建立新参考；后续仅观察 0.01 rad/s 的真实零漂。
  estimator.update(1.0, true, 0.1);
  for (int sample = 1; sample <= 11; ++sample) estimator.update(1.0 + 0.001 * sample, true, 0.1);

  EXPECT_NEAR(estimator.drift_rate_rad_s(), 0.01, 1e-12);
}

TEST(YawDriftEstimator, MovementDuringCalibrationNeverEntersSampleMean) {
  YawDriftEstimator estimator(make_config(2));

  estimator.update(0.0, true, 0.1);
  estimator.update(0.001, true, 0.1);  // 第一个有效静止样本：0.01 rad/s。
  estimator.update(1.0, false, 0.1);   // 标定中发生真实转动。
  estimator.update(1.0, true, 0.1);    // 只重建参考，不累计跨运动增量。
  estimator.update(1.001, true, 0.1);  // 第二个有效静止样本。

  EXPECT_TRUE(estimator.calibrated());
  EXPECT_NEAR(estimator.drift_rate_rad_s(), 0.01, 1e-12);
}

TEST(YawDriftEstimator, MotionNotificationClearsReferenceWithoutFeedbackFrame) {
  YawDriftEstimator estimator(make_config(0));

  estimator.update(0.0, true, 0.1);
  // 模拟相邻姿态反馈之间完成一次真实转动，运动事件必须主动使旧参考失效。
  estimator.notify_motion();
  estimator.update(1.0, true, 0.1);
  for (int sample = 1; sample <= 11; ++sample) estimator.update(1.0 + 0.001 * sample, true, 0.1);

  EXPECT_NEAR(estimator.drift_rate_rad_s(), 0.01, 1e-12);
}

TEST(YawDriftEstimator, UnwrapsPiBoundaryWhenCompensationDisabled) {
  auto config = make_config(0);
  config.enabled = false;
  YawDriftEstimator estimator(config);

  constexpr double kPi = 3.14159265358979323846;
  estimator.update(kPi - 0.01, false, 0.1);
  const double unwrapped = estimator.update(-kPi + 0.01, false, 0.1);

  EXPECT_NEAR(unwrapped, kPi + 0.01, 1e-12);
}

}  // namespace
