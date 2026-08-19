/**
 * @file test_stationary_covariance_estimator.cpp
 * @brief 验证连续静止方差估计、运动重置和异常输入防御。
 */
#include "kiwi_base/stationary_covariance_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using kiwi_base::StationaryCovarianceConfig;
using kiwi_base::StationaryCovarianceEstimator;

StationaryCovarianceConfig make_config() {
  StationaryCovarianceConfig config{};
  config.minimum_samples = 5;
  config.yaw_variance_floor = 1.0e-6;
  config.gyro_z_variance_floor = 2.0e-6;
  return config;
}

TEST(StationaryCovarianceEstimator, SustainedStationarySamplesRemoveLinearYawDrift) {
  StationaryCovarianceEstimator estimator(make_config());

  // yaw 跨越 +pi/-pi，并含 0.1 rad/s 线性漂移和确定残差；线性项不得计入方差。
  constexpr double kTwoPi = 6.28318530717958647692;
  double yaw = 3.13;
  const double residuals[] = {0.01, -0.01, 0.00, -0.01, 0.01};
  for (std::size_t index = 0; index < 5; ++index) {
    if (index > 0)
      yaw = std::remainder(yaw + 0.01 + residuals[index] - residuals[index - 1], kTwoPi);
    estimator.update(yaw, 1000.0 + 0.1 * static_cast<double>(index), true, 0.1);
  }

  EXPECT_TRUE(estimator.calibrated());
  EXPECT_EQ(estimator.sample_count(), 5U);
  EXPECT_NEAR(estimator.yaw_variance(), 4.0e-4 / 3.0, 1.0e-12);
  EXPECT_NEAR(estimator.gyro_z_variance(), 0.1 / 4.0, 1.0e-12);
}

TEST(StationaryCovarianceEstimator, MotionResetsObservationWindow) {
  StationaryCovarianceEstimator estimator(make_config());
  for (int sample = 0; sample < 5; ++sample)
    estimator.update(0.01 * sample, 0.02 * sample, true, 0.1);
  ASSERT_TRUE(estimator.calibrated());

  estimator.notify_motion();

  EXPECT_FALSE(estimator.calibrated());
  EXPECT_EQ(estimator.sample_count(), 0U);
  EXPECT_DOUBLE_EQ(estimator.yaw_variance(), make_config().yaw_variance_floor);
  EXPECT_DOUBLE_EQ(estimator.gyro_z_variance(), make_config().gyro_z_variance_floor);
}

TEST(StationaryCovarianceEstimator, NonStationaryAndNonFiniteInputsResetWindow) {
  StationaryCovarianceEstimator estimator(make_config());
  estimator.update(0.0, 0.0, true, 0.1);
  estimator.update(0.1, 0.1, true, 0.1);
  estimator.update(0.2, 0.2, true, 0.1);
  ASSERT_EQ(estimator.sample_count(), 3U);

  estimator.update(0.2, 0.2, false, 0.1);
  EXPECT_EQ(estimator.sample_count(), 0U);

  estimator.update(0.0, 0.0, true, 0.1);
  estimator.update(std::numeric_limits<double>::quiet_NaN(), 0.1, true, 0.1);
  EXPECT_EQ(estimator.sample_count(), 0U);

  estimator.update(0.0, std::numeric_limits<double>::infinity(), true, 0.1);
  EXPECT_EQ(estimator.sample_count(), 0U);
  EXPECT_TRUE(std::isfinite(estimator.yaw_variance()));
  EXPECT_TRUE(std::isfinite(estimator.gyro_z_variance()));

  // 极端但有限的输入可能令中间平方和溢出，发布方差仍必须退回有限下限。
  const double maximum = std::numeric_limits<double>::max();
  estimator.update(0.0, maximum, true, 0.1);
  estimator.update(0.0, maximum, true, 0.1);
  estimator.update(0.0, -maximum, true, 0.1);
  EXPECT_FALSE(estimator.calibrated());
  EXPECT_EQ(estimator.sample_count(), 0U);
  EXPECT_TRUE(std::isfinite(estimator.yaw_variance()));
  EXPECT_TRUE(std::isfinite(estimator.gyro_z_variance()));
}

TEST(StationaryCovarianceEstimator, VarianceFloorsPreventOverconfidence) {
  StationaryCovarianceEstimator estimator(make_config());
  for (int sample = 0; sample < 5; ++sample) estimator.update(0.5, -0.2, true, 0.1);

  ASSERT_TRUE(estimator.calibrated());
  EXPECT_DOUBLE_EQ(estimator.yaw_variance(), make_config().yaw_variance_floor);
  EXPECT_DOUBLE_EQ(estimator.gyro_z_variance(), make_config().gyro_z_variance_floor);
}

TEST(StationaryCovarianceEstimator, GapAndCompletedWindowStartFreshObservations) {
  StationaryCovarianceEstimator estimator(make_config());
  for (int sample = 0; sample < 5; ++sample)
    estimator.update(0.01 * sample, 0.02 * sample, true, 0.1);
  ASSERT_TRUE(estimator.calibrated());

  // 结果由调用方保存；估计器只清当前窗口，使下一批样本与上一批完全不重叠。
  estimator.begin_next_window();
  EXPECT_FALSE(estimator.calibrated());
  EXPECT_EQ(estimator.sample_count(), 0U);

  estimator.update(1.0, 0.0, true, 0.1);
  estimator.update(1.1, 0.0, true, 0.1);
  ASSERT_EQ(estimator.sample_count(), 2U);
  // dt>=1 表示断流或时钟异常，跨断流 yaw 跳变不得进入方差窗口。
  estimator.update(-2.0, 0.0, true, 1.0);
  EXPECT_EQ(estimator.sample_count(), 0U);
}

TEST(StationaryCovarianceEstimator, RejectsUnsafeConfiguration) {
  auto config = make_config();
  config.minimum_samples = 2;
  EXPECT_THROW((void)StationaryCovarianceEstimator{config}, std::invalid_argument);

  config = make_config();
  config.yaw_variance_floor = 0.0;
  EXPECT_THROW((void)StationaryCovarianceEstimator{config}, std::invalid_argument);

  config = make_config();
  config.gyro_z_variance_floor = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)StationaryCovarianceEstimator{config}, std::invalid_argument);
}

}  // namespace
