/**
 * @file test_measurement_processing.cpp
 * @brief 验证 IMU 符号修正、静止死区、零漂估计与低通不会把噪声送入 EKF。
 */
#include "kiwi_base/measurement_processing.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using kiwi_base::GyroBiasFilter;
using kiwi_base::GyroFilterConfig;
using kiwi_base::Vector3;
using kiwi_base::correct_vector;

TEST(MeasurementProcessing, AppliesAxisSignsAndDeadband)
{
  const auto output = correct_vector(
    Vector3{0.010, -0.020, -0.030},
    Vector3{1.0, 1.0, -1.0},
    Vector3{0.015, 0.015, 0.015});

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], -0.020);
  EXPECT_DOUBLE_EQ(output[2], 0.030);
}

TEST(MeasurementProcessing, ZerosThresholdAndNonFiniteValues)
{
  const auto output = correct_vector(
    Vector3{0.015, std::numeric_limits<double>::quiet_NaN(), -0.015},
    Vector3{1.0, 1.0, -1.0},
    Vector3{0.015, 0.0, 0.015});

  EXPECT_EQ(output, (Vector3{0.0, 0.0, 0.0}));
}

TEST(MeasurementProcessing, AppliesScaleAfterDeadband)
{
  const auto output = correct_vector(
    Vector3{0.010, -0.020, -0.030},
    Vector3{1.0, 1.0, -1.0},
    Vector3{0.015, 0.015, 0.015},
    Vector3{2.0, 3.0, 1.69});

  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_DOUBLE_EQ(output[1], -0.060);
  EXPECT_DOUBLE_EQ(output[2], 0.0507);
}

TEST(GyroBiasFilter, CalibratesBiasFromStationaryMean)
{
  GyroFilterConfig config{};
  config.calibration_samples = 4;
  GyroBiasFilter filter(config);

  const Vector3 biased{0.010, -0.008, 0.012};
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_FALSE(filter.calibrated());
    filter.process(biased, true, 0.02);
  }
  EXPECT_TRUE(filter.calibrated());
  EXPECT_NEAR(filter.bias()[0], 0.010, 1e-12);
  EXPECT_NEAR(filter.bias()[1], -0.008, 1e-12);
  EXPECT_NEAR(filter.bias()[2], 0.012, 1e-12);

  // 标定后静止输出应为零，运动测量只剩真实成分。
  const auto rest = filter.process(biased, true, 0.02);
  EXPECT_NEAR(rest[2], 0.0, 1e-12);
  const auto moving = filter.process(Vector3{0.010, -0.008, 0.512}, false, 0.02);
  EXPECT_NEAR(moving[2], 0.500, 1e-12);
}

TEST(GyroBiasFilter, MovingSamplesNeverUpdateBias)
{
  GyroFilterConfig config{};
  config.calibration_samples = 2;
  config.bias_time_constant_s = 1.0;
  GyroBiasFilter filter(config);

  // 标定阶段的运动样本不计入均值，样本计数也不推进。
  filter.process(Vector3{1.0, 1.0, 1.0}, false, 0.02);
  EXPECT_FALSE(filter.calibrated());
  EXPECT_EQ(filter.bias(), (Vector3{0.0, 0.0, 0.0}));

  filter.process(Vector3{0.01, 0.01, 0.01}, true, 0.02);
  filter.process(Vector3{0.01, 0.01, 0.01}, true, 0.02);
  EXPECT_TRUE(filter.calibrated());

  // 标定完成后运动样本同样不改变零漂估计。
  filter.process(Vector3{2.0, 2.0, 2.0}, false, 0.02);
  EXPECT_NEAR(filter.bias()[0], 0.01, 1e-12);
}

TEST(GyroBiasFilter, TracksSlowDriftWithEmaWhenStationary)
{
  GyroFilterConfig config{};
  config.calibration_samples = 0;  // 跳过标定，直接在线更新。
  config.bias_time_constant_s = 1.0;
  GyroBiasFilter filter(config);
  EXPECT_TRUE(filter.calibrated());

  // alpha = dt / tau = 0.5：两个样本后 bias = 0.02 * (1 - 0.5^2) 的等比逼近。
  filter.process(Vector3{0.0, 0.0, 0.02}, true, 0.5);
  EXPECT_NEAR(filter.bias()[2], 0.010, 1e-12);
  filter.process(Vector3{0.0, 0.0, 0.02}, true, 0.5);
  EXPECT_NEAR(filter.bias()[2], 0.015, 1e-12);

  // 无效 dt 不推进 EMA，只做零漂扣除。
  const auto output = filter.process(Vector3{0.0, 0.0, 0.02}, true, 0.0);
  EXPECT_NEAR(filter.bias()[2], 0.015, 1e-12);
  EXPECT_NEAR(output[2], 0.005, 1e-12);
}

TEST(GyroBiasFilter, ClampsBiasToConfiguredLimit)
{
  GyroFilterConfig config{};
  config.calibration_samples = 1;
  config.max_bias_rad_s = 0.005;
  GyroBiasFilter filter(config);

  filter.process(Vector3{0.1, -0.1, 0.0}, true, 0.02);
  EXPECT_NEAR(filter.bias()[0], 0.005, 1e-12);
  EXPECT_NEAR(filter.bias()[1], -0.005, 1e-12);
}

TEST(GyroBiasFilter, LowpassSmoothsStepAndResetsOnInvalidDt)
{
  GyroFilterConfig config{};
  // rc = 1/(2*pi*fc) = 0.1 s，dt = 0.1 -> alpha = 0.5。
  config.lowpass_cutoff_hz = 1.0 / (0.2 * M_PI);
  GyroBiasFilter filter(config);

  // 首帧初始化滤波状态：输出等于输入。
  auto output = filter.process(Vector3{0.0, 0.0, 0.0}, false, 0.02);
  EXPECT_NEAR(output[2], 0.0, 1e-12);
  // 阶跃输入按 alpha = 0.5 逼近。
  output = filter.process(Vector3{0.0, 0.0, 1.0}, false, 0.1);
  EXPECT_NEAR(output[2], 0.5, 1e-9);
  output = filter.process(Vector3{0.0, 0.0, 1.0}, false, 0.1);
  EXPECT_NEAR(output[2], 0.75, 1e-9);
  // dt 无效（如串口断流恢复）时重置状态，避免用陈旧历史平滑新数据。
  output = filter.process(Vector3{0.0, 0.0, 1.0}, false, 2.0);
  EXPECT_NEAR(output[2], 1.0, 1e-12);
}
