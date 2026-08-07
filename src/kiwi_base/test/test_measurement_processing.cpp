/**
 * @file test_measurement_processing.cpp
 * @brief 验证 IMU 符号修正与静止死区不会把噪声送入 EKF。
 */
#include "kiwi_base/measurement_processing.hpp"

#include <gtest/gtest.h>

#include <limits>

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
