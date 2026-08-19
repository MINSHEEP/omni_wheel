/**
 * @file test_kinematics.cpp
 * @brief 验证带实车轮径和轮心距的 Kiwi 正逆运动学。
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "kiwi_base/kiwi_kinematics.hpp"

namespace
{
using kiwi_base::KiwiKinematics;
using kiwi_base::Twist2D;
using kiwi_base::limit_twist;
constexpr double kWheelRadius = 0.1015;
constexpr double kCenterDistance = 0.32256;
constexpr double kSin60 = 0.866025403784;

TEST(KiwiKinematics, ForwardAndInverseRoundTrip)
{
  KiwiKinematics kinematics(kWheelRadius, kCenterDistance);
  const Twist2D input{0.2, -0.1, 0.4};
  const Twist2D output = kinematics.forward(kinematics.inverse(input));

  EXPECT_NEAR(output.vx, input.vx, 1e-10);
  EXPECT_NEAR(output.vy, input.vy, 1e-10);
  EXPECT_NEAR(output.wz, input.wz, 1e-10);
}

TEST(KiwiKinematics, MatchesPhysicalBasisMotions)
{
  KiwiKinematics kinematics(kWheelRadius, kCenterDistance);

  const auto forward = kinematics.inverse({1.0, 0.0, 0.0});
  EXPECT_NEAR(forward[0], kSin60 / kWheelRadius, 1e-10);
  EXPECT_NEAR(forward[1], 0.0, 1e-10);
  EXPECT_NEAR(forward[2], -kSin60 / kWheelRadius, 1e-10);

  const auto left = kinematics.inverse({0.0, 1.0, 0.0});
  EXPECT_NEAR(left[0], 0.5 / kWheelRadius, 1e-10);
  EXPECT_NEAR(left[1], -1.0 / kWheelRadius, 1e-10);
  EXPECT_NEAR(left[2], 0.5 / kWheelRadius, 1e-10);

  const auto rotate = kinematics.inverse({0.0, 0.0, 1.0});
  const double yaw_wheel_speed = -kCenterDistance / kWheelRadius;
  EXPECT_NEAR(rotate[0], yaw_wheel_speed, 1e-10);
  EXPECT_NEAR(rotate[1], yaw_wheel_speed, 1e-10);
  EXPECT_NEAR(rotate[2], yaw_wheel_speed, 1e-10);
}

TEST(KiwiKinematics, UsesConfiguredGeometry)
{
  KiwiKinematics kinematics(0.2, 0.3);
  const auto wheel_speed = kinematics.inverse({1.0, 0.0, 1.0});

  EXPECT_NEAR(wheel_speed[0], (kSin60 - 0.3) / 0.2, 1e-10);
  EXPECT_NEAR(wheel_speed[1], -0.3 / 0.2, 1e-10);
  EXPECT_NEAR(wheel_speed[2], (-kSin60 - 0.3) / 0.2, 1e-10);
}

TEST(KiwiKinematics, ForwardRestoresPositiveXDirection)
{
  KiwiKinematics kinematics(kWheelRadius, kCenterDistance);
  const auto wheel_speed = std::array<double, 3>{kSin60 / kWheelRadius, 0.0,
    -kSin60 / kWheelRadius};

  const Twist2D output = kinematics.forward(wheel_speed);
  EXPECT_NEAR(output.vx, 1.0, 1e-10);
  EXPECT_NEAR(output.vy, 0.0, 1e-10);
  EXPECT_NEAR(output.wz, 0.0, 1e-10);
}

TEST(KiwiKinematics, RejectsInvalidGeometry)
{
  EXPECT_THROW(KiwiKinematics(0.0, kCenterDistance), std::invalid_argument);
  EXPECT_THROW(KiwiKinematics(kWheelRadius, -1.0), std::invalid_argument);
  EXPECT_THROW(KiwiKinematics(kWheelRadius, kCenterDistance, 0.0), std::invalid_argument);
  EXPECT_THROW(
    KiwiKinematics(
      kWheelRadius, kCenterDistance, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(KiwiKinematics, ForwardRejectsNonFiniteWheelSpeed)
{
  KiwiKinematics kinematics(kWheelRadius, kCenterDistance, 8.0);
  const auto twist = kinematics.forward(
    {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0});

  EXPECT_DOUBLE_EQ(twist.vx, 0.0);
  EXPECT_DOUBLE_EQ(twist.vy, 0.0);
  EXPECT_DOUBLE_EQ(twist.wz, 0.0);
}

TEST(KiwiKinematics, ScalesWheelSpeedsAndTwistByTheSameFactor)
{
  constexpr double kWheelSpeedLimit = 8.0;
  KiwiKinematics unlimited(kWheelRadius, kCenterDistance);
  KiwiKinematics limited(kWheelRadius, kCenterDistance, kWheelSpeedLimit);
  const Twist2D input{0.8, 0.8, -1.5};
  const auto raw_wheels = unlimited.inverse(input);
  const double raw_peak = std::max({
    std::abs(raw_wheels[0]), std::abs(raw_wheels[1]), std::abs(raw_wheels[2])});
  const double expected_scale = kWheelSpeedLimit / raw_peak;

  const auto result = limited.inverse_with_twist(input);
  EXPECT_NEAR(
    std::max({std::abs(result.wheel_rad_s[0]), std::abs(result.wheel_rad_s[1]),
      std::abs(result.wheel_rad_s[2])}),
    kWheelSpeedLimit, 1e-12);
  for (std::size_t i = 0; i < raw_wheels.size(); ++i)
    EXPECT_NEAR(result.wheel_rad_s[i], raw_wheels[i] * expected_scale, 1e-12);
  EXPECT_NEAR(result.achievable_twist.vx, input.vx * expected_scale, 1e-12);
  EXPECT_NEAR(result.achievable_twist.vy, input.vy * expected_scale, 1e-12);
  EXPECT_NEAR(result.achievable_twist.wz, input.wz * expected_scale, 1e-12);

  // 兼容接口与带 Twist 的接口必须走同一条饱和路径。
  EXPECT_EQ(limited.inverse(input), result.wheel_rad_s);
  const auto restored = limited.forward(result.wheel_rad_s);
  EXPECT_NEAR(restored.vx, result.achievable_twist.vx, 1e-10);
  EXPECT_NEAR(restored.vy, result.achievable_twist.vy, 1e-10);
  EXPECT_NEAR(restored.wz, result.achievable_twist.wz, 1e-10);
}

TEST(KiwiKinematics, ExtremeFiniteTwistCannotProduceNonFiniteOutput)
{
  constexpr double kWheelSpeedLimit = 8.0;
  KiwiKinematics kinematics(kWheelRadius, kCenterDistance, kWheelSpeedLimit);
  const Twist2D input{std::numeric_limits<double>::max(), 0.0, 0.0};

  const auto result = kinematics.inverse_with_twist(input);
  for (double wheel_speed : result.wheel_rad_s)
    EXPECT_TRUE(std::isfinite(wheel_speed));
  EXPECT_NEAR(result.wheel_rad_s[0], kWheelSpeedLimit, 1e-12);
  EXPECT_NEAR(result.wheel_rad_s[1], 0.0, 1e-12);
  EXPECT_NEAR(result.wheel_rad_s[2], -kWheelSpeedLimit, 1e-12);
  EXPECT_TRUE(std::isfinite(result.achievable_twist.vx));
  EXPECT_TRUE(std::isfinite(result.achievable_twist.vy));
  EXPECT_TRUE(std::isfinite(result.achievable_twist.wz));
}

TEST(VelocityLimit, LimitsEachAxisAndKeepsValuesInsideRange)
{
  const Twist2D limits{0.6, 0.4, 1.5};
  const Twist2D limited = limit_twist({1.0, -0.2, -2.0}, limits);

  EXPECT_DOUBLE_EQ(limited.vx, 0.6);
  EXPECT_DOUBLE_EQ(limited.vy, -0.2);
  EXPECT_DOUBLE_EQ(limited.wz, -1.5);
}

TEST(VelocityLimit, ZerosNonFiniteInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Twist2D limited = limit_twist({nan, 0.2, nan}, {0.6, 0.6, 1.5});

  EXPECT_DOUBLE_EQ(limited.vx, 0.0);
  EXPECT_DOUBLE_EQ(limited.vy, 0.2);
  EXPECT_DOUBLE_EQ(limited.wz, 0.0);
}
}  // namespace
