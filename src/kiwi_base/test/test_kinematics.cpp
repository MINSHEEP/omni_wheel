/**
 * @file test_kinematics.cpp
 * @brief 验证带实车轮径和轮心距的 Kiwi 正逆运动学。
 */
#include <gtest/gtest.h>

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
