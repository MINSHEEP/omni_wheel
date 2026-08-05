/**
 * @file test_kinematics.cpp
 * @brief 验证当前冻结的 Kiwi 三行固定公式。
 */
#include <gtest/gtest.h>

#include "kiwi_base/kiwi_kinematics.hpp"

namespace
{
using kiwi_base::KiwiKinematics;
using kiwi_base::Twist2D;

TEST(KiwiKinematics, ForwardAndInverseRoundTrip)
{
  KiwiKinematics kinematics;
  const Twist2D input{0.2, -0.1, 0.4};
  const Twist2D output = kinematics.forward(kinematics.inverse(input));

  EXPECT_NEAR(output.vx, input.vx, 1e-10);
  EXPECT_NEAR(output.vy, input.vy, 1e-10);
  EXPECT_NEAR(output.wz, input.wz, 1e-10);
}

TEST(KiwiKinematics, MatchesFixedFormula)
{
  KiwiKinematics kinematics;

  const auto forward = kinematics.inverse({1.0, 0.0, 0.0});
  EXPECT_NEAR(forward[0], 0.866025403784, 1e-10);
  EXPECT_NEAR(forward[1], 0.0, 1e-10);
  EXPECT_NEAR(forward[2], -0.866025403784, 1e-10);

  const auto left = kinematics.inverse({0.0, 1.0, 0.0});
  EXPECT_NEAR(left[0], 0.5, 1e-10);
  EXPECT_NEAR(left[1], -1.0, 1e-10);
  EXPECT_NEAR(left[2], 0.5, 1e-10);

  const auto rotate = kinematics.inverse({0.0, 0.0, 1.0});
  EXPECT_NEAR(rotate[0], -1.0, 1e-10);
  EXPECT_NEAR(rotate[1], -1.0, 1e-10);
  EXPECT_NEAR(rotate[2], -1.0, 1e-10);
}
}  // namespace
