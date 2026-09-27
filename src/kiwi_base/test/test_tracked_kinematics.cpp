#include <gtest/gtest.h>
#include "kiwi_base/tracked_kinematics.hpp"

using kiwi_base::TrackedKinematics;

TEST(TrackedKinematics, ForwardAndTurn)
{
  const TrackedKinematics kinematics(0.5, 10.0, 20.0);
  const auto forward = kinematics.command(0.4, 0.0);
  EXPECT_FLOAT_EQ(forward.first, 4.0F);
  EXPECT_FLOAT_EQ(forward.second, 4.0F);
  const auto turn = kinematics.command(0.0, 1.0);
  EXPECT_FLOAT_EQ(turn.first, -2.5F);
  EXPECT_FLOAT_EQ(turn.second, 2.5F);
}

TEST(TrackedKinematics, PreservesCurvatureWhenLimited)
{
  const TrackedKinematics kinematics(0.5, 10.0, 5.0);
  const auto command = kinematics.command(0.4, 1.0);
  EXPECT_NEAR(command.first, 1.153846F, 1e-5F);
  EXPECT_FLOAT_EQ(command.second, 5.0F);
  const auto invalid = kinematics.command(NAN, 1.0);
  EXPECT_FLOAT_EQ(invalid.first, 0.0F);
  EXPECT_FLOAT_EQ(invalid.second, 0.0F);
}
