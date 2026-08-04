#include <gtest/gtest.h>
#include "kiwi_base/kiwi_kinematics.hpp"
#include <cmath>
#include <limits>
using kiwi_base::KiwiKinematics; using kiwi_base::WheelGeometry; using kiwi_base::Twist2D;
TEST(KiwiKinematics, SymmetricRoundTrip){std::array<WheelGeometry,3>w{{{0,0.2,0,0.05,1},{-0.1732,-0.1,2.0943951,0.05,1},{0.1732,-0.1,4.1887902,0.05,1}}};KiwiKinematics k(w);ASSERT_TRUE(k.valid())<<k.error();Twist2D in{0.2,-0.1,0.4};auto out=k.forward(k.inverse(in));EXPECT_NEAR(out.vx,in.vx,1e-8);EXPECT_NEAR(out.vy,in.vy,1e-8);EXPECT_NEAR(out.wz,in.wz,1e-8);}
TEST(KiwiKinematics, RejectsBadRadius){std::array<WheelGeometry,3>w{};KiwiKinematics k(w);EXPECT_FALSE(k.valid());}

TEST(KiwiKinematics, RejectsNonFiniteGeometry)
{
  std::array<WheelGeometry, 3> wheels{{
    {0.0, 0.2, 0.0, 0.05, 1.0},
    {-0.1732, -0.1, 2.0943951, 0.05, 1.0},
    {0.1732, -0.1, std::numeric_limits<double>::quiet_NaN(), 0.05, 1.0}}};
  KiwiKinematics k(wheels);
  EXPECT_FALSE(k.valid());
}

TEST(KiwiKinematics, MatchesProvidedKiwiGeometry)
{
  // 与 config/kiwi_kinematics.yaml 一致：轮1在 +y，L=0.32256 m，r=0.1015 m。
  constexpr double l = 0.32256;
  constexpr double r = 0.1015;
  const double pi = std::acos(-1.0);
  const double y = 0.5 * l;
  const double x = std::sqrt(3.0) * 0.5 * l;
  std::array<WheelGeometry, 3> wheels{{
    {0.0, l, 0.0, r, 1.0},
    {-x, -y, 2.0 * pi / 3.0, r, 1.0},
    {x, -y, 4.0 * pi / 3.0, r, 1.0}}};
  KiwiKinematics k(wheels);
  ASSERT_TRUE(k.valid()) << k.error();

  const auto forward = k.inverse({1.0, 0.0, 0.0});
  EXPECT_NEAR(forward[0], 1.0 / r, 1e-8);
  EXPECT_NEAR(forward[1], -0.5 / r, 1e-8);
  EXPECT_NEAR(forward[2], -0.5 / r, 1e-8);

  const auto lateral = k.inverse({0.0, 1.0, 0.0});
  EXPECT_NEAR(lateral[0], 0.0, 1e-8);
  EXPECT_NEAR(lateral[1], std::sqrt(3.0) * 0.5 / r, 1e-8);
  EXPECT_NEAR(lateral[2], -std::sqrt(3.0) * 0.5 / r, 1e-8);

  const auto rotate = k.inverse({0.0, 0.0, 1.0});
  EXPECT_NEAR(rotate[0], -l / r, 1e-8);
  EXPECT_NEAR(rotate[1], -l / r, 1e-8);
  EXPECT_NEAR(rotate[2], -l / r, 1e-8);
}
