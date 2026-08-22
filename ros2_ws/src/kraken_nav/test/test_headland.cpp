// SPDX-License-Identifier: BSD-3-Clause
//
// The reference values below are the ones the Python implementation this
// replaced produced, and that implementation is the one that drove seven legs
// of a four-aisle coverage run without a single Nav2 complaint. They are
// asserted to a micrometre because the port was meant to be exact: if a
// manoeuvre here differs from the one that was driven, that is a regression and
// not a refinement.

#include <cmath>

#include <gtest/gtest.h>

#include "kraken_nav/headland.hpp"

using kraken_nav::headland::Footprint;

namespace
{

// The footprint the costmap is given, rear axle at the origin.
const Footprint kKraken{{2.5, 0.45}, {2.5, -0.45}, {-0.6, -0.45}, {-0.6, 0.45}};
const double kMinRadius = 2.2 / std::tan(0.7);

double pathLength(const std::vector<kraken_nav::headland::Segment> & segments)
{
  double total = 0.0;
  for (const auto & segment : segments) {
    total += segment.length;
  }
  return total;
}

}  // namespace

TEST(Headland, SkipIsATurningDiameterInRows)
{
  EXPECT_NEAR(kMinRadius, 2.611932031, 1e-9);
  EXPECT_EQ(kraken_nav::headland::skipFor(kMinRadius, 3.5), 2);
  // Rows wide enough to turn between directly need no skip at all.
  EXPECT_EQ(kraken_nav::headland::skipFor(kMinRadius, 6.0), 1);
}

TEST(Headland, ShallowHeadlandStillTakesTheUTurn)
{
  auto turn = kraken_nav::headland::planTurn(7.0, kMinRadius, 6.4, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "u-turn");
  EXPECT_NEAR(turn->radius, 3.500000, 1e-6);
  EXPECT_NEAR(turn->depth, 4.674579, 1e-6);
  EXPECT_NEAR(pathLength(turn->segments), 10.995574, 1e-6);
  EXPECT_EQ(turn->segments.size(), 3u);
}

TEST(Headland, DeepHeadlandChangesNothingForAUTurn)
{
  auto turn = kraken_nav::headland::planTurn(7.0, kMinRadius, 17.2, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "u-turn");
  EXPECT_NEAR(turn->radius, 3.500000, 1e-6);
  EXPECT_NEAR(turn->depth, 4.674579, 1e-6);
}

TEST(Headland, TurningRight)
{
  auto turn = kraken_nav::headland::planTurn(-7.0, kMinRadius, 6.4, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "u-turn");
  EXPECT_NEAR(turn->radius, 3.500000, 1e-6);
  EXPECT_LT(turn->segments.front().curvature, 0.0);
}

TEST(Headland, NeighbouringRowNeedsTheBulb)
{
  auto turn = kraken_nav::headland::planTurn(3.5, kMinRadius, 17.4, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "bulb");
  EXPECT_NEAR(turn->radius, 3.526108, 1e-6);
  EXPECT_NEAR(turn->depth, 10.819108, 1e-6);
  EXPECT_EQ(turn->segments.size(), 2u);
}

TEST(Headland, ATighterBulbForALessDeepHeadland)
{
  auto turn = kraken_nav::headland::planTurn(3.5, kMinRadius, 8.2, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "bulb");
  EXPECT_NEAR(turn->radius, 2.676108, 1e-6);
  EXPECT_NEAR(turn->depth, 8.052069, 1e-6);
}

TEST(Headland, TooShallowForABulbMeansReversing)
{
  auto turn = kraken_nav::headland::planTurn(3.5, kMinRadius, 4.2, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "3-point");
  EXPECT_NEAR(turn->radius, 3.526108, 1e-6);
  EXPECT_NEAR(turn->depth, 4.100218, 1e-6);
  ASSERT_EQ(turn->segments.size(), 3u);
  EXPECT_FALSE(turn->segments[0].reverse);
  EXPECT_TRUE(turn->segments[1].reverse);
  EXPECT_FALSE(turn->segments[2].reverse);
  // The middle arc backs the machine round, so its wheels are set the other way.
  EXPECT_LT(turn->segments[0].curvature * turn->segments[1].curvature, 0.0);
}

TEST(Headland, ATightUTurnBeatsAThreePointWhenTheRowIsFarEnough)
{
  auto turn = kraken_nav::headland::planTurn(7.0, kMinRadius, 4.2, kKraken);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->name, "u-turn");
  EXPECT_NEAR(turn->radius, 2.900000, 1e-6);
  EXPECT_NEAR(turn->depth, 4.179916, 1e-6);
}

TEST(Headland, SomeHeadlandsAreSimplyTooShallow)
{
  EXPECT_FALSE(kraken_nav::headland::planTurn(7.0, kMinRadius, 3.2, kKraken).has_value());
}

TEST(Headland, EveryTurnLandsTheRowItWasAimedAt)
{
  for (double offset : {3.5, 7.0, 10.5, -7.0}) {
    auto turn = kraken_nav::headland::planTurn(offset, kMinRadius, 18.0, kKraken);
    ASSERT_TRUE(turn.has_value()) << "no turn for offset " << offset;
    const auto end = kraken_nav::headland::trace(turn->segments).back();
    EXPECT_NEAR(end.y, offset, 0.02) << "offset " << offset << " landed short";
    // Antiparallel: the machine comes out of the manoeuvre facing back down the row.
    EXPECT_NEAR(std::cos(end.heading), -1.0, 1e-3) << "offset " << offset << " ended askew";
  }
}

TEST(Headland, NoTurnIsPlannedAtFullLock)
{
  // A turn driven at full lock has nothing left to correct with, so the ladder
  // must never hand back the machine's own tightest radius while a wider one
  // still fits. This is the bug that ran a bulb 0.6 m wide in the field.
  for (double depth : {6.0, 8.0, 12.0, 18.0}) {
    auto turn = kraken_nav::headland::planTurn(7.0, kMinRadius, depth, kKraken);
    ASSERT_TRUE(turn.has_value());
    EXPECT_GT(turn->radius, kMinRadius * 1.05) << "depth " << depth << " drove at full lock";
  }
}
