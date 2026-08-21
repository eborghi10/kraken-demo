// SPDX-License-Identifier: BSD-3-Clause
//
// Ground truth comes from the level: 19 lines of apple trees 3.5 m apart, with
// a RowSpawnPoint on the centreline of each of the 18 aisles between them, and
// trunks every 3.2 m along a line.

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "kraken_orchard/row_fit.hpp"

namespace
{

constexpr double kPitch = 3.5;
constexpr double kHalf = kPitch / 2.0;
constexpr double kSpacing = 3.2;

/// Trunks as the sensor sees them, given where the robot stands in the aisle.
std::vector<kraken_orchard::Trunk> aisle(
  double offset = 0.0,
  double heading = 0.0,
  const std::vector<double> & lines = {kHalf, -kHalf})
{
  std::vector<kraken_orchard::Trunk> trunks;
  for (const double line : lines) {
    for (double along = -kSpacing; along <= 16.0; along += kSpacing) {
      const double across = line - offset;
      trunks.push_back(
        kraken_orchard::Trunk{
          kraken_orchard::Point2{
            along * std::cos(heading) + across * std::sin(heading),
            -along * std::sin(heading) + across * std::cos(heading)},
          20});
    }
  }
  return trunks;
}

}  // namespace

TEST(RowFit, CentredAndAlignedMeasuresTheLevel)
{
  const kraken_orchard::RowFit fit = kraken_orchard::fitRow(aisle());
  ASSERT_TRUE(fit.valid) << fit.reason;
  EXPECT_NEAR(fit.width, kPitch, 1e-6);
  EXPECT_NEAR(fit.lateral_offset, 0.0, 1e-6);
  EXPECT_NEAR(fit.row_heading, 0.0, 1e-6);
}

TEST(RowFit, OffsetRobotReportsTheCentreOnItsOtherSide)
{
  const kraken_orchard::RowFit fit = kraken_orchard::fitRow(aisle(0.6));
  ASSERT_TRUE(fit.valid) << fit.reason;
  EXPECT_NEAR(fit.width, kPitch, 1e-6);
  EXPECT_NEAR(fit.lateral_offset, -0.6, 1e-6);
}

TEST(RowFit, YawedRobotReportsTheRowRunningTheOtherWay)
{
  const kraken_orchard::RowFit fit = kraken_orchard::fitRow(aisle(0.0, 0.12));
  ASSERT_TRUE(fit.valid) << fit.reason;
  EXPECT_NEAR(fit.width, kPitch, 1e-6);
  EXPECT_NEAR(fit.row_heading, -0.12, 1e-6);
}

TEST(RowFit, OneWallOfTrunksIsNotACorridor)
{
  const kraken_orchard::RowFit fit = kraken_orchard::fitRow(aisle(0.0, 0.0, {kHalf}));
  EXPECT_FALSE(fit.valid);
  EXPECT_NE(fit.reason.find("right"), std::string::npos) << fit.reason;
}

TEST(RowFit, TheNextRowOverDoesNotWidenTheMeasurement)
{
  const kraken_orchard::RowFit fit = kraken_orchard::fitRow(
    aisle(0.0, 0.0, {kHalf, -kHalf, kHalf + kPitch, -kHalf - kPitch}));
  ASSERT_TRUE(fit.valid) << fit.reason;
  EXPECT_NEAR(fit.width, kPitch, 1e-6);
}
