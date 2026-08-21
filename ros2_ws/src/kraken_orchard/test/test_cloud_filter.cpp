// SPDX-License-Identifier: BSD-3-Clause
//
// The sweep is mostly ground and canopy. These check that what reaches the fit
// is the trunks and only the trunks.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "kraken_orchard/cloud_filter.hpp"

namespace
{

const kraken_orchard::CloudFilterOptions kOptions;

/// The sensor rides this far above the ground under it.
constexpr double kMount = -0.966;

double groundAt(double x, double slope)
{
  return kMount + slope * x;
}

void addGround(std::vector<kraken_orchard::Point3> & cloud, double slope = 0.0)
{
  for (double x = -10.0; x <= 12.0; x += 0.1) {
    for (double y = -4.0; y <= 4.0; y += 0.1) {
      cloud.push_back(kraken_orchard::Point3{x, y, groundAt(x, slope)});
    }
  }
}

void addCanopy(std::vector<kraken_orchard::Point3> & cloud, double centre_y, double slope = 0.0)
{
  for (double x = -6.0; x <= 12.0; x += 0.1) {
    for (double height = 2.2; height <= 3.4; height += 0.2) {
      cloud.push_back(kraken_orchard::Point3{x, centre_y, groundAt(x, slope) + height});
    }
  }
}

void addTrunk(
  std::vector<kraken_orchard::Point3> & cloud, double x, double y, double slope = 0.0)
{
  for (double height = 0.5; height <= 1.7; height += 0.15) {
    for (int step = 0; step < 12; ++step) {
      const double angle = step * (2.0 * M_PI / 12.0);
      cloud.push_back(
        kraken_orchard::Point3{
          x + 0.1 * std::cos(angle),
          y + 0.1 * std::sin(angle),
          groundAt(x, slope) + height});
    }
  }
}

std::vector<kraken_orchard::Point3> orchardSweep(double slope = 0.0)
{
  std::vector<kraken_orchard::Point3> cloud;
  addGround(cloud, slope);
  addCanopy(cloud, 1.75, slope);
  addCanopy(cloud, -1.75, slope);
  for (double along = 2.0; along <= 11.0; along += 3.2) {
    addTrunk(cloud, along, 1.75, slope);
    addTrunk(cloud, along, -1.75, slope);
  }
  return cloud;
}

}  // namespace

TEST(CloudFilter, KeepsTheTrunksAndNothingElse)
{
  kraken_orchard::FilterReport report;
  const std::vector<kraken_orchard::Trunk> trunks =
    kraken_orchard::findTrunks(orchardSweep(), kOptions, &report);

  EXPECT_EQ(trunks.size(), 6U);
  EXPECT_GT(report.received, report.above_ground);
  for (const kraken_orchard::Trunk & trunk : trunks) {
    EXPECT_NEAR(std::abs(trunk.centre.y), 1.75, 0.05);
  }
}

TEST(CloudFilter, AHillsideIsStillGround)
{
  // Measured against the sensor rather than the ground beneath it, terrain
  // rising 1 in 10 reaches trunk height 4 m out and swamps the row.
  const std::vector<kraken_orchard::Trunk> trunks =
    kraken_orchard::findTrunks(orchardSweep(0.1), kOptions);

  EXPECT_EQ(trunks.size(), 6U);
  for (const kraken_orchard::Trunk & trunk : trunks) {
    EXPECT_NEAR(std::abs(trunk.centre.y), 1.75, 0.05);
  }
}

TEST(CloudFilter, VoxelisingEvensOutTheRangeBias)
{
  // A near trunk and a far one return wildly different point counts. After the
  // grid they should weigh roughly the same in the fit.
  std::vector<kraken_orchard::Point3> cloud;
  addGround(cloud);
  for (int repeat = 0; repeat < 21; ++repeat) {
    addTrunk(cloud, 2.0, 1.75);
  }
  addTrunk(cloud, 11.0, 1.75);

  const std::vector<kraken_orchard::Trunk> trunks = kraken_orchard::findTrunks(cloud, kOptions);

  ASSERT_EQ(trunks.size(), 2U);
  const bool first_is_near = trunks[0].centre.x < trunks[1].centre.x;
  const int near_points = first_is_near ? trunks[0].points : trunks[1].points;
  const int far_points = first_is_near ? trunks[1].points : trunks[0].points;
  // Twenty one times the raw returns must not buy twenty one times the say.
  EXPECT_LT(near_points, 2 * far_points);
}

TEST(CloudFilter, AFenceIsNotATrunk)
{
  std::vector<kraken_orchard::Point3> cloud;
  addGround(cloud);
  for (double x = -2.0; x <= 3.0; x += 0.1) {
    for (double height = 0.5; height <= 1.5; height += 0.15) {
      cloud.push_back(kraken_orchard::Point3{x, 2.0, kMount + height});
    }
  }
  EXPECT_TRUE(kraken_orchard::findTrunks(cloud, kOptions).empty());
}

TEST(CloudFilter, GroundAloneYieldsNothing)
{
  std::vector<kraken_orchard::Point3> cloud;
  addGround(cloud, 0.1);
  kraken_orchard::FilterReport report;
  EXPECT_TRUE(kraken_orchard::findTrunks(cloud, kOptions, &report).empty());
  EXPECT_EQ(report.above_ground, 0U);
}
