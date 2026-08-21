// SPDX-License-Identifier: BSD-3-Clause
#ifndef KRAKEN_ORCHARD__CLOUD_FILTER_HPP_
#define KRAKEN_ORCHARD__CLOUD_FILTER_HPP_

#include <cstddef>
#include <vector>

namespace kraken_orchard
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// What survives each stage, so a bad fit can be blamed on the right one.
struct FilterReport
{
  std::size_t received{0};
  std::size_t above_ground{0};
  std::size_t voxels{0};
  std::size_t clusters{0};
  std::size_t trunks{0};
};

struct CloudFilterOptions
{
  double ground_cell{0.5};
  double min_trunk_height{0.40};
  double max_trunk_height{1.80};
  double min_range{1.0};
  double max_range{14.0};
  double voxel{0.10};
  double cluster_distance{0.35};
  double max_trunk_width{0.60};
  double min_trunk_extent{0.20};
  std::size_t min_cluster_points{4};
};

struct Trunk
{
  Point2 centre;
  int points{0};
};

/// Reduce a sweep to the trunks standing in it.
///
/// A high resolution sweep is mostly ground and canopy, and what is left is
/// weighted by range: the nearest trunk returns an order of magnitude more
/// points than one at the end of the row, and a line fitted to raw returns
/// pivots about it. Each stage here exists to take that bias out.
std::vector<Trunk> findTrunks(
  const std::vector<Point3> & cloud,
  const CloudFilterOptions & options,
  FilterReport * report = nullptr);

}  // namespace kraken_orchard

#endif  // KRAKEN_ORCHARD__CLOUD_FILTER_HPP_
