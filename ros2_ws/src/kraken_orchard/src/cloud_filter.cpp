// SPDX-License-Identifier: BSD-3-Clause
#include "kraken_orchard/cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace kraken_orchard
{
namespace
{

constexpr std::int64_t kGridOffset = 1 << 20;
constexpr std::uint64_t kCellMask = (1ULL << 21) - 1ULL;

std::int64_t cellIndex(double value, double size)
{
  return static_cast<std::int64_t>(std::floor(value / size));
}

std::uint64_t cellKey(std::int64_t i, std::int64_t j, std::int64_t k)
{
  const std::uint64_t a = static_cast<std::uint64_t>(i + kGridOffset) & kCellMask;
  const std::uint64_t b = static_cast<std::uint64_t>(j + kGridOffset) & kCellMask;
  const std::uint64_t c = static_cast<std::uint64_t>(k + kGridOffset) & kCellMask;
  return (a << 42) | (b << 21) | c;
}

struct VoxelSum
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  int count{0};
};

/// Ground and canopy, dropped by height above the ground directly below.
///
/// The orchard sits on rolling terrain and the robot pitches as it drives, so
/// height above the sensor is not height above the ground: measured that way a
/// hillside 10 m ahead reads the same as a trunk. Take the ground locally
/// instead, as the lowest return around each point.
std::vector<Point3> aboveGround(const std::vector<Point3> & cloud, const CloudFilterOptions & o)
{
  std::vector<Point3> near;
  near.reserve(cloud.size());
  for (const Point3 & p : cloud) {
    const double range = std::hypot(p.x, p.y);
    if (range >= o.min_range && range <= o.max_range) {
      near.push_back(p);
    }
  }

  std::unordered_map<std::uint64_t, double> floor_height;
  floor_height.reserve(near.size());
  for (const Point3 & p : near) {
    const std::uint64_t key =
      cellKey(cellIndex(p.x, o.ground_cell), cellIndex(p.y, o.ground_cell), 0);
    const auto found = floor_height.find(key);
    if (found == floor_height.end()) {
      floor_height.emplace(key, p.z);
    } else {
      found->second = std::min(found->second, p.z);
    }
  }

  std::vector<Point3> kept;
  for (const Point3 & p : near) {
    const std::int64_t i = cellIndex(p.x, o.ground_cell);
    const std::int64_t j = cellIndex(p.y, o.ground_cell);
    // The cell a trunk stands in may hold nothing but trunk, so the ground
    // comes from the neighbourhood.
    double ground = p.z;
    for (std::int64_t di = -1; di <= 1; ++di) {
      for (std::int64_t dj = -1; dj <= 1; ++dj) {
        const auto found = floor_height.find(cellKey(i + di, j + dj, 0));
        if (found != floor_height.end()) {
          ground = std::min(ground, found->second);
        }
      }
    }
    const double height = p.z - ground;
    if (height >= o.min_trunk_height && height <= o.max_trunk_height) {
      kept.push_back(p);
    }
  }
  return kept;
}

/// One point per voxel, so a trunk counts the same near as far.
std::vector<Point3> voxelise(const std::vector<Point3> & cloud, double size)
{
  std::unordered_map<std::uint64_t, VoxelSum> voxels;
  voxels.reserve(cloud.size());
  for (const Point3 & p : cloud) {
    VoxelSum & sum = voxels[cellKey(
        cellIndex(p.x, size), cellIndex(p.y, size), cellIndex(p.z, size))];
    sum.x += p.x;
    sum.y += p.y;
    sum.z += p.z;
    ++sum.count;
  }
  std::vector<Point3> centres;
  centres.reserve(voxels.size());
  for (const auto & entry : voxels) {
    const VoxelSum & sum = entry.second;
    const double n = static_cast<double>(sum.count);
    centres.push_back(Point3{sum.x / n, sum.y / n, sum.z / n});
  }
  return centres;
}

struct Cluster
{
  std::vector<std::size_t> members;
};

/// Single linkage in the ground plane: a trunk is a column of returns with
/// clear air around it, and clear air is what the orchard has plenty of.
std::vector<Cluster> cluster(const std::vector<Point3> & points, double distance)
{
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> grid;
  grid.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    grid[cellKey(cellIndex(points[i].x, distance), cellIndex(points[i].y, distance), 0)]
      .push_back(i);
  }

  const double limit = distance * distance;
  std::vector<bool> taken(points.size(), false);
  std::vector<Cluster> clusters;
  std::vector<std::size_t> queue;

  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (taken[seed]) {
      continue;
    }
    taken[seed] = true;
    queue.assign(1, seed);
    Cluster grown;
    while (!queue.empty()) {
      const std::size_t current = queue.back();
      queue.pop_back();
      grown.members.push_back(current);
      const std::int64_t ci = cellIndex(points[current].x, distance);
      const std::int64_t cj = cellIndex(points[current].y, distance);
      for (std::int64_t di = -1; di <= 1; ++di) {
        for (std::int64_t dj = -1; dj <= 1; ++dj) {
          const auto cell = grid.find(cellKey(ci + di, cj + dj, 0));
          if (cell == grid.end()) {
            continue;
          }
          for (const std::size_t other : cell->second) {
            if (taken[other]) {
              continue;
            }
            const double dx = points[other].x - points[current].x;
            const double dy = points[other].y - points[current].y;
            if (dx * dx + dy * dy <= limit) {
              taken[other] = true;
              queue.push_back(other);
            }
          }
        }
      }
    }
    clusters.push_back(std::move(grown));
  }
  return clusters;
}

}  // namespace

std::vector<Trunk> findTrunks(
  const std::vector<Point3> & cloud,
  const CloudFilterOptions & options,
  FilterReport * report)
{
  const std::vector<Point3> standing = aboveGround(cloud, options);
  const std::vector<Point3> centres = voxelise(standing, options.voxel);
  const std::vector<Cluster> clusters = cluster(centres, options.cluster_distance);

  std::vector<Trunk> trunks;
  for (const Cluster & found : clusters) {
    if (found.members.size() < options.min_cluster_points) {
      continue;
    }
    double sx = 0.0;
    double sy = 0.0;
    double min_x = centres[found.members.front()].x;
    double max_x = min_x;
    double min_y = centres[found.members.front()].y;
    double max_y = min_y;
    double min_z = centres[found.members.front()].z;
    double max_z = min_z;
    for (const std::size_t index : found.members) {
      const Point3 & p = centres[index];
      sx += p.x;
      sy += p.y;
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
      min_z = std::min(min_z, p.z);
      max_z = std::max(max_z, p.z);
    }
    // A hedge or a fence rail is wide, a drooping branch is shallow. Only a
    // column narrow enough to be a trunk and tall enough to be worth avoiding
    // survives.
    if (std::hypot(max_x - min_x, max_y - min_y) > options.max_trunk_width) {
      continue;
    }
    if (max_z - min_z < options.min_trunk_extent) {
      continue;
    }
    const double n = static_cast<double>(found.members.size());
    trunks.push_back(Trunk{Point2{sx / n, sy / n}, static_cast<int>(found.members.size())});
  }

  if (report != nullptr) {
    report->received = cloud.size();
    report->above_ground = standing.size();
    report->voxels = centres.size();
    report->clusters = clusters.size();
    report->trunks = trunks.size();
  }
  return trunks;
}

}  // namespace kraken_orchard
