// SPDX-License-Identifier: BSD-3-Clause
#include "kraken_orchard/row_fit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace kraken_orchard
{
namespace
{

struct Line
{
  double slope{0.0};
  double intercept{0.0};
  double residual{0.0};
  double span{0.0};
};

std::string countReason(std::size_t count, const char * side)
{
  char buffer[96];
  std::snprintf(buffer, sizeof(buffer), "only %zu trunks on the %s", count, side);
  return std::string(buffer);
}

std::string spanReason(double span, const char * side)
{
  char buffer[96];
  std::snprintf(buffer, sizeof(buffer), "the %s trunks span only %.1f m", side, span);
  return std::string(buffer);
}

/// Which way the row runs: the trunks spread far further along it than across.
bool principalDirection(const std::vector<Point2> & points, Point2 & direction)
{
  if (points.size() < 3) {
    return false;
  }
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const Point2 & p : points) {
    mean_x += p.x;
    mean_y += p.y;
  }
  const double n = static_cast<double>(points.size());
  mean_x /= n;
  mean_y /= n;

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const Point2 & p : points) {
    const double dx = p.x - mean_x;
    const double dy = p.y - mean_y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }

  const double middle = 0.5 * (xx + yy);
  const double gap = std::hypot(0.5 * (xx - yy), xy);
  const double major = middle + gap;
  const double minor = middle - gap;
  if (major < 2.0 * minor) {
    return false;
  }

  Point2 axis{xy, major - xx};
  if (std::hypot(axis.x, axis.y) < 1e-9) {
    axis = Point2{major - yy, xy};
  }
  const double length = std::hypot(axis.x, axis.y);
  if (length < 1e-9) {
    return false;
  }
  axis.x /= length;
  axis.y /= length;
  if (axis.x < 0.0) {
    axis.x = -axis.x;
    axis.y = -axis.y;
  }
  direction = axis;
  return true;
}

bool fitLine(const std::vector<Point2> & row_frame, Line & line)
{
  const double n = static_cast<double>(row_frame.size());
  double sum_a = 0.0;
  double sum_c = 0.0;
  double sum_aa = 0.0;
  double sum_ac = 0.0;
  double min_a = row_frame.front().x;
  double max_a = min_a;
  for (const Point2 & p : row_frame) {
    sum_a += p.x;
    sum_c += p.y;
    sum_aa += p.x * p.x;
    sum_ac += p.x * p.y;
    min_a = std::min(min_a, p.x);
    max_a = std::max(max_a, p.x);
  }
  const double denominator = n * sum_aa - sum_a * sum_a;
  if (std::abs(denominator) < 1e-9) {
    return false;
  }
  line.slope = (n * sum_ac - sum_a * sum_c) / denominator;
  line.intercept = (sum_c - line.slope * sum_a) / n;
  line.span = max_a - min_a;

  double squared = 0.0;
  for (const Point2 & p : row_frame) {
    const double error = p.y - (line.slope * p.x + line.intercept);
    squared += error * error;
  }
  line.residual = std::sqrt(squared / n);
  return true;
}

}  // namespace

RowFit fitRow(const std::vector<Trunk> & trunks, const RowFitOptions & options)
{
  if (trunks.size() < 2 * options.min_trunks) {
    return RowFit{false, "only " + std::to_string(trunks.size()) + " trunks in view",
      0.0, 0.0, 0.0, 0, 0};
  }

  // Which trunks belong to this aisle depends on which way the row runs, and
  // the row direction is measured from those same trunks. Start with the
  // robot's own heading, which is right to within its steering error, and let
  // the gate and the direction refine each other.
  Point2 direction{1.0, 0.0};
  std::vector<Point2> kept;
  for (int pass = 0; pass < 3; ++pass) {
    kept.clear();
    for (const Trunk & trunk : trunks) {
      // The next row over stands a full pitch further out; letting it into the
      // fit would measure a corridor the robot is not in.
      const double across = -trunk.centre.x * direction.y + trunk.centre.y * direction.x;
      if (std::abs(across) <= options.max_lateral) {
        kept.push_back(trunk.centre);
      }
    }
    Point2 refined;
    if (!principalDirection(kept, refined)) {
      return RowFit{false, "the trunks are not arranged in lines", 0.0, 0.0, 0.0, 0, 0};
    }
    direction = refined;
  }

  // Sides are taken across the row, not across the robot. Yawed a little in a
  // 37 m row, the far end of the left-hand trunks sits to the robot's right.
  std::vector<Point2> sides[2];
  for (const Point2 & centre : kept) {
    const Point2 row_frame{
      centre.x * direction.x + centre.y * direction.y,
      -centre.x * direction.y + centre.y * direction.x};
    sides[row_frame.y > 0.0 ? 0 : 1].push_back(row_frame);
  }

  const int left_count = static_cast<int>(sides[0].size());
  const int right_count = static_cast<int>(sides[1].size());
  const char * names[2] = {"left", "right"};
  Line lines[2];
  for (int side = 0; side < 2; ++side) {
    if (sides[side].size() < options.min_trunks) {
      return RowFit{false, countReason(sides[side].size(), names[side]),
        0.0, 0.0, 0.0, left_count, right_count};
    }
    if (!fitLine(sides[side], lines[side])) {
      return RowFit{false, std::string("no line through the ") + names[side] + " trunks",
        0.0, 0.0, 0.0, left_count, right_count};
    }
    if (lines[side].span < options.min_span) {
      return RowFit{false, spanReason(lines[side].span, names[side]),
        0.0, 0.0, 0.0, left_count, right_count};
    }
    if (lines[side].residual > options.max_residual) {
      return RowFit{false, std::string("the ") + names[side] + " trunks do not lie on a line",
        0.0, 0.0, 0.0, left_count, right_count};
    }
  }

  if (std::abs(std::atan(lines[0].slope) - std::atan(lines[1].slope)) >
    options.max_heading_difference)
  {
    return RowFit{false, "the two sides are not parallel", 0.0, 0.0, 0.0,
      left_count, right_count};
  }

  const double slope = 0.5 * (lines[0].slope + lines[1].slope);
  const double intercept = 0.5 * (lines[0].intercept + lines[1].intercept);
  const double normalise = std::hypot(1.0, slope);
  return RowFit{
    true,
    "",
    std::abs(lines[0].intercept - lines[1].intercept) / normalise,
    intercept / normalise,
    std::atan2(direction.y, direction.x) + std::atan(slope),
    left_count,
    right_count};
}

}  // namespace kraken_orchard
