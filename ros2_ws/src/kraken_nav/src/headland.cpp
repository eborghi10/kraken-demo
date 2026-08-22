// SPDX-License-Identifier: BSD-3-Clause

#include "kraken_nav/headland.hpp"

#include <algorithm>
#include <cmath>

namespace kraken_nav::headland
{

namespace
{

/// The y a fishtail lands on, which is the only thing the search below reads.
double landing(double offset, double radius, double first, double back)
{
  return trace(fishtail(offset, radius, first, back), 0.2).back().y;
}

/// Radii from the roomiest down to the tightest, widest first.
std::vector<double> radii(double most, double least, double step = 0.05)
{
  std::vector<double> out;
  const int count = std::max(0, static_cast<int>((most - least) / step));
  out.reserve(count + 1);
  for (int n = 0; n < count; ++n) {
    out.push_back(most - n * step);
  }
  out.push_back(least);
  return out;
}

}  // namespace

std::vector<Pose> trace(const std::vector<Segment> & segments, double step)
{
  double x = 0.0, y = 0.0, heading = 0.0;
  std::vector<Pose> path{{0.0, 0.0, 0.0, segments.front().curvature, segments.front().reverse}};
  for (const auto & segment : segments) {
    if (segment.length <= 1e-6) {
      continue;
    }
    const int count = std::max(1, static_cast<int>(std::ceil(segment.length / step)));
    const double ds = segment.length / count * (segment.reverse ? -1.0 : 1.0);
    for (int n = 0; n < count; ++n) {
      const double middle = heading + 0.5 * ds * segment.curvature;
      x += ds * std::cos(middle);
      y += ds * std::sin(middle);
      heading += ds * segment.curvature;
      path.push_back({x, y, heading, segment.curvature, segment.reverse});
    }
  }
  return path;
}

double reach(const std::vector<Pose> & path, const Footprint & footprint)
{
  double ahead = 0.0;
  for (const auto & pose : path) {
    const double c = std::cos(pose.heading), s = std::sin(pose.heading);
    for (const auto & [fx, fy] : footprint) {
      ahead = std::max(ahead, pose.x + c * fx - s * fy);
    }
  }
  return ahead;
}

std::vector<Segment> uTurn(double offset, double radius)
{
  const double side = std::copysign(1.0, offset);
  const double quarter = radius * M_PI / 2.0;
  return {{side / radius, quarter, false},
    {0.0, std::abs(offset) - 2.0 * radius, false},
    {side / radius, quarter, false}};
}

std::vector<Segment> bulbTurn(double offset, double radius)
{
  const double side = std::copysign(1.0, offset);
  const double gamma = std::acos(std::min(1.0, std::abs(offset) / (2.0 * radius)));
  return {{-side / radius, radius * gamma, false},
    {side / radius, radius * (M_PI + gamma), false}};
}

std::vector<Segment> fishtail(double offset, double radius, double first, double back)
{
  // All three arcs turn the machine the same way. The middle one does it
  // backwards, which is why its wheels are set the other way.
  const double side = std::copysign(1.0, offset);
  const double last = M_PI - first - back;
  return {{side / radius, radius * first, false},
    {-side / radius, radius * back, true},
    {side / radius, radius * last, false}};
}

std::optional<Turn> threePointTurn(
  double offset, double radius, const Footprint & footprint, double depth, double step)
{
  // The first and last arcs trade against each other, so the pair is searched:
  // for each first arc the reverse arc that lands the row is found by
  // bisection. Of the solutions that fit, the one with the longest first arc
  // wins rather than the shallowest. The shallowest is always the degenerate
  // one that barely turns before backing up, and it lands the row a metre out
  // if the machine did not start exactly where it thought; spending spare
  // headland on a balanced manoeuvre buys that back.
  std::vector<Turn> found;
  for (int n = 1; n <= 30; ++n) {
    const double first = 0.1 * n;

    bool bracketed = false;
    double low = 0.0, high = 0.0, previous_back = 0.0, previous_error = 0.0;
    bool have_previous = false;
    for (int m = 1; m <= 30; ++m) {
      const double back = 0.1 * m;
      if (first + back >= M_PI) {
        break;
      }
      const double error = landing(offset, radius, first, back) - offset;
      if (have_previous && previous_error * error <= 0.0) {
        low = previous_back;
        high = back;
        bracketed = true;
        break;
      }
      previous_back = back;
      previous_error = error;
      have_previous = true;
    }
    if (!bracketed) {
      continue;
    }

    double low_error = landing(offset, radius, first, low) - offset;
    for (int step_count = 0; step_count < 24; ++step_count) {
      const double middle = 0.5 * (low + high);
      const double error = landing(offset, radius, first, middle) - offset;
      if (error * low_error <= 0.0) {
        high = middle;
      } else {
        low = middle;
        low_error = error;
      }
    }

    auto segments = fishtail(offset, radius, first, 0.5 * (low + high));
    const double needed = reach(trace(segments, step), footprint);
    found.push_back({"3-point", std::move(segments), radius, needed});
  }

  if (found.empty()) {
    return std::nullopt;
  }
  for (auto turn = found.rbegin(); turn != found.rend(); ++turn) {
    if (turn->depth <= depth) {
      return *turn;
    }
  }
  return *std::min_element(
    found.begin(), found.end(),
    [](const Turn & a, const Turn & b) {return a.depth < b.depth;});
}

std::optional<Turn> planTurn(
  double offset, double min_radius, double depth, const Footprint & footprint, double step,
  const Accept & accept)
{
  auto usable = [&](const std::vector<Segment> & segments) {
      return !accept || accept(segments);
    };

  // Every manoeuvre is tried from the roomiest radius down to the tightest, and
  // the first that fits the headland wins. Radius is taken as large as the
  // ground allows rather than as small as the machine can manage, because a
  // turn driven at full lock has nothing left to correct with: every trim can
  // only widen it, so the first metre of drift is unrecoverable and the
  // manoeuvre becomes a one-way bet on the arithmetic. Backing off a third of
  // the way leaves about a fifth of the steering in hand, which is what the
  // tracker trims with. This is not a refinement -- driven at full lock the
  // bulb turn below ran 0.6 m wide and had to be abandoned.
  const double roomy = 1.35 * min_radius;

  if (std::abs(offset) >= 2.0 * min_radius) {
    for (double radius : radii(std::min(0.5 * std::abs(offset), roomy), min_radius)) {
      auto segments = uTurn(offset, radius);
      const double needed = reach(trace(segments, step), footprint);
      if (needed <= depth && usable(segments)) {
        return Turn{"u-turn", std::move(segments), radius, needed};
      }
    }
  }

  for (double radius : radii(roomy, min_radius)) {
    auto segments = bulbTurn(offset, radius);
    const double needed = reach(trace(segments, step), footprint);
    if (needed <= depth && usable(segments)) {
      return Turn{"bulb", std::move(segments), radius, needed};
    }
  }

  for (double radius : radii(roomy, min_radius, 0.2)) {
    auto turn = threePointTurn(offset, radius, footprint, depth, step);
    if (turn && turn->depth <= depth && usable(turn->segments)) {
      return turn;
    }
  }
  return std::nullopt;
}

int skipFor(double min_radius, double pitch)
{
  // A turn between antiparallel rows fits without reversing when the rows are
  // at least a turning diameter apart, so this is the diameter measured in row
  // spacings, rounded up.
  return std::max(1, static_cast<int>(std::ceil(2.0 * min_radius / pitch)));
}

}  // namespace kraken_nav::headland
