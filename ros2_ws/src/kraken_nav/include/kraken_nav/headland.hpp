// SPDX-License-Identifier: BSD-3-Clause
//
// Headland turns worked out as geometry, before the machine moves.
//
// A headland turn is not a search problem. The vehicle has one turning radius,
// the rows have one spacing and the headland has one depth; between them they
// decide which manoeuvre is possible, and once it is chosen there is nothing
// left to decide while driving. That is the whole point of doing it here: a
// turn made of constant-curvature segments can be driven at a fixed steering
// angle and a fixed speed, and the controller is left correcting the difference
// between the arithmetic and the ground rather than re-deciding the manoeuvre
// twenty times a second.
//
// Three manoeuvres, in the order they are preferred:
//
//   u-turn   a quarter circle, a straight, and a quarter circle. Needs the next
//            row to be at least a turning diameter away and about one turning
//            radius of headland. Shortest, shallowest, and never reverses.
//   bulb     when the next row is nearer than a turning diameter, swing away
//            from it first and come back round the outside. Still never
//            reverses, but it costs roughly three radii of headland.
//   3-point  when the headland is too shallow for either, reversing is the only
//            way round. Slowest, and the only manoeuvre that puts the machine's
//            blind end towards the trees, so it is the last resort.
//
// Depth is measured to the swept footprint rather than to the axle. The Kraken
// carries 2.5 m of itself ahead of its rear axle, so its nose is already that
// far into the headland before the turn starts and its outer front corner
// reaches about 1.3 m further out than the radius the axle turns on. Planning
// on the axle alone is how a turn that fits on paper takes the front of the
// machine through a tree.
//
// Curvature here is tan(steer)/wheelbase -- the circle the wheels are set to,
// not the circle the body traces. It keeps its sign when the machine backs
// along it, which is what makes a three-point turn expressible as three arcs.
//
// Nothing in this file knows about ROS, so it can be exercised on its own.

#ifndef KRAKEN_NAV__HEADLAND_HPP_
#define KRAKEN_NAV__HEADLAND_HPP_

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kraken_nav::headland
{

struct Segment
{
  double curvature;
  double length;
  bool reverse;
};

struct Pose
{
  double x;
  double y;
  double heading;
  double curvature;
  bool reverse;
};

struct Turn
{
  std::string name;
  std::vector<Segment> segments;
  double radius;
  double depth;
};

/// Corners of the machine in its own frame, rear axle at the origin.
using Footprint = std::vector<std::pair<double, double>>;

/// Sample a segment list into poses, starting at the origin heading +x.
std::vector<Pose> trace(const std::vector<Segment> & segments, double step = 0.05);

/// How far the swept footprint reaches ahead of where the manoeuvre started.
double reach(const std::vector<Pose> & path, const Footprint & footprint);

std::vector<Segment> uTurn(double offset, double radius);
std::vector<Segment> bulbTurn(double offset, double radius);

/// Forward arc, reverse arc, forward arc; the three points of a three-point turn.
std::vector<Segment> fishtail(double offset, double radius, double first, double back);

/// The best three-point turn that lands the next row, for the depth given.
std::optional<Turn> threePointTurn(
  double offset, double radius, const Footprint & footprint, double depth, double step = 0.05);

/// Whether a candidate manoeuvre is drivable on the ground, not just on paper.
using Accept = std::function<bool (const std::vector<Segment> &)>;

/// Choose the turn into a row `offset` across, given the headland available.
///
/// Depth alone cannot certify a turn: it is measured along the heading, and
/// the manoeuvre leaves that heading. `accept` is offered every candidate that
/// fits, so the caller can check the ground the turn actually sweeps.
std::optional<Turn> planTurn(
  double offset, double min_radius, double depth, const Footprint & footprint,
  double step = 0.05, const Accept & accept = nullptr);

/// How many rows to leave between passes so a turn need never reverse.
int skipFor(double min_radius, double pitch);

}  // namespace kraken_nav::headland

#endif  // KRAKEN_NAV__HEADLAND_HPP_
