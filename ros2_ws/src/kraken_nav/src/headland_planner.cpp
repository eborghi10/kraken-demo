// SPDX-License-Identifier: BSD-3-Clause

#include "kraken_nav/headland_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace kraken_nav
{

void HeadlandPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  costmap_ros_ = costmap_ros;
  checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());
  frame_ = costmap_ros_->getGlobalFrameID();
  name_ = name;

  double steering = 0.7, front = 2.5, rear = 0.6;
  auto get = [&](const std::string & key, double fallback) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(fallback));
      return node->get_parameter(name_ + "." + key).as_double();
    };
  wheelbase_ = get("wheelbase", 2.2);
  steering = get("max_steering_angle", 0.7);
  front = get("front_overhang", 2.5);
  rear = get("rear_overhang", 0.6);
  half_width_ = get("half_width", 0.45);
  // Straight out of the row before anything swings, so the back of the machine
  // is past the last trunks before the wheels go over.
  entry_ = get("entry", 0.8);
  clearance_ = get("clearance", 1.0);
  // Zero measures the headland off the costmap, which is the only honest
  // source: the two headlands in this orchard differ by eleven metres.
  given_depth_ = get("headland_depth", 0.0);
  step_ = get("step", 0.05);

  min_radius_ = wheelbase_ / std::tan(steering);
  footprint_ = {{front, half_width_}, {front, -half_width_},
    {-rear, -half_width_}, {-rear, half_width_}};

  RCLCPP_INFO(
    logger_, "%s turns on %.2f m at %.2f rad of steering", name_.c_str(), min_radius_, steering);
}

double HeadlandPlanner::freeDepth(double x, double y, double heading, double limit) const
{
  // Read along the way the robot is pointing, in a corridor its own width.
  // Unknown ground counts as blocked, not free, and the distinction decides
  // whether the machine drives into the trees. Counted as free, this headland
  // measured 6.8 m from twelve metres back, because from there the lidar had
  // not swept it and the whole far side was unknown; driven, it is 5.9 m, and
  // the manoeuvre chosen to fit 6.8 m does not fit 5.9. A headland is only as
  // deep as it has been seen to be, so the answer to "how much room is there"
  // has to be asked from close enough to have looked.
  auto * costmap = costmap_ros_->getCostmap();
  const double c = std::cos(heading), s = std::sin(heading);
  const double resolution = costmap->getResolution();

  for (double distance = 0.0; distance < limit; distance += resolution) {
    for (double lateral : {-half_width_, 0.0, half_width_}) {
      const double px = x + c * distance - s * lateral;
      const double py = y + s * distance + c * lateral;
      unsigned int mx = 0, my = 0;
      if (!costmap->worldToMap(px, py, mx, my)) {
        return distance;
      }
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return distance;
      }
    }
  }
  return limit;
}

nav_msgs::msg::Path HeadlandPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()>)
{
  const double sx = start.pose.position.x, sy = start.pose.position.y;
  const double heading = tf2::getYaw(start.pose.orientation);
  const double c = std::cos(heading), s = std::sin(heading);

  // The goal is the mouth of the next row, so its offset across the rows says
  // which manoeuvre is needed and its distance along them says where to stop.
  const double dx = goal.pose.position.x - sx, dy = goal.pose.position.y - sy;
  const double along = c * dx + s * dy;
  const double offset = -s * dx + c * dy;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros_->getCostmap()->getMutex()));
  const double depth = given_depth_ > 0.0 ? given_depth_ : freeDepth(sx, sy, heading);
  // What the manoeuvre is actually allowed. The entry is spent before it
  // starts, and the clearance is not spent at all: the turn is chosen from
  // where the row is predicted to end, and the machine arrives there with a
  // little error in hand. Measured, this headland is 6.7 m and the widest
  // u-turn sweeps 5.5 m of it, which fits on paper by 0.4 m and did not fit on
  // the ground. Held back, the same ladder simply chooses a tighter turn.
  const double room = depth - entry_ - clearance_;

  // Depth is read along the heading; the turn leaves the heading. A u-turn onto
  // a row seven metres over sweeps seven metres sideways, across ground the
  // depth ray never looked at, so a clear ray is no promise that the manoeuvre
  // is clear. Seven turns got away with it and the eighth put the machine into
  // the trees. So every candidate is driven over the costmap here, footprint
  // and all, and one that touches anything lethal is passed over for the next
  // rung down the ladder.
  int rejected = 0;
  double worst = 0.0;
  std::string why = "was stopped before it began";
  auto clear = [&](const std::vector<headland::Segment> & segments) {
      std::vector<headland::Segment> whole{{0.0, entry_, false}};
      whole.insert(whole.end(), segments.begin(), segments.end());
      double travelled = 0.0;
      for (const auto & pose : headland::trace(whole, step_)) {
        travelled += step_;
        const double px = sx + c * pose.x - s * pose.y;
        const double py = sy + s * pose.x + c * pose.y;
        const double cost = checker_->footprintCostAtPose(
          px, py, heading + pose.heading, costmap_ros_->getRobotFootprint());
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          ++rejected;
          worst = std::max(worst, travelled);
          if (travelled >= worst) {
            why = "costs " + std::to_string(static_cast<int>(cost)) + " at (" +
              std::to_string(px) + ", " + std::to_string(py) + "), " +
              std::to_string(travelled) + " m into the manoeuvre";
          }
          return false;
        }
      }
      return true;
    };

  auto chosen = headland::planTurn(offset, min_radius_, room, footprint_, step_, clear);
  lock.unlock();
  if (!chosen) {
    // Whether the machine is standing somewhere it believes is solid separates
    // a headland that is genuinely too tight from a costmap it has poisoned.
    const double here = checker_->footprintCostAtPose(
      sx, sy, heading, costmap_ros_->getRobotFootprint());
    throw nav2_core::NoValidPathCouldBeFound(
            "headland measures " + std::to_string(depth) + " m; no turn onto a row " +
            std::to_string(std::abs(offset)) + " m across fits in it, not even a three-point one. " +
            std::to_string(rejected) + " candidates hit something, the one that got furthest " +
            why + "; the machine's own footprint here costs " +
            std::to_string(static_cast<int>(here)));
  }

  double length = 0.0;
  for (const auto & segment : chosen->segments) {
    length += segment.length;
  }
  RCLCPP_INFO(
    logger_,
    "%s %.2f m to its %s in %.1f m of headland, needing %.1f m: radius %.2f m at %.2f rad of "
    "steering, %.1f m of path",
    chosen->name.c_str(), std::abs(offset), offset > 0.0 ? "left" : "right", depth,
    entry_ + chosen->depth, chosen->radius, std::atan(wheelbase_ / chosen->radius),
    entry_ + length);

  // A turn changes which row the machine is in, not how far along it is. A bulb
  // ends six metres deeper into the headland than it started, and a row planned
  // from out there makes the search planner fan out across open ground until it
  // exhausts its iterations; running back to the row mouth first puts the next
  // start in the corridor, where it belongs.
  const double manoeuvre_end = headland::trace(chosen->segments, step_).back().x;
  std::vector<headland::Segment> segments{{0.0, entry_, false}};
  segments.insert(segments.end(), chosen->segments.begin(), chosen->segments.end());
  segments.push_back({0.0, std::max(0.0, entry_ + manoeuvre_end - along), false});

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_;
  path.header.stamp = start.header.stamp;
  for (const auto & pose : headland::trace(segments, step_)) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    stamped.pose.position.x = sx + c * pose.x - s * pose.y;
    stamped.pose.position.y = sy + s * pose.x + c * pose.y;
    stamped.pose.orientation.z = std::sin(0.5 * (heading + pose.heading));
    stamped.pose.orientation.w = std::cos(0.5 * (heading + pose.heading));
    path.poses.push_back(stamped);
  }
  return path;
}

}  // namespace kraken_nav

PLUGINLIB_EXPORT_CLASS(kraken_nav::HeadlandPlanner, nav2_core::GlobalPlanner)
