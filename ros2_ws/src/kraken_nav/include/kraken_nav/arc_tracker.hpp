// SPDX-License-Identifier: BSD-3-Clause
//
// Drive a path that already knows its own curvature.
//
// The paths this follows are not suggestions. A headland manoeuvre is a
// sequence of constant-curvature arcs worked out from the machine's turning
// radius and the room available, so the steering angle every metre of it wants
// is known before the wheels move. This commands that angle outright and lets
// feedback do nothing but trim it.
//
// The trim is capped at a share of the steering left unused at the radius the
// manoeuvre was planned on, which is the point of planning on a radius wider
// than the machine's tightest. A turn driven at full lock has nothing left to
// correct with -- every trim can only widen it -- and the previous sampling
// controller, re-deciding the whole manoeuvre every 50 ms, moved the steering
// 0.24 to 0.36 rad between consecutive commands and reversed its direction
// about seven times a second. Measured on the same ground, this moves it 0.008.
//
// Speed is not part of the plan and is decided here: full speed where the path
// is straight, the turning speed where it bends, and braked to a stop wherever
// the path changes direction, because an Ackermann machine cannot swap ends
// while rolling.

#ifndef KRAKEN_NAV__ARC_TRACKER_HPP_
#define KRAKEN_NAV__ARC_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace kraken_nav
{

class ArcTracker : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset() override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  /// A path point with everything the wheels need to know about it.
  struct Point
  {
    double x;
    double y;
    double heading;
    double curvature;   ///< tan(steer)/wheelbase, signed for the way it turns
    bool reverse;
    double to_end;      ///< metres of travel left to the last point
    double to_cusp;     ///< metres to where the path changes direction
  };

  /// Read curvature and direction back out of a plain sequence of poses.
  void digest(const nav_msgs::msg::Path & path);

  /// Walk the index forward past every point the machine has already passed.
  void advance(double x, double y);

  bool blocked(std::size_t from, const tf2::Transform & costmap_from_plan, std::string * why);

  rclcpp::Logger logger_{rclcpp::get_logger("ArcTracker")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> checker_;
  std::string name_;

  std::vector<Point> plan_;
  std::string plan_frame_;
  std::size_t index_{0};
  double speed_{0.0};
  double speed_fraction_{1.0};
  rclcpp::Time last_command_;
  bool running_{false};

  // Worst case seen on this plan, reported once when it ends rather than
  // sampled by hand off a topic afterwards.
  double worst_cross_{0.0};
  double worst_step_{0.0};
  double last_steer_{0.0};
  bool reported_{false};

  double wheelbase_{2.2};
  double min_radius_{2.61};
  double desired_linear_vel_{0.8};
  double turn_linear_vel_{0.4};
  double max_accel_{0.4};
  double max_decel_{0.4};
  double straight_curvature_{0.02};
  double curvature_window_{0.15};
  std::size_t curvature_samples_{4};
  double min_segment_{1.0};
  double heading_gain_{0.8};
  double cross_gain_{0.6};
  double correction_share_{0.3};
  double max_cross_track_{0.75};
  double collision_lookahead_{2.0};
  double transform_tolerance_{0.2};
};

}  // namespace kraken_nav

#endif  // KRAKEN_NAV__ARC_TRACKER_HPP_
