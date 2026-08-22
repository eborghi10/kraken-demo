// SPDX-License-Identifier: BSD-3-Clause
//
// A headland turn, planned as a Nav2 global planner.
//
// It is a planner and not a piece of the mission script because that is what it
// is: given where the machine is and which row it wants to be in next, it
// returns the path between them. Putting it behind the same interface as the
// search-based planner is what lets a row and the turn that follows it be
// concatenated and driven as one goal, which is the whole reason the machine no
// longer has to stop at the end of a row.
//
// The manoeuvre itself is decided in headland.cpp, which knows no ROS. All this
// class adds is the two things only the running system can supply: how deep the
// headland actually is, read off the costmap, and the transform from the
// machine's frame into the map's.

#ifndef KRAKEN_NAV__HEADLAND_PLANNER_HPP_
#define KRAKEN_NAV__HEADLAND_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kraken_nav/headland.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace kraken_nav
{

class HeadlandPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  /// How far the machine could run straight on before something stopped it.
  double freeDepth(double x, double y, double heading, double limit = 25.0) const;

  rclcpp::Logger logger_{rclcpp::get_logger("HeadlandPlanner")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  checker_;
  std::string frame_;
  std::string name_;

  headland::Footprint footprint_;
  double min_radius_{2.61};
  double wheelbase_{2.2};
  double half_width_{0.45};
  double entry_{0.8};
  double clearance_{1.0};
  double given_depth_{0.0};
  double step_{0.05};
};

}  // namespace kraken_nav

#endif  // KRAKEN_NAV__HEADLAND_PLANNER_HPP_
