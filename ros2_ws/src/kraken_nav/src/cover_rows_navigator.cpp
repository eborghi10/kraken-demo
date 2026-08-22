// SPDX-License-Identifier: BSD-3-Clause

#include "kraken_nav/cover_rows_navigator.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "kraken_nav/headland.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace kraken_nav
{

namespace
{

/// Aisle order that leaves no aisle unvisited and almost no turn too tight.
///
/// One pass per remainder class: aisles 0, skip, 2*skip ... on the way out and
/// the ones in between on the way back. Exactly one turn, where the outward
/// pass meets the return, is between neighbours; every other turn crosses the
/// full skip.
std::vector<uint16_t> aisleOrder(uint16_t count, uint16_t skip)
{
  std::vector<uint16_t> order;
  for (uint16_t start = 0; start < skip && order.size() < count; ++start) {
    std::vector<uint16_t> pass;
    for (uint16_t aisle = start; aisle < count; aisle += skip) {
      pass.push_back(aisle);
    }
    if (start % 2 == 1) {
      std::reverse(pass.begin(), pass.end());
    }
    order.insert(order.end(), pass.begin(), pass.end());
  }
  return order;
}

}  // namespace

bool CoverRowsNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  auto declare = [&](const std::string & key, auto fallback) {
      if (!node->has_parameter(key)) {
        node->declare_parameter(key, fallback);
      }
      return node->get_parameter(key);
    };
  path_blackboard_id_ = declare("path_blackboard_id", std::string("path")).as_string();
  legs_blackboard_id_ = declare("legs_blackboard_id", std::string("legs")).as_string();
  missed_blackboard_id_ = declare("missed_blackboard_id", std::string("missed")).as_string();
  wheelbase_ = declare(getName() + ".wheelbase", 2.2).as_double();
  max_steering_angle_ = declare(getName() + ".max_steering_angle", 0.7).as_double();

  odom_smoother_ = odom_smoother;
  return true;
}

bool CoverRowsNavigator::cleanup()
{
  odom_smoother_.reset();
  legs_.reset();
  missed_.reset();
  return true;
}

std::string CoverRowsNavigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  const std::string key = "default_cover_rows_bt_xml";
  if (!node->has_parameter(key)) {
    node->declare_parameter(
      key,
      ament_index_cpp::get_package_share_directory("kraken_nav") +
      "/behavior_trees/cover_rows.xml");
  }
  return node->get_parameter(key).as_string();
}

geometry_msgs::msg::PoseStamped CoverRowsNavigator::inRowFrame(
  double along, double across, double heading) const
{
  const double c = std::cos(anchor_heading_), s = std::sin(anchor_heading_);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = feedback_utils_.global_frame;
  pose.header.stamp = clock_->now();
  pose.pose.position.x = anchor_x_ + c * along - s * across;
  pose.pose.position.y = anchor_y_ + s * along + c * across;
  pose.pose.orientation.z = std::sin(0.5 * (heading + anchor_heading_));
  pose.pose.orientation.w = std::cos(0.5 * (heading + anchor_heading_));
  return pose;
}

bool CoverRowsNavigator::layOutTheField(ActionT::Goal::ConstSharedPtr goal)
{
  geometry_msgs::msg::PoseStamped here;
  if (!nav2_util::getCurrentPose(
      here, *feedback_utils_.tf, feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance))
  {
    RCLCPP_ERROR(logger_, "no %s pose; is localisation up?", feedback_utils_.robot_frame.c_str());
    return false;
  }

  // The rows are surveyed, not guessed at. A heading read off the filter while
  // the machine stands still is worth nothing, and an error in it multiplies by
  // the width of the orchard: a degree out over eighteen aisles aims the last
  // leg a metre into the trees.
  const double reported = tf2::getYaw(here.pose.orientation);
  anchor_heading_ = std::isfinite(goal->row_heading_deg) ?
    goal->row_heading_deg * M_PI / 180.0 : reported;
  anchor_x_ = here.pose.position.x;
  anchor_y_ = here.pose.position.y;

  const double min_radius = wheelbase_ / std::tan(max_steering_angle_);
  const uint16_t skip = goal->aisle_skip > 0 ?
    goal->aisle_skip : headland::skipFor(min_radius, goal->aisle_pitch);

  RCLCPP_INFO(
    logger_,
    "turning radius %.2f m over %.2f m rows: skipping %u, so a turn crosses %.1f m",
    min_radius, goal->aisle_pitch, skip, skip * goal->aisle_pitch);
  RCLCPP_INFO(
    logger_, "rows anchored at %s (%.1f, %.1f) heading %+.0f deg (localisation reported %+.0f)",
    feedback_utils_.global_frame.c_str(), anchor_x_, anchor_y_,
    anchor_heading_ * 180.0 / M_PI, reported * 180.0 / M_PI);

  const auto order = aisleOrder(goal->aisles, skip);
  legs_ = std::make_shared<std::vector<Leg>>();
  missed_ = std::make_shared<std::vector<uint16_t>>();

  bool outbound = true;
  for (std::size_t i = 0; i < order.size(); ++i) {
    const double heading = outbound ? 0.0 : M_PI;
    const double end = outbound ? goal->row_far_x : goal->row_near_x;

    Leg leg;
    leg.aisle = order[i];
    leg.row = inRowFrame(end, order[i] * goal->aisle_pitch, heading);
    leg.has_turn = i + 1 < order.size();
    if (leg.has_turn) {
      // A turn changes which row the machine is in, not how far along it is, so
      // it ends back at the mouth it left by, facing the other way.
      leg.turn = inRowFrame(end, order[i + 1] * goal->aisle_pitch, heading + M_PI);
      outbound = !outbound;
    }
    legs_->push_back(leg);
  }
  return true;
}

bool CoverRowsNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  if (!bt_action_server_->loadBehaviorTree(std::string())) {
    RCLCPP_ERROR(logger_, "could not load the coverage behaviour tree");
    return false;
  }
  if (goal->aisles == 0 || goal->aisle_pitch <= 0.0f) {
    RCLCPP_ERROR(logger_, "an orchard needs at least one aisle and a positive pitch");
    return false;
  }
  if (!layOutTheField(goal)) {
    return false;
  }

  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<Legs>(legs_blackboard_id_, legs_);
  blackboard->set<Missed>(missed_blackboard_id_, missed_);
  blackboard->set<int>("leg_index", 0);
  blackboard->set<int>("number_recoveries", 0);
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils_.tf);
  blackboard->set<std::string>("global_frame", feedback_utils_.global_frame);
  blackboard->set<std::string>("robot_base_frame", feedback_utils_.robot_frame);
  return true;
}

void CoverRowsNavigator::onLoop()
{
  auto blackboard = bt_action_server_->getBlackboard();
  int index = 0;
  // Absent before the first leg is handed out, which is not worth reporting on.
  (void)blackboard->get<int>("leg_index", index);

  auto feedback = std::make_shared<ActionT::Feedback>();
  feedback->legs_total = static_cast<uint16_t>(legs_ ? legs_->size() : 0);
  feedback->legs_done = static_cast<uint16_t>(std::max(0, index - 1));
  if (legs_ && index > 0 && static_cast<std::size_t>(index) <= legs_->size()) {
    feedback->current_aisle = (*legs_)[index - 1].aisle;
  }

  geometry_msgs::msg::PoseStamped pose;
  nav_msgs::msg::Path path;
  (void)blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, path);
  if (nav2_util::getCurrentPose(
      pose, *feedback_utils_.tf, feedback_utils_.global_frame, feedback_utils_.robot_frame,
      feedback_utils_.transform_tolerance) && !path.poses.empty())
  {
    feedback->distance_remaining = static_cast<float>(
      nav2_util::geometry_utils::euclidean_distance(pose, path.poses.back()));
  }

  feedback->navigation_time = clock_->now() - start_time_;
  bt_action_server_->publishFeedback(feedback);
}

void CoverRowsNavigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  // A coverage run carries the day's progress in it. Swapping the field out
  // from under a machine halfway down a row is a new job, not a preemption.
  RCLCPP_WARN(logger_, "refusing to preempt a coverage run; cancel it and send a new goal");
  (void)goal;
  bt_action_server_->terminatePendingGoal();
}

void CoverRowsNavigator::goalCompleted(
  ActionT::Result::SharedPtr result, const nav2_behavior_tree::BtStatus)
{
  const std::size_t total = legs_ ? legs_->size() : 0;
  const std::size_t missed = missed_ ? missed_->size() : 0;
  result->covered = static_cast<uint16_t>(total - missed);
  result->missed = missed_ ? *missed_ : std::vector<uint16_t>();
  result->total_time = clock_->now() - start_time_;

  if (missed == 0) {
    RCLCPP_INFO(logger_, "covered %zu aisles, all %zu legs succeeded", total, total);
  } else {
    std::string names;
    for (uint16_t aisle : *missed_) {
      names += (names.empty() ? "" : ", ") + std::to_string(aisle);
    }
    RCLCPP_WARN(
      logger_, "%zu of %zu legs failed: aisle %s", missed, total, names.c_str());
  }
}

}  // namespace kraken_nav

PLUGINLIB_EXPORT_CLASS(kraken_nav::CoverRowsNavigator, nav2_core::NavigatorBase)
