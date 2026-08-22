// SPDX-License-Identifier: BSD-3-Clause

#ifndef KRAKEN_NAV__COVER_ROWS_NAVIGATOR_HPP_
#define KRAKEN_NAV__COVER_ROWS_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kraken_interfaces/action/cover_rows.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav_msgs/msg/path.hpp"

namespace kraken_nav
{

/// One aisle's work: the row to drive, and where the turn out of it ends.
struct Leg
{
  uint16_t aisle;
  geometry_msgs::msg::PoseStamped row;
  geometry_msgs::msg::PoseStamped turn;
  bool has_turn;
};

using Legs = std::shared_ptr<std::vector<Leg>>;
using Missed = std::shared_ptr<std::vector<uint16_t>>;

/**
 * @brief Covers an orchard row by row, as a Nav2 navigator plugin.
 *
 * The navigator owns what is mission state and nothing else: where the rows
 * are, what order to take them in, and how far through the day's work the
 * machine is. Driving a single leg is left to the behaviour tree, which builds
 * it out of the stock Nav2 nodes -- plan, concatenate, follow, recover.
 */
class CoverRowsNavigator
  : public nav2_core::BehaviorTreeNavigator<kraken_interfaces::action::CoverRows>
{
public:
  using ActionT = kraken_interfaces::action::CoverRows;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;
  bool cleanup() override;

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;
  std::string getName() override {return std::string("cover_rows");}

  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  void goalCompleted(
    ActionT::Result::SharedPtr result, const nav2_behavior_tree::BtStatus final_bt_status) override;

private:
  /// Lay out the day's work in a frame anchored to where the machine stands.
  bool layOutTheField(ActionT::Goal::ConstSharedPtr goal);

  /// A pose in the row frame, expressed in the map.
  geometry_msgs::msg::PoseStamped inRowFrame(double along, double across, double heading) const;

  double anchor_x_{0.0}, anchor_y_{0.0}, anchor_heading_{0.0};
  double wheelbase_{2.2};
  double max_steering_angle_{0.7};

  std::string path_blackboard_id_{"path"};
  std::string legs_blackboard_id_{"legs"};
  std::string missed_blackboard_id_{"missed"};

  Legs legs_;
  Missed missed_;
  rclcpp::Time start_time_;
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace kraken_nav

#endif  // KRAKEN_NAV__COVER_ROWS_NAVIGATOR_HPP_
