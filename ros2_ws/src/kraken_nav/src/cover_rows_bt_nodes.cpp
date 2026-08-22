// SPDX-License-Identifier: BSD-3-Clause
//
// The only behaviour tree nodes this mission needs that Nav2 does not already
// ship. Planning, concatenating, following, clearing and backing up are all
// stock; what is missing is only the bookkeeping of which aisle comes next and
// the cue to start thinking about the turn.

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kraken_nav/cover_rows_navigator.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace kraken_nav
{

namespace
{

/// Nav2 guarantees the node on every behaviour tree blackboard; nothing else
/// this file needs is worth crashing a mission over if it is missing.
rclcpp::Logger loggerOf(const BT::TreeNode & node)
{
  return node.config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger();
}

}  // namespace

/// Hand out the next aisle. Fails when the field is finished, which is what
/// ends the loop the tree runs it in.
class NextLeg : public BT::SyncActionNode
{
public:
  NextLeg(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("row_goal", "Far end of the row to drive"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("turn_goal", "Mouth of the next row"),
      BT::OutputPort<bool>("has_turn", "Whether a turn follows this row"),
      BT::OutputPort<int>("aisle", "Which aisle this leg covers"),
    };
  }

  BT::NodeStatus tick() override
  {
    auto legs = config().blackboard->get<Legs>("legs");
    int index = config().blackboard->get<int>("leg_index");
    if (!legs || static_cast<std::size_t>(index) >= legs->size()) {
      return BT::NodeStatus::FAILURE;
    }

    const Leg & leg = (*legs)[index];
    setOutput("row_goal", leg.row);
    setOutput("turn_goal", leg.turn);
    setOutput("has_turn", leg.has_turn);
    setOutput("aisle", static_cast<int>(leg.aisle));
    config().blackboard->set<int>("leg_index", index + 1);

    RCLCPP_INFO(loggerOf(*this), "leg %d/%zu: aisle %u", index + 1, legs->size(), leg.aisle);
    return BT::NodeStatus::SUCCESS;
  }
};

/// Write off the aisle being driven, so the run can carry on to the next one
/// and still report honestly at the end.
class MissedLeg : public BT::SyncActionNode
{
public:
  MissedLeg(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto legs = config().blackboard->get<Legs>("legs");
    auto missed = config().blackboard->get<Missed>("missed");
    const int index = config().blackboard->get<int>("leg_index") - 1;
    if (legs && missed && index >= 0 && static_cast<std::size_t>(index) < legs->size()) {
      missed->push_back((*legs)[index].aisle);
      RCLCPP_WARN(loggerOf(*this), "leg %d failed: aisle %u", index + 1, (*legs)[index].aisle);
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// Fires once per leg, when the end of the path being driven first comes
/// within reach.
///
/// The turn out of a row cannot be planned from the row's other end: at that
/// distance the costmap holds nothing but unknown ground, and unknown ground
/// has to be treated as solid. Asked late, from a costmap the machine has just
/// swept with its own lidar, the same question has a truthful answer.
///
/// It has to fire only once. Joining the turn onto the path does not put the
/// end of that path out of reach for long -- the machine is driving towards it
/// -- so an unlatched cue replans the same turn ten times over and re-sends the
/// path under a controller that is already following it.
class TurnDue : public BT::ConditionNode
{
public:
  TurnDue(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path being driven"),
      BT::InputPort<double>("distance", 5.0, "How near the end counts as near"),
      BT::InputPort<bool>("has_turn", true, "False on the last leg, which just stops"),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    double distance = 5.0;
    bool has_turn = true;
    getInput("distance", distance);
    getInput("has_turn", has_turn);

    auto blackboard = config().blackboard;
    const int leg = blackboard->get<int>("leg_index");
    if (!has_turn || leg == planned_for_leg_ || !getInput("path", path) || path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(
        pose, *blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"),
        blackboard->get<std::string>("global_frame"),
        blackboard->get<std::string>("robot_base_frame")))
    {
      return BT::NodeStatus::FAILURE;
    }
    if (nav2_util::geometry_utils::euclidean_distance(pose, path.poses.back()) > distance) {
      return BT::NodeStatus::FAILURE;
    }

    planned_for_leg_ = leg;
    return BT::NodeStatus::SUCCESS;
  }

private:
  int planned_for_leg_{-1};
};

}  // namespace kraken_nav

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kraken_nav::NextLeg>("NextLeg");
  factory.registerNodeType<kraken_nav::MissedLeg>("MissedLeg");
  factory.registerNodeType<kraken_nav::TurnDue>("TurnDue");
}
