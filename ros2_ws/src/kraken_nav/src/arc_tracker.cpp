// SPDX-License-Identifier: BSD-3-Clause

#include "kraken_nav/arc_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace kraken_nav
{

void ArcTracker::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = name;

  auto get = [&](const std::string & key, double fallback) {
      nav2_util::declare_parameter_if_not_declared(
        node, name_ + "." + key, rclcpp::ParameterValue(fallback));
      return node->get_parameter(name_ + "." + key).as_double();
    };
  wheelbase_ = get("wheelbase", 2.2);
  const double steering = get("max_steering_angle", 0.7);
  desired_linear_vel_ = get("desired_linear_vel", 0.8);
  turn_linear_vel_ = get("turn_linear_vel", 0.4);
  max_accel_ = get("max_accel", 0.4);
  max_decel_ = get("max_decel", 0.4);
  straight_curvature_ = get("straight_curvature", 0.02);
  curvature_window_ = get("curvature_window", 0.15);
  curvature_samples_ = static_cast<std::size_t>(get("curvature_samples", 4.0));
  min_segment_ = get("min_segment", 1.0);
  heading_gain_ = get("heading_gain", 0.8);
  cross_gain_ = get("cross_gain", 0.6);
  correction_share_ = get("correction_share", 0.3);
  max_cross_track_ = get("max_cross_track", 0.75);
  collision_lookahead_ = get("collision_lookahead", 2.0);
  transform_tolerance_ = get("transform_tolerance", 0.2);

  min_radius_ = wheelbase_ / std::tan(steering);
  checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());

  // The row and the turn appended to it arrive here as one path and nowhere
  // else, so this is the only place the mission is visible as the machine
  // understands it.
  // Transient local so an rviz started mid-mission sees the current leg rather
  // than an empty display until the next one is planned.
  plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>(
    "mission_plan", rclcpp::QoS(1).transient_local());
}

void ArcTracker::cleanup()
{
  checker_.reset();
  plan_publisher_.reset();
}
void ArcTracker::activate()
{
  plan_publisher_->on_activate();
  reset();
}
void ArcTracker::deactivate()
{
  plan_publisher_->on_deactivate();
  reset();
}

void ArcTracker::reset()
{
  plan_.clear();
  index_ = 0;
  speed_ = 0.0;
  running_ = false;
  worst_cross_ = worst_step_ = last_steer_ = 0.0;
  reported_ = false;
}

void ArcTracker::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_fraction_ = 1.0;
  } else if (percentage) {
    speed_fraction_ = speed_limit / 100.0;
  } else {
    speed_fraction_ = speed_limit / desired_linear_vel_;
  }
}

void ArcTracker::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2) {
    throw nav2_core::InvalidPath("a path of fewer than two poses has no direction to follow");
  }
  digest(path);
  plan_frame_ = path.header.frame_id;
  plan_publisher_->publish(path);
  index_ = 0;
  running_ = false;
  worst_cross_ = worst_step_ = last_steer_ = 0.0;
  reported_ = false;

  // What was actually handed over, in the terms the wheels care about. A row
  // that comes back with direction changes in it, or with a radius tighter than
  // the machine owns, is a planning problem showing up as a driving one.
  std::size_t cusps = 0;
  double sharpest = 0.0;
  for (std::size_t i = 1; i < plan_.size(); ++i) {
    cusps += plan_[i].reverse != plan_[i - 1].reverse;
    sharpest = std::max(sharpest, std::abs(plan_[i].curvature));
  }
  RCLCPP_INFO(
    logger_, "given %.1f m of path in %zu poses, %zu direction changes, tightest radius %.2f m",
    plan_.front().to_end, plan_.size(), cusps,
    sharpest > 1e-6 ? 1.0 / sharpest : std::numeric_limits<double>::infinity());
}

void ArcTracker::digest(const nav_msgs::msg::Path & path)
{
  // A nav_msgs/Path carries poses and nothing else, so the curvature the wheels
  // are meant to hold has to be read back out of it -- and a search planner's
  // output is not a description of how to steer. Smac returns this orchard's
  // 46 m rows as 114 poses, 0.4 m apart, each carrying a few degrees of its own
  // noise. Read literally, by differencing neighbours, that straight row comes
  // back claiming two direction changes and a 1.47 m turning radius, which is
  // tighter than the machine can physically steer. Both readings are artefacts
  // of the sampling, and both were driven, which is how the machine ended up
  // 0.77 m out of a 3.5 m row.
  //
  // So neither direction nor curvature is read between neighbours. Direction is
  // read over a run: a change of direction that persists for less than the
  // length of a real manoeuvre segment is noise, not a cusp. Curvature is read
  // over a window, where the total heading change is what matters and the noise
  // on the two ends is divided by the window rather than by one 0.4 m step.
  const std::size_t count = path.poses.size();
  std::vector<double> heading(count), step(count, 0.0);
  std::vector<bool> reverse(count, false);

  for (std::size_t i = 0; i < count; ++i) {
    heading[i] = tf2::getYaw(path.poses[i].pose.orientation);
  }
  for (std::size_t i = 0; i + 1 < count; ++i) {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    step[i] = std::hypot(dx, dy);
    reverse[i] = dx * std::cos(heading[i]) + dy * std::sin(heading[i]) < 0.0;
  }
  reverse[count - 1] = reverse[count - 2];

  // Absorb every run shorter than a manoeuvre segment into the one before it.
  // The shortest arc a headland turn is ever built from is over a metre long,
  // so nothing real is lost, and a single pose whose heading points the wrong
  // way no longer splits the path in two.
  for (std::size_t i = 0; i < count; ) {
    std::size_t j = i;
    double run = 0.0;
    while (j < count && reverse[j] == reverse[i]) {
      run += step[j];
      ++j;
    }
    if (i > 0 && run < min_segment_) {
      std::fill(reverse.begin() + i, reverse.begin() + j, reverse[i - 1]);
      // The run just merged may have joined two runs that now agree, so start
      // again from the run it was absorbed into rather than stepping past it.
      i = j;
      continue;
    }
    i = j;
  }

  plan_.assign(count, Point{});
  for (std::size_t i = 0; i < count; ++i) {
    plan_[i].x = path.poses[i].pose.position.x;
    plan_[i].y = path.poses[i].pose.position.y;
    plan_[i].heading = heading[i];
    plan_[i].reverse = reverse[i];

    // How far the window has to reach is a property of how noisy the plan is,
    // not of how long it is, and the plan says which it is by how finely it
    // sampled itself. A search planner that returns poses 0.4 m apart is
    // telling you it resolved the path to 0.4 m; the geometry planner, tracing
    // an exact arc every 5 cm, is telling you it did not guess. Counting
    // samples rather than metres therefore averages hard over the one and
    // barely at all over the other -- which matters, because a fixed 1.5 m
    // window smears the two arcs of a bulb turn into each other across the
    // point where the steering is supposed to cross over, and that cost a
    // measured 0.75 m of drift halfway round.
    double travelled = 0.0, turned = 0.0;
    std::size_t j = i, samples = 0;
    while (j + 1 < count && reverse[j] == reverse[i] &&
      (samples < curvature_samples_ || travelled < curvature_window_))
    {
      travelled += step[j];
      turned += angles::shortest_angular_distance(heading[j], heading[j + 1]);
      ++samples;
      ++j;
    }
    // The last few poses of a segment leave too short a base to divide by, and
    // say nothing about curvature that the pose before them did not.
    const double signed_travel = travelled * (reverse[i] ? -1.0 : 1.0);
    plan_[i].curvature = travelled > 0.5 * curvature_window_ ?
      turned / signed_travel : (i > 0 ? plan_[i - 1].curvature : 0.0);
    // Below this the arc is straighter than the machine can be steered to hold,
    // so it is a straight, and holding zero beats holding the noise.
    if (std::abs(plan_[i].curvature) < straight_curvature_) {
      plan_[i].curvature = 0.0;
    }
  }

  double to_end = 0.0;
  for (std::size_t i = count; i-- > 0; ) {
    plan_[i].to_end = to_end;
    to_end += i > 0 ? step[i - 1] : 0.0;
  }

  double to_cusp = 0.0;
  for (std::size_t i = count; i-- > 0; ) {
    if (i + 1 < count && reverse[i] != reverse[i + 1]) {
      to_cusp = 0.0;
    }
    plan_[i].to_cusp = to_cusp;
    to_cusp += i > 0 ? step[i - 1] : 0.0;
  }
}

void ArcTracker::advance(double x, double y)
{
  while (index_ + 1 < plan_.size()) {
    const auto & point = plan_[index_];
    double ahead = (x - point.x) * std::cos(point.heading) + (y - point.y) * std::sin(point.heading);
    if (point.reverse) {
      ahead = -ahead;
    }
    if (ahead <= 0.0) {
      break;
    }
    ++index_;
  }
}

bool ArcTracker::blocked(
  std::size_t from, const tf2::Transform & costmap_from_plan, std::string * why)
{
  const auto & footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {
    return false;
  }
  // The plan is traced far more finely than the costmap is drawn, so testing
  // every pose of it would ask the same cells the same question five times
  // over. One test per costmap cell of travel is as much as the map can answer.
  const double resolution = costmap_ros_->getCostmap()->getResolution();
  const double turn = tf2::getYaw(costmap_from_plan.getRotation());
  const double start = plan_[from].to_end;

  // Recursive, so this is safe whether or not the caller already holds it.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *costmap_ros_->getCostmap()->getMutex());

  double tested = -resolution;
  for (std::size_t i = from; i < plan_.size(); ++i) {
    const double gone = start - plan_[i].to_end;
    if (gone > collision_lookahead_) {
      break;
    }
    if (gone - tested < resolution) {
      continue;
    }
    tested = gone;

    const tf2::Vector3 here = costmap_from_plan * tf2::Vector3(plan_[i].x, plan_[i].y, 0.0);
    const double cost =
      checker_->footprintCostAtPose(here.x(), here.y(), turn + plan_[i].heading, footprint);
    // Only an occupied cell counts. The value below it, "inscribed", means the
    // footprint is merely near something, and in a 3.5 m aisle driven by a
    // 0.9 m machine against a 0.55 m inflation radius that is most of the row:
    // measured, the costmap reports it 0.9 m ahead of a robot standing still
    // and unobstructed. Refusing to drive there would refuse to drive at all.
    // Unknown ground does not count either, though it scores higher than
    // occupied does and so has to be excluded by name, or the machine would
    // decline to enter any row it had not already driven.
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE &&
      cost != nav2_costmap_2d::NO_INFORMATION)
    {
      char said[160];
      std::snprintf(
        said, sizeof(said),
        "the path ahead is blocked: %.1f m along it the footprint costs %.0f at %s (%.2f, %.2f)",
        gone, cost, costmap_ros_->getGlobalFrameID().c_str(), here.x(), here.y());
      *why = said;
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::TwistStamped ArcTracker::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  // Everything is worked out in the frame the plan came in, so the plan is
  // never transformed: it can be five thousand poses long and the robot pose is
  // one. The command that comes back is in the machine's own frame either way.
  geometry_msgs::msg::TransformStamped plan_from_costmap;
  try {
    plan_from_costmap = tf_->lookupTransform(
      plan_frame_, pose.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & error) {
    throw nav2_core::ControllerTFError(
            "no " + pose.header.frame_id + " -> " + plan_frame_ + " transform: " + error.what());
  }

  tf2::Transform plan_from_costmap_tf;
  tf2::fromMsg(plan_from_costmap.transform, plan_from_costmap_tf);
  tf2::Transform robot;
  tf2::fromMsg(pose.pose, robot);
  robot = plan_from_costmap_tf * robot;

  const double x = robot.getOrigin().x(), y = robot.getOrigin().y();
  const double yaw = tf2::getYaw(robot.getRotation());

  advance(x, y);
  // The cusp cap below parks the machine on a direction change, and advance()
  // only steps forward off a point once it has been driven past, which a parked
  // machine never does. Stepping over by hand is what makes the second leg of a
  // three point turn start; without it the plan stalls on the cusp for good.
  if (speed_ == 0.0 && index_ + 1 < plan_.size() &&
    plan_[index_].reverse != plan_[index_ + 1].reverse)
  {
    ++index_;
  }
  const auto & target = plan_[index_];

  const double across = -(x - target.x) * std::sin(target.heading) +
    (y - target.y) * std::cos(target.heading);
  const double drift = angles::shortest_angular_distance(target.heading, yaw);
  worst_cross_ = std::max(worst_cross_, std::abs(across));
  if (std::abs(across) > max_cross_track_) {
    RCLCPP_WARN(
      logger_, "%.2f m off the path it was driving; giving up rather than lunging back", across);
    throw nav2_core::NoValidControl("too far off the planned path to recover from");
  }

  std::string why;
  if (blocked(index_, plan_from_costmap_tf.inverse(), &why)) {
    speed_ = 0.0;
    RCLCPP_WARN(logger_, "%s", why.c_str());
    throw nav2_core::NoValidControl(why);
  }

  // Which correction term reverses when the machine does is worth deriving
  // rather than guessing, because guessing it backwards is stable-looking and
  // divergent. With cross-track e and heading error psi, the machine obeys
  // e' = v.psi and psi' = v.k. Driving forward, k = -(a.psi + b.e) gives that
  // pair a negative trace and a positive determinant, so it settles. Put v < 0
  // into the same law and one eigenvalue turns positive: the machine leaves the
  // path faster the harder it corrects. The law that settles in reverse is
  // k = +a.psi - b.e. So it is the heading term that flips, not the lateral one
  // -- the opposite of what seems obvious, and worth a measured 0.77 m of drift
  // out of a 3.5 m row before the tracker gave up.
  const double limit = 1.0 / min_radius_;
  const double allowance = correction_share_ * limit;
  const double towards = target.reverse ? -drift : drift;
  const double trim = std::clamp(
    -(heading_gain_ * towards + cross_gain_ * across), -allowance, allowance);
  const double curvature = std::clamp(target.curvature + trim, -limit, limit);

  double wanted = std::abs(target.curvature) > straight_curvature_ ?
    turn_linear_vel_ : desired_linear_vel_;
  wanted *= speed_fraction_;
  // An Ackermann machine cannot swap ends while rolling, so a change of
  // direction is a stop whether or not it is asked for as one.
  wanted = std::min(wanted, std::sqrt(2.0 * max_decel_ * std::max(0.0, target.to_end)));
  wanted = std::min(wanted, std::sqrt(2.0 * max_decel_ * std::max(0.0, target.to_cusp)));
  if (target.reverse) {
    wanted = -wanted;
  }

  const rclcpp::Time now = clock_->now();
  const bool continuing = running_;
  const double elapsed = continuing ? std::clamp((now - last_command_).seconds(), 0.0, 0.5) : 0.0;
  running_ = true;
  last_command_ = now;
  const double change = std::max(max_accel_, max_decel_) * elapsed;
  speed_ = std::clamp(wanted, speed_ - change, speed_ + change);

  const double steer = std::atan(wheelbase_ * curvature);
  // Only between commands, so the first one is not scored against a remembered
  // zero. Taking up the steering a plan starts with is not a jump in it.
  if (continuing) {
    worst_step_ = std::max(worst_step_, std::abs(steer - last_steer_));
  }
  last_steer_ = steer;
  // Reported just short of the end rather than at it: the goal checker calls
  // the leg done a tolerance short of the last pose, so a summary saved for the
  // final pose would never be printed.
  if (target.to_end < 1.0 && !reported_) {
    reported_ = true;
    RCLCPP_INFO(
      logger_, "path driven: %.2f m off it at worst, steering moved at most %.3f rad between "
      "commands", worst_cross_, worst_step_);
  }

  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = now;
  command.twist.linear.x = speed_;
  command.twist.angular.z = speed_ * curvature;
  return command;
}

}  // namespace kraken_nav

PLUGINLIB_EXPORT_CLASS(kraken_nav::ArcTracker, nav2_core::Controller)
