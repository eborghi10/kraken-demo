// SPDX-License-Identifier: BSD-3-Clause
//
// Measure the aisle the robot is standing in from the trunks either side of it.
//
// Inside a row the useful structure is a corridor, not a map. The trunks are
// the only thing that constrains the robot and the lidar sees them directly,
// so this fits a line to each side and reports the corridor between them: how
// wide it is, where its centre lies, and which way it runs. Following that
// needs no estimate of where the robot is in the world, only what is beside
// it, which is why it survives the localisation drift that a costmap search
// does not.
//
// The centreline is published as a path so nav2's controller can drive it
// through FollowPath and keep its own footprint collision checking. This
// replaces the planner inside a row, not the controller.
//
// This measures a row, it does not find one.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "kraken_interfaces/msg/row_estimate.hpp"
#include "kraken_orchard/cloud_filter.hpp"
#include "kraken_orchard/row_fit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace kraken_orchard
{

class RowFollower : public rclcpp::Node
{
public:
  RowFollower()
  : Node("row_follower")
  {
    filter_.ground_cell = declare_parameter("ground_cell", filter_.ground_cell);
    filter_.min_trunk_height = declare_parameter("min_trunk_height", filter_.min_trunk_height);
    filter_.max_trunk_height = declare_parameter("max_trunk_height", filter_.max_trunk_height);
    filter_.min_range = declare_parameter("min_range", filter_.min_range);
    filter_.max_range = declare_parameter("max_range", filter_.max_range);
    filter_.voxel = declare_parameter("voxel", filter_.voxel);
    filter_.cluster_distance = declare_parameter("cluster_distance", filter_.cluster_distance);
    filter_.max_trunk_width = declare_parameter("max_trunk_width", filter_.max_trunk_width);
    filter_.min_trunk_extent = declare_parameter("min_trunk_extent", filter_.min_trunk_extent);
    filter_.min_cluster_points = static_cast<std::size_t>(
      declare_parameter("min_cluster_points", static_cast<int>(filter_.min_cluster_points)));

    fit_.max_lateral = declare_parameter("max_lateral", fit_.max_lateral);
    fit_.min_trunks = static_cast<std::size_t>(
      declare_parameter("min_trunks", static_cast<int>(fit_.min_trunks)));
    fit_.min_span = declare_parameter("min_span", fit_.min_span);
    fit_.max_residual = declare_parameter("max_residual", fit_.max_residual);
    fit_.max_heading_difference = declare_parameter(
      "max_heading_difference", fit_.max_heading_difference);

    ahead_ = declare_parameter("path_ahead", 10.0);
    behind_ = declare_parameter("path_behind", 3.0);
    step_ = declare_parameter("path_step", 0.5);

    row_publisher_ = create_publisher<kraken_interfaces::msg::RowEstimate>("orchard/row", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("orchard/centreline", 10);
    trunk_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("orchard/trunks", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pc", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    std::vector<Point3> cloud;
    cloud.reserve(msg.width * msg.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
        cloud.push_back(Point3{*x, *y, *z});
      }
    }

    FilterReport report;
    const std::vector<Trunk> trunks = findTrunks(cloud, filter_, &report);
    const RowFit fit = fitRow(trunks, fit_);

    publishTrunks(msg.header, trunks);

    kraken_interfaces::msg::RowEstimate estimate;
    estimate.header = msg.header;
    estimate.valid = fit.valid;
    estimate.reason = fit.reason;
    estimate.width = fit.width;
    estimate.lateral_offset = fit.lateral_offset;
    estimate.row_heading = fit.row_heading;
    estimate.left_trunks = fit.left_trunks;
    estimate.right_trunks = fit.right_trunks;
    row_publisher_->publish(estimate);

    if (!fit.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no row here: %s (%zu returns, %zu above ground, %zu voxels, %zu clusters, %zu trunks)",
        fit.reason.c_str(), report.received, report.above_ground, report.voxels,
        report.clusters, report.trunks);
      return;
    }
    path_publisher_->publish(centreline(msg.header, fit));
  }

  void publishTrunks(const std_msgs::msg::Header & header, const std::vector<Trunk> & trunks)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(trunks.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const Trunk & trunk : trunks) {
      *x = static_cast<float>(trunk.centre.x);
      *y = static_cast<float>(trunk.centre.y);
      *z = 0.0F;
      ++x;
      ++y;
      ++z;
    }
    trunk_publisher_->publish(cloud);
  }

  nav_msgs::msg::Path centreline(
    const std_msgs::msg::Header & header, const RowFit & fit) const
  {
    nav_msgs::msg::Path path;
    path.header = header;
    const double forward_x = std::cos(fit.row_heading);
    const double forward_y = std::sin(fit.row_heading);
    // Starts behind the sensor so the path still reaches the axle the
    // controller steers about, 2.28 m astern of the lidar.
    for (double distance = -behind_; distance <= ahead_; distance += step_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = -forward_y * fit.lateral_offset + forward_x * distance;
      pose.pose.position.y = forward_x * fit.lateral_offset + forward_y * distance;
      pose.pose.orientation.z = std::sin(0.5 * fit.row_heading);
      pose.pose.orientation.w = std::cos(0.5 * fit.row_heading);
      path.poses.push_back(pose);
    }
    return path;
  }

  CloudFilterOptions filter_;
  RowFitOptions fit_;
  double ahead_{10.0};
  double behind_{3.0};
  double step_{0.5};

  rclcpp::Publisher<kraken_interfaces::msg::RowEstimate>::SharedPtr row_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trunk_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace kraken_orchard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kraken_orchard::RowFollower>());
  rclcpp::shutdown();
  return 0;
}
