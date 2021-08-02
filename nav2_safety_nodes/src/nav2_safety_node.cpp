#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
#include <sstream>
#include <algorithm>
#include <limits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "rclcpp/time.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_safety_nodes/nav2_safety_node.hpp"

using namespace std::chrono_literals;

namespace nav2_safety_nodes{

SafetyZone::SafetyZone()
: nav2_util::LifecycleNode("SafetyZone", "", true, rclcpp::NodeOptions().arguments())
{
  logger_ = get_logger();
  RCLCPP_INFO(logger_, "Creating Safety Polygon");

  // pass polygon parameters at string
  declare_parameter("safety_polygon", rclcpp::ParameterValue(std::string("[]")));
  declare_parameter("zone_action", rclcpp::ParameterValue(0.0));
  declare_parameter("zone_priority", rclcpp::ParameterValue(1));
  declare_parameter("zone_num_pts", rclcpp::ParameterValue(1));
  declare_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
}

SafetyZone::~SafetyZone()
{
}

nav2_util::CallbackReturn
SafetyZone::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring");

  auto node = shared_from_this();
  // Getting all parameters
  getParameters();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating");
  // Create the publishers and subscribers
  safety_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
  "published_polygon", rclcpp::SystemDefaultsQoS());
  // Laserscan subscriber
  subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SafetyZone::laser_callback, this, std::placeholders::_1));
  // Velocity publisher
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Timer -> 10hz
  timer_ = create_wall_timer(
  100ms, std::bind(&SafetyZone::timer_callback, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating");
  subscriber_.reset();
  publisher_.reset();
  safety_polygon_pub_.reset();
  timer_.reset();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

  // Get Parameters
void
SafetyZone::getParameters()
{
  RCLCPP_DEBUG(logger_, " getParameters");

  // Get all of the required parameters
  get_parameter("safety_polygon", safety_polygon_).as_string();
  get_parameter("zone_action", zone_action_);
  get_parameter("zone_priority", zone_priority_);
  get_parameter("zone_num_pts", zone_num_pts_);
  get_parameter("base_frame", base_frame_);

  auto node = shared_from_this();

  // If the safety_polygon has been specified, it must be in the correct format
  if (safety_polygon_ != "" && safety_polygon_ != "[]") {
    // Polygon parameter has been specified, polygon -> point vector(safety_zone)
    std::vector<geometry_msgs::msg::Point> safety_zone_vector;
    makeVectorPointsFromString(safety_polygon_, safety_zone);
  }else {
  // Polygon provided but invalid, so stay with the radius
  RCLCPP_ERROR(
  logger_, "The safety_polygon is invalid: \"%s\" :) ",
  safety_polygon_.c_str());
  }
}

// string of polygon points and returns a polygon vector
bool
SafetyZone::makeVectorPointsFromString(
  const std::string & safety_polygon_,
  std::vector<geometry_msgs::msg::Point> & safety_zone)
{
  return nav2_util::makeVectorPointsFromString(safety_polygon_, safety_zone);
}

void
SafetyZone::laser_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr _msg)
{
  // project the laser into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = _msg->header;

  // project the scan into a point cloud
  try {
    projector_.transformLaserScanToPointCloud(_msg->header.frame_id, *_msg, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "High fidelity enabled, but TF returned a transform exception to frame.");
    projector_.projectLaser(*_msg, cloud);
  }

  sensor_msgs::msg::PointCloud2 base_frame_cloud;
  tf2_ros::Buffer & tf2_buffer_;

  // transform the point cloud to base_frame
  tf2_buffer_.transform(cloud, base_frame_cloud, base_frame_, tf_tolerance_);
  base_frame_cloud.header.stamp = cloud.header.stamp;
}


}  // namespace nav2_safety_nodes
