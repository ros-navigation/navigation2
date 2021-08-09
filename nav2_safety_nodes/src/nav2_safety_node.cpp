#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <algorithm>
#include <limits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "laser_geometry/laser_geometry.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_safety_nodes/nav2_safety_node.hpp"

using namespace std::chrono_literals;

namespace nav2_safety_nodes
{

SafetyZone::SafetyZone()
: nav2_util::LifecycleNode("SafetyZone", "", false)
{
  logger_ = get_logger();
  RCLCPP_INFO(logger_, "Creating Safety Polygon");

  // pass polygon parameters at string
  declare_parameter("safety_polygon", rclcpp::ParameterValue(std::string("[]")));
  declare_parameter("zone_action", rclcpp::ParameterValue(0.0));
  declare_parameter("zone_priority", rclcpp::ParameterValue(1));
  declare_parameter("zone_num_pts", rclcpp::ParameterValue(1));
  declare_parameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter("tf_tolerance", rclcpp::ParameterValue(0.01));
  declare_parameter("queue_size", rclcpp::ParameterValue(static_cast<int>(std::thread::hardware_concurrency())));

  tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
}

SafetyZone::~SafetyZone()
{
}

nav2_util::CallbackReturn
SafetyZone::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring");
  getParameters();
  initTransforms();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating");
  initPubSub();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  publisher_.reset();
  subscriber_.reset();
  safety_polygon_pub_.reset();
  timer_.reset();
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
  safety_polygon_ = get_parameter("safety_polygon").as_string();
  zone_action_ = get_parameter("zone_action").as_double();
  zone_priority_ = get_parameter("zone_priority").as_int();
  zone_num_pts_ = get_parameter("zone_num_pts").as_int();
  base_frame_ = get_parameter("base_frame").as_string();
  tf_tolerance_ = get_parameter("tf_tolerance").as_double();
  input_queue_size_ = get_parameter("queue_size").as_int();

  // If the safety_polygon has been specified, it must be in the correct format
  if (safety_polygon_ != "" && safety_polygon_ != "[]") {
    // Polygon parameter has been specified, polygon -> point vector(safety_zone)
    std::vector<geometry_msgs::msg::Point> safety_zone;
    makeVectorPointsFromString(safety_polygon_, safety_zone);
  } else {
    // Polygon provided but invalid, so stay with the radius
    RCLCPP_ERROR(
      logger_, "The safety_polygon is invalid: \"%s\" :) ",
      safety_polygon_.c_str());
  }
}

void
SafetyZone::initTransforms()
{
  RCLCPP_INFO(get_logger(), "initTransforms");

  // Initialize transform listener and broadcaster
  tf2_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf2_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
}

// Publishers and subscribers
void
SafetyZone::initPubSub()
{
  RCLCPP_INFO(logger_, "initPubSub");
  // Create the publishers and subscribers
  safety_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    "published_polygon", rclcpp::SystemDefaultsQoS());
  // Pointcloud publisher  
  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS());
  // Laserscan subscriber
  subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", rclcpp::SystemDefaultsQoS(),
    std::bind(&SafetyZone::laser_callback, this, std::placeholders::_1));
  // Velocity publisher
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
  // Timer -> 10hzs
  timer_ = create_wall_timer(
    100ms, std::bind(&SafetyZone::timer_callback, this));

  RCLCPP_INFO(logger_, "Subscribed to laser topic.");
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
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  // project the laser into a point cloud
  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  projector_.projectLaser(*message, *cloud_msg);
  
  // Transform cloud if necessary
  if (!base_frame_.empty() && cloud_msg->header.frame_id != base_frame_) {
    try {
      initTransforms();
      *cloud_msg = tf2_->transform(*cloud_msg, base_frame_, tf2::durationFromSec(tf_tolerance_));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(logger_, "Transform failure: " << ex.what());
      return;
    }
    point_cloud_pub_->publish(std::move(cloud_msg));
  }
}

void
SafetyZone::timer_callback()
{
  // while (!*cloud_msg.empty()){

  // }
}

}  // namespace nav2_safety_nodes
