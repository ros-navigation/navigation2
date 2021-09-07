#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <utility>

#include "nav2_safety_nodes/nav2_safety_node.hpp"

using namespace std::chrono_literals;

namespace nav2_safety_nodes
{

SafetyZone::SafetyZone()
: nav2_util::LifecycleNode("SafetyZone", "", false)
{
  logger_ = get_logger();
  RCLCPP_INFO(logger_, "Creating Safety Polygon");

  // Vector of string for multiple LaserScan topics
  const std::vector<std::string> scan{};
  // pass polygon parameters at string
  declare_parameter("safety_polygon", std::string("[]"));
  declare_parameter("zone_action", rclcpp::ParameterValue(0.0));
  declare_parameter("zone_priority", rclcpp::ParameterValue(1));
  declare_parameter("zone_num_pts", rclcpp::ParameterValue(1));
  declare_parameter("base_frame", std::string("base_link"));
  declare_parameter("tf_tolerance", rclcpp::ParameterValue(0.01));
  declare_parameter("scan_topics", scan);
  declare_parameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
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
  safety_polygon_pub_->on_activate();
  point_cloud_pub_->on_activate();
  speed_limit_pub_->on_activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating");
  safety_polygon_pub_->on_deactivate();
  point_cloud_pub_->on_deactivate();
  speed_limit_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SafetyZone::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up");
  safety_polygon_pub_.reset();
  point_cloud_pub_.reset();
  speed_limit_pub_.reset();
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
  std::string speed_limit_topic;
  // Get all of the required parameters
  safety_polygon_ = get_parameter("safety_polygon").as_string();
  zone_action_ = get_parameter("zone_action").as_double();
  zone_priority_ = get_parameter("zone_priority").as_int();
  zone_num_pts_ = get_parameter("zone_num_pts").as_int();
  base_frame_ = get_parameter("base_frame").as_string();
  tf_tolerance_ = get_parameter("tf_tolerance").as_double();
  scan_topics_ = get_parameter("scan_topics").as_string_array();
  speed_limit_topic_ = get_parameter("speed_limit_topic").as_string();

  // If the safety_polygon has been specified, it must be in the correct format
  if (safety_polygon_ != "" && safety_polygon_ != "[]") {
    // Polygon parameter has been specified, polygon -> point vector(safety_zone)
    std::vector<geometry_msgs::msg::Point> safety_zone;
    /// string of polygon points and returns a polygon vector
    nav2_util::makeVectorPointsFromString(safety_polygon_, safety_zone);
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
  // Velocity publisher
  speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic_, rclcpp::QoS(10));

  // Multiple Laserscan subscribers
  scan_subscribers_.reserve(scan_topics_.size());
  RCLCPP_INFO(logger_, "Subscribing to scan topics");
  for (unsigned int i = 0; i < scan_topics_.size(); i++) {
    scan_subscribers_.push_back(
      create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topics_[i].c_str(), rclcpp::SystemDefaultsQoS(),
        std::bind(&SafetyZone::laserCallback, this, std::placeholders::_1)));
  }
  // Timer -> 10hzs
  timer_ = create_wall_timer(
    100ms, std::bind(&SafetyZone::timerCallback, this));
  RCLCPP_INFO(logger_, "Subscribed to scan topics");
}

void
SafetyZone::laserCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  // project the laser into a point cloud
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud;

   try {
    projector_.projectLaser(*message, *cloud);
  } catch (...) {
    RCLCPP_WARN(
      logger_,
      "Unable to project laser into cloud");
  }

  // Transform the cloud if necessary
  if (!base_frame_.empty() && cloud->header.frame_id != base_frame_) {
    try {
      *cloud = tf2_->transform(*cloud, base_frame_, tf2::durationFromSec(tf_tolerance_));
      sensor_data.push(*cloud);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(logger_, "Transform failure: " << ex.what());
      return;
    }
  } else if (cloud->header.frame_id == base_frame_) {
    sensor_data.push(*cloud);
  }
}

double
SafetyZone::dotProduct(const Eigen::Vector3d &pt1,
    const Eigen::Vector3d &pt2){
    return pt1[0]*pt2[1]-pt1[1]*pt2[0];
}

// Function for detecting if cloud points are inside safety polygon or not
int
SafetyZone::detectPoints(
  const sensor_msgs::msg::PointCloud2 &cloud,
  std::vector<geometry_msgs::msg::Point> safety_zone)
{
  int points_inside = 0;
  int count_same_side_results = 0, on_edge = 0;
  const int n = safety_zone.size();
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  // iterating through cloud points
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    double px = *iter_x, py = *iter_y, pz = *iter_z;
    // iterating through polygon points
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::Point a = safety_zone[i];
      geometry_msgs::msg::Point b = safety_zone[(i + 1) % n];
      Eigen::Vector3d affine_segment = {b.x - a.x, b.y - a.y, b.z - a.z};
      Eigen::Vector3d affine_point = {px - a.x, py - a.y, pz - a.z};
      double x = dotProduct(affine_segment, affine_point);
      if (x > 0) {
        count_same_side_results++;
      } else if (x < 0) {
        count_same_side_results--;
      } else {
        on_edge++;
      }
    }
    // checking if point is on same side of all edges
    if (count_same_side_results == n || count_same_side_results == (-1)*n || on_edge == 2) {
      RCLCPP_INFO(logger_, "Yes");
      points_inside++;
    } else {
      RCLCPP_INFO(logger_, "No");
    }
  }
  return points_inside;
}

void
SafetyZone::timerCallback()
{
  std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
    std::make_unique<nav2_msgs::msg::SpeedLimit>();
  while (!sensor_data.empty())
  {
    if (detectPoints(sensor_data.front(), safety_zone) >= zone_num_pts_) {
      msg->header.frame_id = base_frame_;
      msg->header.stamp = clock_->now();
      msg->speed_limit = zone_action_;
      speed_limit_pub_->publish(std::move(msg));
    }
    sensor_data.pop();
  }
}
}  // namespace nav2_safety_nodes
