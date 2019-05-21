// Copyright (c) 2019 Steve Macenski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_safety_monitor/sensors/laser.hpp"

namespace nav2_safety_monitor
{

LaserSensor::LaserSensor(rclcpp::Node::SharedPtr & node, std::string topic)
: SafetySensor(node, topic), tf_(nullptr),
  safety_timeout_(0.0, 0.0), scan_timeout_(0.0, 0.0)
{
  tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, std::bind(&LaserSensor::laserCallback,
    this, std::placeholders::_1));
  safety_zone_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "safety_zone", 1);
  collision_zone_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "collision_zone", 1);

  double safety_timeout_sec, scan_timeout_sec;
  node_->get_parameter_or<double>("safety_timeout", safety_timeout_sec, 120.0);
  node_->get_parameter_or<double>("scan_timeout", scan_timeout_sec, 10.0);
  node_->get_parameter_or<std::string>("global_frame", global_frame_, "base_link");
  safety_timeout_ = rclcpp::Duration(safety_timeout_sec, 0.0);
  scan_timeout_ = rclcpp::Duration(scan_timeout_sec, 0.0);

  getSafetyZone();
}

void
LaserSensor::process()
{
  bool slow_pt = false;
  double x = 0.0, y = 0.0;

  // project the scan into a point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = current_scan_->header;
  try {
    projector_.transformLaserScanToPointCloud(global_frame_, *current_scan_,
      cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(),
      "High fidelity enabled, but TF returned a "
      "failed transform, trying low fidelity.");
    projector_.projectLaser(*current_scan_, cloud);
  }

  // find if any point is in a region
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  for ( ; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    x = *iter_x;
    y = *iter_y;
    if (isPtInCollisionZone(x,y)) {
      if (current_state_ != SafetyState::COLLISION) {
        collision_start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(),"Stopping due to imminent collision!");
      }
      current_state_ = SafetyState::COLLISION;
      return;
    } else if (isPtInSafetyZone(x,y)) {
      slow_pt = true;
    }
  }

  if (slow_pt) {
    current_state_ = SafetyState::SLOW;
  } else {
    current_state_ = SafetyState::FREE;
  }

  // publish debug visualizations
  poly_safety_.header.stamp = node_->now();
  poly_collision_.header.stamp = node_->now();
  safety_zone_pub_->publish(poly_safety_);
  collision_zone_pub_->publish(poly_collision_);
}

SafetyState
LaserSensor::getState()
{
  if (current_state_ == SafetyState::UNKNOWN) {
    return SafetyState::FREE;
  }

  if (node_->now() - current_scan_->header.stamp > scan_timeout_) {
    return SafetyState::FREE;
  }

  if (current_state_ == SafetyState::COLLISION) {
    if (node_->now() - collision_start_time_ < safety_timeout_) {
      return SafetyState::COLLISION;
    } else {
      return SafetyState::FREE;
    }
  }

  return current_state_;
}

void
LaserSensor::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  current_scan_ = msg;
}

void
LaserSensor::getSafetyZone()
{
  // TODO(stevemacenski) allow for multiple zones
  rclcpp::Parameter safety_param = node_->get_parameter("safety_zone");
  rclcpp::Parameter collision_param = node_->get_parameter("collision_zone");
  try {
    std::vector<double> safety_info = safety_param.as_double_array();
    std::vector<double> collision_info = collision_param.as_double_array();
    if (safety_info.size() != 4 || collision_info.size() != 4) {
      RCLCPP_WARN(node_->get_logger(), "Collision or safety zone params "
        "are malformed. Must be array size 4 of values: "
        "[wid, len, offset_x, offset_y]"
        " with respect to the laser coordinates."
        "Defaulting to sizes 0.");
      safety_info.clear();
      collision_info.clear();
      safety_info.resize(4, 0.0);
      collision_info.resize(4, 0.0);
    }

    safety_zone_ = LaserBoxes(safety_info);
    collision_zone_ = LaserBoxes(collision_info);
  } catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to get safety zones from file."
      "Exiting.");
    exit(-1);
  }

  populateZoneMessages();
  return;
}

void
LaserSensor::setZones(const LaserBoxes & safety_zone,
  const LaserBoxes & collision_zone)
{
  RCLCPP_INFO(node_->get_logger(), "Setting new safety regions.");
  safety_zone_ = safety_zone;
  collision_zone_ = collision_zone;
  populateZoneMessages();
}

void
LaserSensor::populateZoneMessages()
{
  poly_safety_.header.frame_id = global_frame_;
  poly_safety_.header.stamp = node_->now();
  poly_collision_.header = poly_safety_.header;

  poly_safety_.polygon.points.resize(5);
  poly_collision_.polygon.points.resize(5);

  double half_width = safety_zone_.width / 2.0;
  double half_length = safety_zone_.length / 2.0;
  double off_x = safety_zone_.offset_x;
  double off_y = safety_zone_.offset_y;

  poly_safety_.polygon.points.at(0).x = off_x - half_length;
  poly_safety_.polygon.points.at(0).y = off_y + half_width;
  poly_safety_.polygon.points.at(0).z = 0.05;
  poly_safety_.polygon.points.at(1).x = off_x + half_length;
  poly_safety_.polygon.points.at(1).y = off_y + half_width;
  poly_safety_.polygon.points.at(1).z = 0.05;
  poly_safety_.polygon.points.at(2).x = off_x + half_length;
  poly_safety_.polygon.points.at(2).y = off_y - half_width;
  poly_safety_.polygon.points.at(2).z = 0.05;
  poly_safety_.polygon.points.at(3).x = off_x - half_length;
  poly_safety_.polygon.points.at(3).y = off_y - half_width;
  poly_safety_.polygon.points.at(3).z = 0.05;
  poly_safety_.polygon.points.at(4) = poly_safety_.polygon.points.at(0);

  half_width = collision_zone_.width / 2.0;
  half_length = collision_zone_.length / 2.0;
  off_x = collision_zone_.offset_x;
  off_y = collision_zone_.offset_y;

  poly_collision_.polygon.points.at(0).x = off_x - half_length;
  poly_collision_.polygon.points.at(0).y = off_y + half_width;
  poly_collision_.polygon.points.at(0).z = 0.05;
  poly_collision_.polygon.points.at(1).x = off_x + half_length;
  poly_collision_.polygon.points.at(1).y = off_y + half_width;
  poly_collision_.polygon.points.at(1).z = 0.05;
  poly_collision_.polygon.points.at(2).x = off_x + half_length;
  poly_collision_.polygon.points.at(2).y = off_y - half_width;
  poly_collision_.polygon.points.at(2).z = 0.05;
  poly_collision_.polygon.points.at(3).x = off_x - half_length;
  poly_collision_.polygon.points.at(3).y = off_y - half_width;
  poly_collision_.polygon.points.at(3).z = 0.05;
  poly_collision_.polygon.points.at(4) = poly_collision_.polygon.points.at(0);
}

bool
LaserSensor::isPtInCollisionZone(const double & x, const double & y)
{
  return IsPtInZone(x, y, collision_zone_);
}

bool
LaserSensor::isPtInSafetyZone(const double & x, const double & y)
{
  return IsPtInZone(x, y, safety_zone_);
}

bool LaserSensor::IsPtInZone(const double & x, const double & y,
  const LaserBoxes & region)
{
  if (x > region.offset_x - region.length/2.0 &&
    x < region.offset_x + region.length/2.0) {
    if (y > region.offset_y - region.width/2.0 &&
      y < region.offset_y + region.width/2.0) {
      return true;
    }
  }
  return false;
}

} // end namespace nav2_safety_monitor
