// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "nav2_collision_monitor/circle.hpp"

#include <math.h>
#include <cmath>
#include <exception>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Circle", polygon_name_.c_str());
}

Circle::~Circle()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Circle", polygon_name_.c_str());
}

void Circle::getPolygon(std::vector<Point> & poly) const
{
  // Number of polygon points. More edges means better approximation.
  const double polygon_edges = 16;
  // Increment of angle during points position calculation
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();

  // Making new polygon looks like a circle
  Point p;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    poly.push_back(p);
  }
}

int Circle::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (Point point : points) {
    if (point.x * point.x + point.y * point.y < radius_squared_) {
      num++;
    }
  }

  return num;
}

bool Circle::isShapeSet()
{
  if (radius_squared_ == -1.0) {
    RCLCPP_WARN(logger_, "[%s]: Circle radius is not set yet", polygon_name_.c_str());
    return false;
  }
  return true;
}

bool Circle::getParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Clear the polygon subscription topic. It will be set later, if necessary.
  polygon_sub_topic.clear();

  bool use_dynamic_sub = true;  // if getting parameter radius fails, use dynamic subscription
  try {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".radius", rclcpp::PARAMETER_DOUBLE);
    radius_ = node->get_parameter(polygon_name_ + ".radius").as_double();
    radius_squared_ = radius_ * radius_;
    use_dynamic_sub = false;
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Polygon circle radius is not defined. Using dynamic subscription instead.",
      polygon_name_.c_str());
  }

  bool ret = true;
  if (!getCommonParameters(
      polygon_sub_topic, polygon_pub_topic, footprint_topic, use_dynamic_sub))
  {
    if (use_dynamic_sub && polygon_sub_topic.empty()) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Error while getting circle parameters: static radius and sub topic both not defined",
        polygon_name_.c_str());
    }
    ret = false;
  }

  // There is no footprint subscription for the Circle. Thus, set string as empty.
  footprint_topic.clear();

  return ret;
}

void Circle::createSubscription(std::string & polygon_sub_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!polygon_sub_topic.empty()) {
    RCLCPP_INFO(
      logger_,
      "[%s]: Subscribing on %s topic for polygon",
      polygon_name_.c_str(), polygon_sub_topic.c_str());
    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    if (polygon_subscribe_transient_local_) {
      polygon_qos.transient_local();
    }
    radius_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      polygon_sub_topic, polygon_qos,
      std::bind(&Circle::radiusCallback, this, std::placeholders::_1));
  }
}

void Circle::updatePolygon(double radius)
{
  // Update circle radius
  radius_ = radius;
  radius_squared_ = radius_ * radius_;

  // Create a polygon from radius and store it
  std::vector<Point> poly;
  getPolygon(poly);
  polygon_.polygon.points.clear();  // clear polygon points
  for (const Point & p : poly) {
    geometry_msgs::msg::Point32 p_s;
    p_s.x = p.x;
    p_s.y = p.y;
    // p_s.z will remain 0.0
    polygon_.polygon.points.push_back(p_s);  // add new points
  }
}

void Circle::radiusCallback(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    logger_,
    "[%s]: Polygon circle radius update has been arrived",
    polygon_name_.c_str());
  updatePolygon(msg->data);
}

}  // namespace nav2_collision_monitor
