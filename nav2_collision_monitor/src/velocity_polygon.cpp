// Copyright (c) 2022 Dexory
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

#include "nav2_collision_monitor/velocity_polygon.hpp"
#include "nav2_collision_monitor/polygon.hpp"

#include "nav2_util/node_utils.hpp"


namespace nav2_collision_monitor
{

VelocityPolygon::VelocityPolygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::string & velocity_polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: node_(node), polygon_name_(polygon_name), velocity_polygon_name_(velocity_polygon_name),
  tf_buffer_(tf_buffer), base_frame_id_(base_frame_id), transform_tolerance_(transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating VelocityPolygon", velocity_polygon_name_.c_str());
}


VelocityPolygon::~VelocityPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying VelocityPolygon", velocity_polygon_name_.c_str());
  poly_.clear();
}


bool VelocityPolygon::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".points",
    rclcpp::PARAMETER_DOUBLE_ARRAY);
  std::vector<double> polygon_points = node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ + ".points").as_double_array();

  if (!Polygon::setPolygonShape(polygon_points, poly_)) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Polygon has incorrect points description",
      polygon_name_.c_str());
    return false;
  }

  // holonomic param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".holonomic",
    rclcpp::ParameterValue(false));
  holonomic_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".holonomic").as_bool();

  // polygon_sub_topic param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".polygon_sub_topic",
    rclcpp::ParameterValue("/collision_monitor/"+polygon_name_ +"/"+velocity_polygon_name_+"/set_polygon"));
  polygon_sub_topic_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".polygon_sub_topic").as_string();

  if (!polygon_sub_topic_.empty()) {
    RCLCPP_INFO(
      logger_,
      "[%s][%s]: Subscribing on %s topic for polygon",
      polygon_name_.c_str(), velocity_polygon_name_.c_str(), polygon_sub_topic_.c_str());
    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      polygon_sub_topic_, polygon_qos,
      std::bind(&VelocityPolygon::polygonCallback, this, std::placeholders::_1));
  }

  // linear_max param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".linear_max",
    rclcpp::ParameterValue(0.0));
  linear_max_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".linear_max").as_double();

  // linear_min param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".linear_min",
    rclcpp::ParameterValue(0.0));
  linear_min_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".linear_min").as_double();

  // direction_end_angle param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".direction_end_angle",
    rclcpp::ParameterValue(0.0));
  direction_end_angle_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".direction_end_angle").as_double();

  // direction_start_angle param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".direction_start_angle",
    rclcpp::ParameterValue(0.0));
  direction_start_angle_ =
    node->get_parameter(
    polygon_name_ + "." + velocity_polygon_name_ +
    ".direction_start_angle").as_double();

  // theta_max param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".theta_max", rclcpp::ParameterValue(
      0.0));
  theta_max_ =
    node->get_parameter(polygon_name_ + "." + velocity_polygon_name_ + ".theta_max").as_double();

  // theta_min param
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + "." + velocity_polygon_name_ + ".theta_min", rclcpp::ParameterValue(
      0.0));
  theta_min_ =
    node->get_parameter(polygon_name_ + "." + velocity_polygon_name_ + ".theta_min").as_double();

  return true;
}

void VelocityPolygon::polygonCallback(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    logger_,
    "[%s][%s]: Polygon shape update has been arrived",
    polygon_name_.c_str(), velocity_polygon_name_.c_str());
  updatePolygon(msg);
}

void VelocityPolygon::updatePolygon(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  std::size_t new_size = msg->polygon.points.size();

  if (new_size < 3) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Polygon should have at least 3 points",
      polygon_name_.c_str());
    return;
  }

  // Get the transform from PolygonStamped frame to base_frame_id_
  tf2::Transform tf_transform;
  if (
    !nav2_util::getTransform(
      msg->header.frame_id, base_frame_id_,
      transform_tolerance_, tf_buffer_, tf_transform))
  {
    return;
  }

  // Set main poly_ vertices first time
  poly_.resize(new_size);
  for (std::size_t i = 0; i < new_size; i++) {
    // Transform point coordinates from PolygonStamped frame -> to base frame
    tf2::Vector3 p_v3_s(msg->polygon.points[i].x, msg->polygon.points[i].y, 0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Fill poly_ array
    poly_[i] = {p_v3_b.x(), p_v3_b.y()};
  }
}



bool VelocityPolygon::isInRange(const Velocity & cmd_vel_in)
{
  if (holonomic_) {
    const double twist_linear = std::hypot(cmd_vel_in.x, cmd_vel_in.y);

    // check if direction in angle range(min -> max)
    double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    bool direction_in_range;
    if (direction_start_angle_ <= direction_end_angle_) {
      direction_in_range =
        (direction >= direction_start_angle_ && direction <= direction_end_angle_);
    } else {
      direction_in_range =
        (direction >= direction_start_angle_ || direction <= direction_end_angle_);
    }

    return twist_linear <= linear_max_ &&
           twist_linear >= linear_min_ &&
           direction_in_range &&
           cmd_vel_in.tw <= theta_max_ &&
           cmd_vel_in.tw >= theta_min_;
  } else {
    // non-holonomic
    return cmd_vel_in.x <= linear_max_ &&
           cmd_vel_in.x >= linear_min_ &&
           cmd_vel_in.tw <= theta_max_ &&
           cmd_vel_in.tw >= theta_min_;
  }
}

std::vector<Point> VelocityPolygon::getPolygon()
{
  return poly_;
}


}  // namespace nav2_collision_monitor
