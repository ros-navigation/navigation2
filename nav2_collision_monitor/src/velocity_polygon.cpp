// Copyright (c) 2023 Dexory
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

#include "nav2_ros_common/node_utils.hpp"

namespace nav2_collision_monitor
{

VelocityPolygon::VelocityPolygon(
  const nav2::LifecycleNode::WeakPtr & node, const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  RCLCPP_INFO(logger_, "[%s]: Creating VelocityPolygon", polygon_name_.c_str());
}

VelocityPolygon::~VelocityPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying VelocityPolygon", polygon_name_.c_str());
}

bool VelocityPolygon::getParameters(
  std::string & polygon_sub_topic,
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  clock_ = node->get_clock();

  if (!getCommonParameters(polygon_sub_topic, polygon_pub_topic, footprint_topic, false)) {
    return false;
  }

  try {
    // Get velocity_polygons parameter
    std::vector<std::string> velocity_polygons =
      node->declare_or_get_parameter<std::vector<std::string>>(
        polygon_name_ + ".velocity_polygons");

    // holonomic param
    holonomic_ = node->declare_or_get_parameter(
      polygon_name_ + ".holonomic", false);

    for (std::string velocity_polygon_name : velocity_polygons) {
      // polygon points parameter
      std::vector<Point> poly;
      std::string poly_string =
        node->declare_or_get_parameter<std::string>(
          polygon_name_ + "." + velocity_polygon_name + ".points");

      if (!getPolygonFromString(poly_string, poly)) {
        return false;
      }

      // linear_min param
      double linear_min = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".linear_min");

      // linear_max param
      double linear_max = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".linear_max");

      // theta_min param
      double theta_min = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".theta_min");

      // theta_max param
      double theta_max = node->declare_or_get_parameter<double>(
        polygon_name_ + "." + velocity_polygon_name + ".theta_max");

      // direction_end_angle param and direction_start_angle param
      double direction_end_angle = 0.0;
      double direction_start_angle = 0.0;
      if (holonomic_) {
        direction_end_angle = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".direction_end_angle", M_PI);

        direction_start_angle = node->declare_or_get_parameter(
          polygon_name_ + "." + velocity_polygon_name + ".direction_start_angle", -M_PI);
      }

      SubPolygonParameter sub_polygon = {
        poly, velocity_polygon_name, linear_min, linear_max, theta_min,
        theta_max, direction_end_angle, direction_start_angle};
      sub_polygons_.push_back(sub_polygon);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "[%s]: Error while getting polygon parameters: %s", polygon_name_.c_str(),
      ex.what());
    return false;
  }
  return true;
}

void VelocityPolygon::updatePolygon(const Velocity & cmd_vel_in)
{
  for (auto & sub_polygon : sub_polygons_) {
    if (isInRange(cmd_vel_in, sub_polygon)) {
      // Set the polygon that is within the speed range
      poly_ = sub_polygon.poly_;

      // Update visualization polygon
      polygon_.polygon.points.clear();
      for (const Point & p : poly_) {
        geometry_msgs::msg::Point32 p_s;
        p_s.x = p.x;
        p_s.y = p.y;
        // p_s.z will remain 0.0
        polygon_.polygon.points.push_back(p_s);
      }
      return;
    }
  }

  // Log for uncovered velocity
  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 2.0,
    "Velocity is not covered by any of the velocity polygons. x: %.3f y: %.3f tw: %.3f ",
    cmd_vel_in.x, cmd_vel_in.y, cmd_vel_in.tw);
  return;
}

bool VelocityPolygon::isInRange(
  const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon)
{
  bool in_range =
    (cmd_vel_in.x <= sub_polygon.linear_max_ && cmd_vel_in.x >= sub_polygon.linear_min_ &&
    cmd_vel_in.tw <= sub_polygon.theta_max_ && cmd_vel_in.tw >= sub_polygon.theta_min_);

  if (holonomic_) {
    // Additionally check if moving direction in angle range(start -> end) for holonomic case
    const double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    if (sub_polygon.direction_start_angle_ <= sub_polygon.direction_end_angle_) {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ &&
        direction <= sub_polygon.direction_end_angle_);
    } else {
      in_range &=
        (direction >= sub_polygon.direction_start_angle_ ||
        direction <= sub_polygon.direction_end_angle_);
    }
  }

  return in_range;
}

}  // namespace nav2_collision_monitor
