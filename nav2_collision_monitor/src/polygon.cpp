// Copyright (c) 2022 Samsung Research Russia
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

#include "nav2_collision_monitor/polygon.hpp"

#include <memory>
#include <exception>
#include <utility>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Polygon::Polygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name)
: node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING), slowdown_ratio_(0.0)
{
}

Polygon::~Polygon()
{
  poly_.clear();
}


bool Polygon::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string polygon_topic;

  if (!getParameters(polygon_topic)) {
    return false;
  }

  if (visualize_) {
    // Fill polygon_ points for future usage
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point & p : poly) {
      geometry_msgs::msg::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.points.push_back(p_s);
    }

    rclcpp::QoS polygon_qos = rclcpp::SystemDefaultsQoS();  // set to default
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
      polygon_topic, polygon_qos);
  }

  return true;
}

void Polygon::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void Polygon::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

std::string Polygon::getName() const
{
  return polygon_name_;
}

ActionType Polygon::getActionType() const
{
  return action_type_;
}

int Polygon::getMaxPoints() const
{
  return max_points_;
}

double Polygon::getSlowdownRatio() const
{
  return slowdown_ratio_;
}

void Polygon::getPolygon(std::vector<Point> & poly) const
{
  poly = poly_;
}

void Polygon::setPolygon(const std::vector<geometry_msgs::msg::Point> & poly)
{
  std::size_t new_size = poly.size();
  poly_.resize(new_size);
  polygon_.points.resize(new_size);

  geometry_msgs::msg::Point32 p_s;
  for (std::size_t i = 0; i < new_size; i++) {
    poly_[i] = {poly[i].x, poly[i].y};
    p_s.x = poly[i].x;
    p_s.y = poly[i].y;
    polygon_.points[i] = p_s;
  }
}

int Polygon::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (const Point & point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

void Polygon::publish(const std::string & base_frame_id) const
{
  if (!visualize_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id;
  poly_s->polygon = polygon_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

bool Polygon::getBasicParameters(std::string & polygon_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Get action type
  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + ".action_type", rclcpp::ParameterValue("stop"));  // Stop by default
  const std::string at_str =
    node->get_parameter(polygon_name_ + ".action_type").as_string();
  if (at_str == "stop") {
    action_type_ = STOP;
  } else if (at_str == "slowdown") {
    action_type_ = SLOWDOWN;
  } else if (at_str == "approach") {
    action_type_ = APPROACH;
  } else {  // Error if something else
    RCLCPP_ERROR(logger_, "[%s]: Unknown action type: %s", polygon_name_.c_str(), at_str.c_str());
    return false;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
  max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();

  if (action_type_ == SLOWDOWN) {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
    slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
  }

  nav2_util::declare_parameter_if_not_declared(
    node, polygon_name_ + ".visualize", rclcpp::ParameterValue(false));
  visualize_ = node->get_parameter(polygon_name_ + ".visualize").as_bool();
  if (visualize_) {
    // Get polygon topic parameter in case if it is going to be published
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".polygon_topic", rclcpp::ParameterValue(polygon_name_));
    polygon_topic = node->get_parameter(polygon_name_ + ".polygon_topic").as_string();
  }

  return true;
}

bool Polygon::getParameters(std::string & polygon_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getBasicParameters(polygon_topic)) {
    return false;
  }

  if (action_type_ == APPROACH) {
    // This is robot footprint: do not need to get polygon points from ROS parameters.
    // It will be set dynamically later.
    return true;
  }

  try {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".points", rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> poly_row =
      node->get_parameter(polygon_name_ + ".points").as_double_array();
    // Check for points format correctness
    if (poly_row.size() <= 4 || poly_row.size() % 2 != 0) {
      RCLCPP_ERROR(
        logger_,
        "[%s]: Polygon has incorrect points description",
        polygon_name_.c_str());
      return false;
    }

    // Obtain polygon vertices
    Point point;
    bool first = true;
    for (double val : poly_row) {
      if (first) {
        point.x = val;
      } else {
        point.y = val;
        poly_.push_back(point);
      }
      first = !first;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

inline bool Polygon::isPointInside(const Point & point) const
{
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result

  // In ROS polygon has the last vertex is equal to first, so it is not considered
  j = poly_size - 2;
  for (i = 0; i < poly_size - 1; i++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates
    if ((point.y < poly_[i].y) != (point.y < poly_[j].y)) {
      // Calculating intersection coordinate of X+ ray
      const double x_inter = poly_[i].x +
        (point.y - poly_[i].y) * (poly_[j].x - poly_[i].x) /
        (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    j = i;
  }
  return res;
}

}  // namespace nav2_collision_monitor
