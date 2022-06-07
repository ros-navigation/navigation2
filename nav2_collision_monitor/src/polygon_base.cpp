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

#include "nav2_collision_monitor/polygon_base.hpp"
#include "nav2_collision_monitor/kinematics.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

PolygonBase::PolygonBase(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::string & base_frame_id,
  const double simulation_time_step)
: node_(node), polygon_name_(polygon_name), base_frame_id_(base_frame_id),
  action_type_(DO_NOTHING), max_points_(-1), slowdown_ratio_(0.0), time_before_collision_(-1.0),
  simulation_time_step_(simulation_time_step), visualize_(false)
{
  RCLCPP_INFO(logger_, "[%s]: Creating PolygonBase", polygon_name_.c_str());
}

PolygonBase::~PolygonBase()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying PolygonBase", polygon_name_.c_str());
}


bool PolygonBase::configure()
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
    // Fill polygon points for future usage
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

void PolygonBase::activate()
{
  if (visualize_) {
    polygon_pub_->on_activate();
  }
}

void PolygonBase::deactivate()
{
  if (visualize_) {
    polygon_pub_->on_deactivate();
  }
}

ActionType PolygonBase::getActionType()
{
  return action_type_;
}

int PolygonBase::getMaxPoints()
{
  return max_points_;
}

double PolygonBase::getSlowdownRatio()
{
  return slowdown_ratio_;
}

double PolygonBase::getTimeBeforeCollision()
{
  return time_before_collision_;
}

double PolygonBase::getCollisionTime(
  const std::vector<Point> & points, const Velocity & velocity)
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0, 0, 0};
  Velocity vel = velocity;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    std::vector<Point> points_transformed = points;
    transformPoints(pose, points_transformed);
    if (getPointsInside(points_transformed) > 0) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

Velocity PolygonBase::getSafeVelocity(
  const Velocity & velocity, const double collision_time)
{
  const double change_ratio = collision_time / time_before_collision_;
  return velocity * change_ratio;
}

void PolygonBase::publish()
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
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygon_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

bool PolygonBase::getParameters(std::string & polygon_topic)
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

  if (action_type_ == STOP) {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();
  } else if (action_type_ == SLOWDOWN) {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".max_points", rclcpp::ParameterValue(3));
    max_points_ = node->get_parameter(polygon_name_ + ".max_points").as_int();
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".slowdown_ratio", rclcpp::ParameterValue(0.5));
    slowdown_ratio_ = node->get_parameter(polygon_name_ + ".slowdown_ratio").as_double();
  } else {  // action_type_ == APPROACH
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".time_before_collision",
      rclcpp::ParameterValue(5.0));
    time_before_collision_ =
      node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
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

}  // namespace nav2_collision_monitor
