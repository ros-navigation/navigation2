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

PolygonBase::PolygonBase()
: polygon_type_(POLYGON_BASE), polygon_name_(""), action_type_(DO_NOTHING),
  stop_points_(-1), slowdown_(0.0), time_before_collision_(-1.0),
  simulation_time_step_(0.0)
{
}

PolygonBase::PolygonBase(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string polygon_name,
  const double simulation_time_step)
: node_(node), polygon_type_(POLYGON_BASE), polygon_name_(polygon_name), action_type_(DO_NOTHING),
  stop_points_(-1), slowdown_(0.0), time_before_collision_(-1.0),
  simulation_time_step_(simulation_time_step)
{
  auto node_sptr = node_.lock();
  if (node_sptr) {
    RCLCPP_INFO(node_sptr->get_logger(), "Creating PolygonBase");
  }
}

PolygonBase::~PolygonBase()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Destroying PolygonBase");
  }
}

bool PolygonBase::getParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  try {
    // Get action type
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".action_type", rclcpp::ParameterValue("stop"));  // Stop by default
    const std::string at_str =
      node->get_parameter(polygon_name_ + ".action_type").as_string();
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else {  // at_str == "approach"
      action_type_ = APPROACH;
    }

    if (action_type_ == STOP) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".stop_points", rclcpp::ParameterValue(3));
      stop_points_ = node->get_parameter(polygon_name_ + ".stop_points").as_int();
    } else if (action_type_ == SLOWDOWN) {
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".stop_points", rclcpp::ParameterValue(3));
      stop_points_ = node->get_parameter(polygon_name_ + ".stop_points").as_int();
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".slowdown", rclcpp::ParameterValue(0.5));
      slowdown_ = node->get_parameter(polygon_name_ + ".slowdown").as_double();
    } else {  // action_type_ == APPROACH
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name_ + ".time_before_collision",
        rclcpp::ParameterValue(5.0));
      time_before_collision_ =
        node->get_parameter(polygon_name_ + ".time_before_collision").as_double();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node->get_logger(), "Error while getting basic polygon parameters: %s", ex.what());
    return false;
  }

  return true;
}

PolygonType PolygonBase::getPolygonType()
{
  return polygon_type_;
}

void PolygonBase::setPolygonType(const PolygonType pt)
{
  polygon_type_ = pt;
}

ActionType PolygonBase::getActionType()
{
  return action_type_;
}

int PolygonBase::getStopPoints()
{
  return stop_points_;
}

void PolygonBase::setStopPoints(const int sp)
{
  stop_points_ = sp;
}

double PolygonBase::getSlowdown()
{
  return slowdown_;
}

void PolygonBase::setSlowdown(const double slowdown)
{
  slowdown_ = slowdown;
}

double PolygonBase::getTimeBeforeCollision()
{
  return time_before_collision_;
}

void PolygonBase::setTimeBeforeCollision(const double tbc)
{
  time_before_collision_ = tbc;
}

void PolygonBase::getCollision(
  const Point & point, const Velocity & velocity, double & coll_time)
{
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0, 0, 0};
  Velocity vel = velocity;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_; time += simulation_time_step_) {
    // NOTE: vel is changing during the simulation
    stepRobot(vel, simulation_time_step_, pose);
    Point point_fixed = point;
    transformPoint(pose, point_fixed);
    if (isPointInside(point_fixed)) {
      coll_time = time;
      return;
    }
  }

  // There is no collision
  coll_time = -1.0;
}

Velocity PolygonBase::getSafeVelocity(
  const Velocity & velocity, const double coll_time)
{
  const double change_ratio = coll_time / time_before_collision_;
  return velocity * change_ratio;
}

}  // namespace nav2_collision_monitor
