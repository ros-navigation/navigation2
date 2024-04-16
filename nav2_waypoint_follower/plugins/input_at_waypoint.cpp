// Copyright (c) 2020 Samsung Research America
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

#include "nav2_waypoint_follower/plugins/input_at_waypoint.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace nav2_waypoint_follower
{

using std::placeholders::_1;

InputAtWaypoint::InputAtWaypoint()
: input_received_(false),
  is_enabled_(true),
  timeout_(10.0, 0.0)
{
}

InputAtWaypoint::~InputAtWaypoint()
{
}

void InputAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();

  if (!node) {
    throw std::runtime_error{"Failed to lock node in input at waypoint plugin!"};
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double timeout;
  std::string input_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".timeout",
    rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".input_topic",
    rclcpp::ParameterValue("input_at_waypoint/input"));
  timeout = node->get_parameter(plugin_name + ".timeout").as_double();
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".input_topic", input_topic);

  timeout_ = rclcpp::Duration(timeout, 0.0);

  RCLCPP_INFO(
    logger_, "InputAtWaypoint: Subscribing to input topic %s.", input_topic.c_str());
  subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    input_topic, 1, std::bind(&InputAtWaypoint::Cb, this, _1));
}

void InputAtWaypoint::Cb(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_received_ = true;
}

bool InputAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }

  input_received_ = false;

  rclcpp::Time start = clock_->now();
  rclcpp::Rate r(50);
  bool input_received = false;
  while (clock_->now() - start < timeout_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input_received = input_received_;
    }

    if (input_received) {
      return true;
    }

    r.sleep();
  }

  RCLCPP_WARN(
    logger_, "Unable to get external input at wp %i. Moving on.", curr_waypoint_index);
  return false;
}

}  // namespace nav2_waypoint_follower

PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::InputAtWaypoint,
  nav2_core::WaypointTaskExecutor)
