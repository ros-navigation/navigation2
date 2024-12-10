/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Joseph Duchesne
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_controller/plugins/path_complete_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

PathCompleteGoalChecker::PathCompleteGoalChecker()
: SimpleGoalChecker(), path_length_tolerance_(1)
{
}

void PathCompleteGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".path_length_tolerance",
    rclcpp::ParameterValue(path_length_tolerance_));

  node->get_parameter(plugin_name + ".path_length_tolerance", path_length_tolerance_);

  // Replace SimpleGoalChecker's callback for dynamic parameters
  node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PathCompleteGoalChecker::dynamicParametersCallback, this, _1));
}

void PathCompleteGoalChecker::reset()
{
  SimpleGoalChecker::reset();
}

bool PathCompleteGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & twist, const nav_msgs::msg::Path & path)
{
  // return false if more than path_length_tolerance_ waypoints exist
  // note: another useful version of this could check path length
  if (path.poses.size() > (unsigned int)path_length_tolerance_) {
    return false;
  }
  // otherwise defer to SimpleGoalChecker's isGoalReached
  return SimpleGoalChecker::isGoalReached(query_pose, goal_pose, twist, path);
}

rcl_interfaces::msg::SetParametersResult
PathCompleteGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  // call the base class (might be unnessesary since the base class will already bind this event)
  rcl_interfaces::msg::SetParametersResult result =
    SimpleGoalChecker::dynamicParametersCallback(parameters);
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".path_length_tolerance") {
        path_length_tolerance_ = parameter.as_int();
      }
    }
  }
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::PathCompleteGoalChecker, nav2_core::GoalChecker)
