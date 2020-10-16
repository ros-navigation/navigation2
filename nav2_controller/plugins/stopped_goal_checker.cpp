/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#include <cmath>
#include <string>
#include <memory>
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::hypot;
using std::fabs;

namespace nav2_controller
{

StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  SimpleGoalChecker::initialize(nh, plugin_name);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.25));

  nh->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  nh->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
}

bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  return fabs(velocity.angular.z) <= rot_stopped_velocity_ &&
         hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::StoppedGoalChecker, nav2_core::GoalChecker)
