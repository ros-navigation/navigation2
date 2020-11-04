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

#include <memory>
#include <string>
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_controller
{

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void SimpleGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  nh->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  nh->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  nh->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
}

void SimpleGoalChecker::reset()
{
  check_xy_ = true;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_goal_tolerance_;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::SimpleGoalChecker, nav2_core::GoalChecker)
