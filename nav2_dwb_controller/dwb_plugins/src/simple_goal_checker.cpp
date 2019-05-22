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

#include "dwb_plugins/simple_goal_checker.hpp"
#include <memory>
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"

namespace dwb_plugins
{

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25), yaw_goal_tolerance_(0.25), xy_goal_tolerance_sq_(0.0625)
{
}

void SimpleGoalChecker::initialize(const nav2_util::LifecycleNode::SharedPtr & nh)
{
  nh->declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nh->declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.25));

  nh->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
  nh->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose2D & query_pose, const geometry_msgs::msg::Pose2D & goal_pose,
  const nav_2d_msgs::msg::Twist2D &)
{
  double dx = query_pose.x - goal_pose.x,
    dy = query_pose.y - goal_pose.y;
  if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
    return false;
  }
  double dyaw = angles::shortest_angular_distance(query_pose.theta, goal_pose.theta);
  return fabs(dyaw) < yaw_goal_tolerance_;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(dwb_plugins::SimpleGoalChecker, dwb_core::GoalChecker)
