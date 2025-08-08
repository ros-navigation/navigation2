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

#include "dwb_critics/goal_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dwb_critics
{

void GoalAlignCritic::onInit()
{
  GoalDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = node->declare_or_get_parameter(dwb_plugin_name_ + "." + name_ +
      ".forward_point_distance", 0.325);
}

bool GoalAlignCritic::prepare(
  const geometry_msgs::msg::Pose & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose & goal,
  const nav_msgs::msg::Path & global_plan)
{
  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end
  double angle_to_goal = atan2(goal.position.y - pose.position.y,
    goal.position.x - pose.position.x);

  nav_msgs::msg::Path target_poses = global_plan;
  target_poses.poses.back().pose.position.x += forward_point_distance_ * cos(angle_to_goal);
  target_poses.poses.back().pose.position.y += forward_point_distance_ * sin(angle_to_goal);

  return GoalDistCritic::prepare(pose, vel, goal, target_poses);
}

double GoalAlignCritic::scorePose(const geometry_msgs::msg::Pose & pose)
{
  return GoalDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalAlignCritic, dwb_core::TrajectoryCritic)
