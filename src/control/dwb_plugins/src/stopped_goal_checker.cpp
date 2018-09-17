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

#include <dwb_plugins/stopped_goal_checker.h>
#include <pluginlib/class_list_macros.h>

namespace dwb_plugins
{

StoppedGoalChecker::StoppedGoalChecker() :
  SimpleGoalChecker(), rot_stopped_velocity_(0.25), trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(const ros::NodeHandle& nh)
{
  SimpleGoalChecker::initialize(nh);
  nh.param("rot_stopped_velocity", rot_stopped_velocity_, 0.25);
  nh.param("trans_stopped_velocity", trans_stopped_velocity_, 0.25);
}

bool StoppedGoalChecker::isGoalReached(const geometry_msgs::Pose2D& query_pose, const geometry_msgs::Pose2D& goal_pose,
                                       const nav_2d_msgs::Twist2D& velocity)
{
  bool ret = SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret)
  {
    return ret;
  }
  return fabs(velocity.theta) <= rot_stopped_velocity_
         && fabs(velocity.x) <= trans_stopped_velocity_ && fabs(velocity.y) <= trans_stopped_velocity_;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(dwb_plugins::StoppedGoalChecker, dwb_local_planner::GoalChecker)
