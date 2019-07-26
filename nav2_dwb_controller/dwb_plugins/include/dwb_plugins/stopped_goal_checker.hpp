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

#ifndef DWB_PLUGINS__STOPPED_GOAL_CHECKER_HPP_
#define DWB_PLUGINS__STOPPED_GOAL_CHECKER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "dwb_plugins/simple_goal_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that checks the position difference and velocity
 */
class StoppedGoalChecker : public SimpleGoalChecker
{
public:
  StoppedGoalChecker();
  // Standard GoalChecker Interface
  void initialize(const nav2_util::LifecycleNode::SharedPtr & nh) override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose2D & query_pose, const geometry_msgs::msg::Pose2D & goal_pose,
    const nav_2d_msgs::msg::Twist2D & velocity) override;

protected:
  double rot_stopped_velocity_, trans_stopped_velocity_;
};

}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__STOPPED_GOAL_CHECKER_HPP_
