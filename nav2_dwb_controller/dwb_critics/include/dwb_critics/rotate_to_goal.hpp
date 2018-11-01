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
#ifndef DWB_CRITICS__ROTATE_TO_GOAL_HPP_
#define DWB_CRITICS__ROTATE_TO_GOAL_HPP_

#include <string>
#include <vector>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

/**
 * @class RotateToGoalCritic
 * @brief Forces the commanded trajectories to only be rotations if within a certain distance window
 *
 * This used to be built in to the DWA Local Planner, but has been moved to a critic for consistency.
 *
 * If the current pose is within xy_goal_tolerance LINEAR distance from the goal pose, score trajectories
 * that have linear movement as invalid and prefer trajectories that will move toward the goal yaw.
 */
class RotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  bool in_window_;
  double goal_yaw_;
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;  ///< Cached squared tolerance
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__ROTATE_TO_GOAL_HPP_
