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
 * This used to be built in to the DWA Local Planner as the LatchedStopRotate controller,
 * but has been moved to a critic for consistency.
 *
 * The critic has three distinct phases.
 *  1) If the current pose is outside xy_goal_tolerance LINEAR distance from the goal pose, this critic
 *     will just return score 0.0.
 *  2) If within the xy_goal_tolerance and the robot is still moving with non-zero linear motion, this critic
 *     will only allow trajectories that are slower than the current speed in order to stop the robot (within
 *     the robot's acceleration limits). The returned score will be the robot's linear speed squared, multiplied
 *     by the slowing_factor parameter (default 5.0) added to the result of scoreRotation.
 *  3) If within the xy_goal_tolerance and the robot has sufficiently small linear motion, this critic will
 *     score trajectories that have linear movement as invalid and score the rest based on the result of the
 *     scoreRotation method
 *
 * The scoreRotation method can be overriden, but the default behavior is to return the shortest angular distance
 * between the goal pose and a pose from the trajectory. Which pose depends on the lookahead_time parameter.
 *  * If the lookahead_time parameter is negative, the pose evaluated will be the last pose in the trajectory,
 *    which is the same as DWA's behavior. This is the default.
 *  * Otherwise, a new pose will be projected using the dwb_local_planner::projectPose. By using a lookahead
 *    time shorter than sim_time, the critic will be less concerned about overshooting the goal yaw and thus will
 *    continue to turn faster for longer.
 */
class RotateToGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  void reset() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  /**
   * @brief Assuming that this is an actual rotation when near the goal, score the trajectory.
   *
   * This (easily overridden) method assumes that the critic is in the third phase (as described above)
   * and returns a numeric score for the trajectory relative to the goal yaw.
   * @param traj Trajectory to score
   * @return numeric score
   */
  virtual double scoreRotation(const dwb_msgs::msg::Trajectory2D & traj);

private:
  bool in_window_;
  bool rotating_;
  double goal_yaw_;
  double xy_goal_tolerance_;
  double xy_goal_tolerance_sq_;  ///< Cached squared tolerance
  double current_xy_speed_sq_, stopped_xy_velocity_sq_;
  double slowing_factor_;
  double lookahead_time_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__ROTATE_TO_GOAL_HPP_
