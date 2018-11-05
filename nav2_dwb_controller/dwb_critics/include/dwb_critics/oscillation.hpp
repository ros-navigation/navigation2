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

#ifndef DWB_CRITICS__OSCILLATION_HPP_
#define DWB_CRITICS__OSCILLATION_HPP_

#include <vector>
#include <string>
#include "dwb_core/trajectory_critic.hpp"

namespace dwb_critics
{

/**
 * @class OscillationCritic
 * @brief Checks to see whether the sign of the commanded velocity flips frequently
 *
 * This critic figures out if the commanded trajectories are oscillating by seeing
 * if one of the dimensions (x,y,theta) flips from positive to negative and then back
 * (or vice versa) without moving sufficiently far or waiting sufficiently long.
 *
 * Scenario 1: Robot moves one meter forward, and then two millimeters backward.
 * Another forward motion would be considered oscillating, since the x dimension would then
 * flip from positive to negative and then back to negative. Hence, when scoring different
 * trajectories, positive velocity commands will get the oscillation_score (-5.0, or invalid)
 * and only negative velocity commands will be considered valid.

 * Scenario 2: Robot moves one meter forward, and then one meter backward.
 * The robot has thus moved one meter since flipping the sign of the x direction, which
 * is greater than our oscillation_reset_dist, so its not considered oscillating, so all
 * trajectories are considered valid.
 *
 * Note: The critic will only check oscillations in the x dimension while it exceeds
 * a particular value (x_only_threshold_). If it dips below that magnitude, it will
 * also check for oscillations in the y and theta dimensions. If x_only_threshold_ is
 * negative, then the critic will always check all dimensions.
 *
 * Implementation Details:
 * The critic saves the robot's current position when it prepares, and what the actual
 * commanded velocity was during the debrief step. Upon debriefing, if the sign of any of
 * dimensions has flipped since the last command, the position is saved as prev_stationary_pose_.
 *
 * If the linear or angular distance from prev_stationary_pose_ to the current pose exceeds
 * the limits, the oscillation flags are reset so the previous sign change is no longer remembered.
 * This assumes that oscillation_reset_dist_ or oscillation_reset_angle_ are positive. Otherwise,
 * it uses a time based delay reset function.
 */
class OscillationCritic : public dwb_core::TrajectoryCritic
{
public:
  OscillationCritic()
  : oscillation_reset_time_(0) {}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override;
  void debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

private:
  /**
   * @class CommandTrend
   * @brief Helper class for performing the same logic on the x,y and theta dimensions
   */
  class CommandTrend
  {
public:
    CommandTrend();
    void reset();

    /**
     * @brief update internal flags based on the commanded velocity
     * @param velocity commanded velocity for the dimension this trend is tracking
     * @return true if the sign has flipped
     */
    bool update(double velocity);

    /**
     * @brief Check to see whether the proposed velocity would be considered oscillating
     * @param velocity the velocity to evaluate
     * @return true if the sign has flipped more than once
     */
    bool isOscillating(double velocity);

    /**
     * @brief Check whether we are currently tracking a flipped sign
     * @return True if the sign has flipped
     */
    bool hasSignFlipped();

private:
    // Simple Enum for Tracking
    // cppcheck-suppress syntaxError
    enum class Sign { ZERO, POSITIVE, NEGATIVE };

    Sign sign_;
    bool positive_only_, negative_only_;
  };

  /**
   * @brief Given a command that has been selected, track each component's sign for oscillations
   * @param cmd_vel The command velocity selected by the algorithm
   * @return True if the sign on any of the components flipped
   */
  bool setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  /**
   * @brief Return true if the robot has travelled far enough or waited long enough
   */
  bool resetAvailable();

  CommandTrend x_trend_, y_trend_, theta_trend_;
  double oscillation_reset_dist_, oscillation_reset_angle_, x_only_threshold_;
  rclcpp::Duration oscillation_reset_time_;

  // Cached square parameter
  double oscillation_reset_dist_sq_;

  // Saved positions
  geometry_msgs::msg::Pose2D pose_, prev_stationary_pose_;
  // Saved timestamp
  rclcpp::Time prev_reset_time_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__OSCILLATION_HPP_
