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

#ifndef DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_
#define DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "dwb_plugins/velocity_iterator.hpp"
#include "dwb_plugins/kinematic_parameters.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{

/**
 * @class StandardTrajectoryGenerator
 * @brief Standard DWA-like trajectory generator.
 */
class StandardTrajectoryGenerator : public dwb_core::TrajectoryGenerator
{
public:
  // Standard TrajectoryGenerator interface
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;

  dwb_msgs::msg::Trajectory2D generateTrajectory(
    const geometry_msgs::msg::Pose2D & start_pose,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const nav_2d_msgs::msg::Twist2D & cmd_vel) override;

protected:
  /**
   * @brief Initialize the VelocityIterator pointer. Put in its own function for easy overriding
   */
  virtual void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & nh);

  /**
   * @brief Calculate the velocity after a set period of time, given the desired velocity and acceleration limits
   *
   * @param cmd_vel Desired velocity
   * @param start_vel starting velocity
   * @param dt amount of time in seconds
   * @return new velocity after dt seconds
   */
  virtual nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel, const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt);

  /**
   * @brief Use the robot's kinematic model to predict new positions for the robot
   *
   * @param start_pose Starting pose
   * @param vel Actual robot velocity (assumed to be within acceleration limits)
   * @param dt amount of time in seconds
   * @return New pose after dt seconds
   */
  virtual geometry_msgs::msg::Pose2D computeNewPosition(
    const geometry_msgs::msg::Pose2D start_pose, const nav_2d_msgs::msg::Twist2D & vel,
    const double dt);


  /**
   * @brief Compute an array of time deltas between the points in the generated trajectory.
   *
   * @param cmd_vel The desired command velocity
   * @return vector of the difference between each time step in the generated trajectory
   *
   * If we are discretizing by time, the returned vector will be the same constant time_granularity
   * for all cmd_vels. Otherwise, you will get times based on the linear/angular granularity.
   *
   * Right now the vector contains a single value repeated many times, but this method could be overridden
   * to allow for dynamic spacing
   */
  virtual std::vector<double> getTimeSteps(const nav_2d_msgs::msg::Twist2D & cmd_vel);

  KinematicsHandler::Ptr kinematics_handler_;
  std::shared_ptr<VelocityIterator> velocity_iterator_;

  double sim_time_;

  // Sampling Parameters
  bool discretize_by_time_;

  /// @brief If discretizing by time, the amount of time between each point in the traj
  double time_granularity_;

  /// @brief If not discretizing by time, the amount of linear space between points
  double linear_granularity_;

  /// @brief If not discretizing by time, the amount of angular space between points
  double angular_granularity_;

  /// @brief the name of the overlying plugin ID
  std::string plugin_name_;

  /* Backwards Compatibility Parameter: include_last_point
   *
   * dwa had an off-by-one error built into it.
   * It generated N trajectory points, where N = ceil(sim_time / time_delta).
   * If for example, sim_time=3.0 and time_delta=1.5, it would generate trajectories with 2 points, which
   * indeed were time_delta seconds apart. However, the points would be at t=0 and t=1.5, and thus the
   * actual sim_time was much less than advertised.
   *
   * This is remedied by adding one final point at t=sim_time, but only if include_last_point_ is true.
   *
   * Nothing I could find actually used the time_delta variable or seemed to care that the trajectories
   * were not projected out as far as they intended.
   */
  bool include_last_point_;
};


}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__STANDARD_TRAJ_GENERATOR_HPP_
