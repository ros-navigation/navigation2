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

#include "dwb_plugins/standard_traj_generator.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include "dwb_plugins/xy_theta_iterator.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

using nav_2d_utils::loadParameterWithDeprecation;

namespace dwb_plugins
{

void StandardTrajectoryGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  kinematics_handler_ = std::make_shared<KinematicsHandler>();
  kinematics_handler_->initialize(nh, plugin_name_);
  initializeIterator(nh);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".sim_time", rclcpp::ParameterValue(1.7));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".discretize_by_time", rclcpp::ParameterValue(false));

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".time_granularity", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".linear_granularity", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".angular_granularity", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".include_last_point", rclcpp::ParameterValue(true));

  /*
   * If discretize_by_time, then sim_granularity represents the amount of time that should be between
   *  two successive points on the trajectory.
   *
   * If discretize_by_time is false, then sim_granularity is the maximum amount of distance between
   *  two successive points on the trajectory, and angular_sim_granularity is the maximum amount of
   *  angular distance between two successive points.
   */
  nh->get_parameter(plugin_name + ".sim_time", sim_time_);
  nh->get_parameter(plugin_name + ".discretize_by_time", discretize_by_time_);
  nh->get_parameter(plugin_name + ".time_granularity", time_granularity_);
  nh->get_parameter(plugin_name + ".linear_granularity", linear_granularity_);
  nh->get_parameter(plugin_name + ".angular_granularity", angular_granularity_);
  nh->get_parameter(plugin_name + ".include_last_point", include_last_point_);
}

void StandardTrajectoryGenerator::initializeIterator(
  const nav2_util::LifecycleNode::SharedPtr & nh)
{
  velocity_iterator_ = std::make_shared<XYThetaIterator>();
  velocity_iterator_->initialize(nh, kinematics_handler_, plugin_name_);
}

void StandardTrajectoryGenerator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  velocity_iterator_->startNewIteration(current_velocity, sim_time_);
}

bool StandardTrajectoryGenerator::hasMoreTwists()
{
  return velocity_iterator_->hasMoreTwists();
}

nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::nextTwist()
{
  return velocity_iterator_->nextTwist();
}

std::vector<double> StandardTrajectoryGenerator::getTimeSteps(
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  std::vector<double> steps;
  if (discretize_by_time_) {
    steps.resize(ceil(sim_time_ / time_granularity_));
  } else {  // discretize by distance
    double vmag = hypot(cmd_vel.x, cmd_vel.y);

    // the distance the robot would travel in sim_time if it did not change velocity
    double projected_linear_distance = vmag * sim_time_;

    // the angle the robot would rotate in sim_time
    double projected_angular_distance = fabs(cmd_vel.theta) * sim_time_;

    // Pick the maximum of the two
    int num_steps = ceil(
      std::max(
        projected_linear_distance / linear_granularity_,
        projected_angular_distance / angular_granularity_));
    steps.resize(num_steps);
  }
  if (steps.size() == 0) {
    steps.resize(1);
  }
  std::fill(steps.begin(), steps.end(), sim_time_ / steps.size());
  return steps;
}

dwb_msgs::msg::Trajectory2D StandardTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose,
  const nav_2d_msgs::msg::Twist2D & start_vel,
  const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.velocity = cmd_vel;
  //  simulate the trajectory
  geometry_msgs::msg::Pose2D pose = start_pose;
  nav_2d_msgs::msg::Twist2D vel = start_vel;
  double running_time = 0.0;
  std::vector<double> steps = getTimeSteps(cmd_vel);
  traj.poses.push_back(start_pose);
  for (double dt : steps) {
    //  calculate velocities
    vel = computeNewVelocity(cmd_vel, vel, dt);

    //  update the position of the robot using the velocities passed in
    pose = computeNewPosition(pose, vel, dt);

    traj.poses.push_back(pose);
    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
    running_time += dt;
  }  //  end for simulation steps

  if (include_last_point_) {
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(rclcpp::Duration::from_seconds(running_time));
  }

  return traj;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
nav_2d_msgs::msg::Twist2D StandardTrajectoryGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D & cmd_vel,
  const nav_2d_msgs::msg::Twist2D & start_vel, const double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  nav_2d_msgs::msg::Twist2D new_vel;
  new_vel.x = projectVelocity(
    start_vel.x, kinematics.getAccX(),
    kinematics.getDecelX(), dt, cmd_vel.x);
  new_vel.y = projectVelocity(
    start_vel.y, kinematics.getAccY(),
    kinematics.getDecelY(), dt, cmd_vel.y);
  new_vel.theta = projectVelocity(
    start_vel.theta,
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, cmd_vel.theta);
  return new_vel;
}

geometry_msgs::msg::Pose2D StandardTrajectoryGenerator::computeNewPosition(
  const geometry_msgs::msg::Pose2D start_pose,
  const nav_2d_msgs::msg::Twist2D & vel, const double dt)
{
  geometry_msgs::msg::Pose2D new_pose;
  new_pose.x = start_pose.x +
    (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;
  new_pose.y = start_pose.y +
    (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;
  new_pose.theta = start_pose.theta + vel.theta * dt;
  return new_pose;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(
  dwb_plugins::StandardTrajectoryGenerator,
  dwb_core::TrajectoryGenerator)
