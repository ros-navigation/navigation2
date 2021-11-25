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

#include "dwb_critics/oscillation.hpp"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::OscillationCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{


OscillationCritic::CommandTrend::CommandTrend()
{
  reset();
}

void OscillationCritic::CommandTrend::reset()
{
  sign_ = Sign::ZERO;
  positive_only_ = false;
  negative_only_ = false;
}

bool OscillationCritic::CommandTrend::update(double velocity)
{
  bool flag_set = false;
  if (velocity < 0.0) {
    if (sign_ == Sign::POSITIVE) {
      negative_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::NEGATIVE;
  } else if (velocity > 0.0) {
    if (sign_ == Sign::NEGATIVE) {
      positive_only_ = true;
      flag_set = true;
    }
    sign_ = Sign::POSITIVE;
  }
  return flag_set;
}

bool OscillationCritic::CommandTrend::isOscillating(double velocity)
{
  return (positive_only_ && velocity < 0.0) || (negative_only_ && velocity > 0.0);
}

bool OscillationCritic::CommandTrend::hasSignFlipped()
{
  return positive_only_ || negative_only_;
}

void OscillationCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  clock_ = node->get_clock();

  oscillation_reset_dist_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_dist", 0.05);
  oscillation_reset_dist_sq_ = oscillation_reset_dist_ * oscillation_reset_dist_;
  oscillation_reset_angle_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".oscillation_reset_angle", 0.2);
  oscillation_reset_time_ = rclcpp::Duration::from_seconds(
    nav_2d_utils::searchAndGetParam(
      node,
      dwb_plugin_name_ + "." + name_ + ".oscillation_reset_time", -1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".x_only_threshold", rclcpp::ParameterValue(0.05));

  /**
   * Historical Parameter Loading
   * If x_only_threshold is set, use that.
   * If min_speed_xy is set in the namespace (as it is often used for trajectory generation), use that.
   * If min_trans_vel is set in the namespace, as it used to be used for trajectory generation, complain then use that.
   * Otherwise, set x_only_threshold_ to 0.05
   */
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".x_only_threshold", x_only_threshold_);
  // TODO(crdelsey): How to handle searchParam?
  // std::string resolved_name;
  // if (node->hasParam("x_only_threshold"))
  // {
  //   node->param("x_only_threshold", x_only_threshold_);
  // }
  // else if (node->searchParam("min_speed_xy", resolved_name))
  // {
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else if (node->searchParam("min_trans_vel", resolved_name))
  // {
  //   ROS_WARN_NAMED("OscillationCritic",
  //     "Parameter min_trans_vel is deprecated. "
  //     "Please use the name min_speed_xy or x_only_threshold instead.");
  //   node->param(resolved_name, x_only_threshold_);
  // }
  // else
  // {
  //   x_only_threshold_ = 0.05;
  // }

  reset();
}

bool OscillationCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  pose_ = pose;
  return true;
}

void OscillationCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  if (setOscillationFlags(cmd_vel)) {
    prev_stationary_pose_ = pose_;
    prev_reset_time_ = clock_->now();
  }

  // if we've got restrictions... check if we can reset any oscillation flags
  if (x_trend_.hasSignFlipped() || y_trend_.hasSignFlipped() || theta_trend_.hasSignFlipped()) {
    // Reset flags if enough time or distance has passed
    if (resetAvailable()) {
      reset();
    }
  }
}

bool OscillationCritic::resetAvailable()
{
  if (oscillation_reset_dist_ >= 0.0) {
    double x_diff = pose_.x - prev_stationary_pose_.x;
    double y_diff = pose_.y - prev_stationary_pose_.y;
    double sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (sq_dist > oscillation_reset_dist_sq_) {
      return true;
    }
  }
  if (oscillation_reset_angle_ >= 0.0) {
    double th_diff = pose_.theta - prev_stationary_pose_.theta;
    if (fabs(th_diff) > oscillation_reset_angle_) {
      return true;
    }
  }
  if (oscillation_reset_time_ >= rclcpp::Duration::from_seconds(0.0)) {
    auto t_diff = (clock_->now() - prev_reset_time_);
    if (t_diff > oscillation_reset_time_) {
      return true;
    }
  }
  return false;
}

void OscillationCritic::reset()
{
  x_trend_.reset();
  y_trend_.reset();
  theta_trend_.reset();
}

bool OscillationCritic::setOscillationFlags(const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  bool flag_set = false;
  // set oscillation flags for moving forward and backward
  flag_set |= x_trend_.update(cmd_vel.x);

  // we'll only set flags for strafing and rotating when we're not moving forward at all
  if (x_only_threshold_ < 0.0 || fabs(cmd_vel.x) <= x_only_threshold_) {
    flag_set |= y_trend_.update(cmd_vel.y);
    flag_set |= theta_trend_.update(cmd_vel.theta);
  }
  return flag_set;
}

double OscillationCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (x_trend_.isOscillating(traj.velocity.x) ||
    y_trend_.isOscillating(traj.velocity.y) ||
    theta_trend_.isOscillating(traj.velocity.theta))
  {
    throw dwb_core::
          IllegalTrajectoryException(name_, "Trajectory is oscillating.");
  }
  return 0.0;
}

}  // namespace dwb_critics
