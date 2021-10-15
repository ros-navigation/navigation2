/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Ubicomp
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

#include "dwb_critics/keep_velocity.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace dwb_critics
{
  void KeepVelocityCritic::onInit()
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    node->declare_parameter(
          dwb_plugin_name_ + "." + name_ + ".decay_scale",
          rclcpp::ParameterValue(0.999));
    node->declare_parameter(
          dwb_plugin_name_ + "." + name_ + ".theta_tolerance",
          rclcpp::ParameterValue(0.1));
    node->declare_parameter(
          dwb_plugin_name_ + "." + name_ + ".straight_to_turn_scale",
          rclcpp::ParameterValue(1.0));
    node->declare_parameter(
          dwb_plugin_name_ + "." + name_ + ".turn_to_straight_scale",
          rclcpp::ParameterValue(1.0));
    // Scale is set to 0 by default, so if it was not set otherwise, set to 0
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".decay_scale", decay_scale_);
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".straight_to_turn_scale", stt_scale_);
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".turn_to_straight_scale", tts_scale_);
    node->get_parameter(dwb_plugin_name_ + "." + name_ + ".theta_tolercance", theta_tolerance_);
    reset();
  }

  double KeepVelocityCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
  {
    if(going_forward_ && fabs(traj.velocity.theta)>theta_tolerance_)
    {
      return current_scale_ * scale_ * stt_scale_;
    }
    if(going_forward_ == false && fabs(traj.velocity.theta)<theta_tolerance_)
    {
      return current_scale_ * scale_ * tts_scale_;
    }
    return 0;
  }

  void KeepVelocityCritic::reset()
  {
    going_forward_ = true;
    current_scale_ = decay_scale_;
  }

  void KeepVelocityCritic::debrief(const nav_2d_msgs::msg::Twist2D & cmd_vel)
  {
    bool will_go_forward = false;
    if(fabs(cmd_vel.theta)<theta_tolerance_)
      will_go_forward = true;
    if((will_go_forward == true && going_forward_ == true) || (will_go_forward == false && going_forward_ == false))
    {
      current_scale_ *= decay_scale_;
    }
    else
    {
      current_scale_ = decay_scale_;
      going_forward_ = will_go_forward;
    }
  }
}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::KeepVelocityCritic, dwb_core::TrajectoryCritic)
