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

#include "dwb_critics/prefer_forward.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::PreferForwardCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void PreferForwardCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  penalty_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".penalty", 1.0);
  strafe_x_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".strafe_x", 0.1);
  strafe_theta_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".strafe_theta", 0.2);
  theta_scale_ = node->declare_or_get_parameter(
    dwb_plugin_name_ + "." + name_ + ".theta_scale", 10.0);
}

double PreferForwardCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // backward motions bad on a robot without backward sensors
  if (traj.velocity.x < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.velocity.x < strafe_x_ && fabs(traj.velocity.theta) < strafe_theta_) {
    return penalty_;
  }

  // the more we rotate, the less we progress forward
  return fabs(traj.velocity.theta) * theta_scale_;
}

}  // namespace dwb_critics
