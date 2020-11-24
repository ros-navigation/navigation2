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

#include "dwb_plugins/limited_accel_generator.hpp"
#include <vector>
#include <memory>
#include <string>
#include "nav_2d_utils/parameters.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

namespace dwb_plugins
{

void LimitedAccelGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  StandardTrajectoryGenerator::initialize(nh, plugin_name_);

  nav2_util::declare_parameter_if_not_declared(nh, plugin_name + ".sim_period");

  if (nh->get_parameter(plugin_name + ".sim_period", acceleration_time_)) {
  } else {
    double controller_frequency = nav_2d_utils::searchAndGetParam(
      nh, "controller_frequency", 20.0);
    if (controller_frequency > 0) {
      acceleration_time_ = 1.0 / controller_frequency;
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("LimitedAccelGenerator"),
        "A controller_frequency less than or equal to 0 has been set. "
        "Ignoring the parameter, assuming a rate of 20Hz");
      acceleration_time_ = 0.05;
    }
  }
}

void LimitedAccelGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  // Limit our search space to just those within the limited acceleration_time
  velocity_iterator_->startNewIteration(current_velocity, acceleration_time_);
}

nav_2d_msgs::msg::Twist2D LimitedAccelGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D & cmd_vel,
  const nav_2d_msgs::msg::Twist2D & /*start_vel*/,
  const double /*dt*/)
{
  return cmd_vel;
}

}  // namespace dwb_plugins

PLUGINLIB_EXPORT_CLASS(dwb_plugins::LimitedAccelGenerator, dwb_core::TrajectoryGenerator)
