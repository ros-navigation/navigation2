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

#ifndef DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_
#define DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_

#include <memory>
#include <string>

#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace dwb_plugins
{
/**
 * @class LimitedAccelGenerator
 * @brief Limits the acceleration in the generated trajectories to a fraction of the simulated time.
 */
class LimitedAccelGenerator : public StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2::LifecycleNode::SharedPtr & nh,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;

protected:
  /**
   * @brief Calculate the velocity after a set period of time, given the desired velocity and acceleration limits
   *
   * Unlike the StandardTrajectoryGenerator, the velocity remains constant in the LimitedAccelGenerator
   *
   * @param cmd_vel Desired velocity
   * @param start_vel starting velocity
   * @param dt amount of time in seconds
   * @return cmd_vel
   */
  nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & cmd_vel,
    const nav_2d_msgs::msg::Twist2D & start_vel,
    const double dt) override;
  double acceleration_time_;
  std::string plugin_name_;
};
}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__LIMITED_ACCEL_GENERATOR_HPP_
