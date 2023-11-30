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

#ifndef DWB_PLUGINS__XY_THETA_ITERATOR_HPP_
#define DWB_PLUGINS__XY_THETA_ITERATOR_HPP_

#include <memory>
#include <string>

#include "dwb_plugins/velocity_iterator.hpp"
#include "dwb_plugins/one_d_velocity_iterator.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace dwb_plugins
{
class XYThetaIterator : public VelocityIterator
{
public:
  XYThetaIterator()
  : kinematics_handler_(nullptr), x_it_(nullptr), y_it_(nullptr), th_it_(nullptr) {}
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    KinematicsHandler::Ptr kinematics,
    const std::string & plugin_name) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity, double dt) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;

protected:
  virtual bool isValidVelocity();
  void iterateToValidVelocity();
  int vx_samples_, vy_samples_, vtheta_samples_;
  KinematicsHandler::Ptr kinematics_handler_;

  std::shared_ptr<OneDVelocityIterator> x_it_, y_it_, th_it_;
};
}  // namespace dwb_plugins

#endif  // DWB_PLUGINS__XY_THETA_ITERATOR_HPP_
