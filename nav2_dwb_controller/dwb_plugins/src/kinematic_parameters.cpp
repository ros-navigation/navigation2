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

#include "dwb_plugins/kinematic_parameters.h"
#include <cmath>
#include <memory>
#include <string>
#include "nav_2d_utils/parameters.h"

using std::fabs;

using nav_2d_utils::moveDeprecatedParameter;
namespace dwb_plugins
{

/**
 * @brief Helper function to set the deceleration to the negative acceleration if it was not already set.
 * @param nh NodeHandle
 * @param dimension String representing the dimension, used in constructing parameter names
 */
void setDecelerationAsNeeded(const std::shared_ptr<rclcpp::Node> & nh, const std::string dimension)
{
  std::string decel_param = "decel_lim_" + dimension;
  double temp;
  if (nh->get_parameter(decel_param, temp)) {return;}

  std::string accel_param = "acc_lim_" + dimension;
  double accel;
  if (!nh->get_parameter(accel_param, accel)) {return;}

  nh->set_parameters({rclcpp::Parameter(decel_param, -accel)});
}

KinematicParameters::KinematicParameters()
: min_vel_x_(0.0), min_vel_y_(0.0), max_vel_x_(0.0), max_vel_y_(0.0), max_vel_theta_(0.0),
  min_speed_xy_(0.0), max_speed_xy_(0.0), min_speed_theta_(0.0),
  acc_lim_x_(0.0), acc_lim_y_(0.0), acc_lim_theta_(0.0),
  decel_lim_x_(0.0), decel_lim_y_(0.0), decel_lim_theta_(0.0),
  min_speed_xy_sq_(0.0), max_speed_xy_sq_(0.0)
{
}

void KinematicParameters::initialize(const std::shared_ptr<rclcpp::Node> & nh)
{
  // Special handling for renamed parameters
  moveDeprecatedParameter<double>(nh, "max_vel_theta", "max_rot_vel");
  moveDeprecatedParameter<double>(nh, "min_speed_xy", "min_trans_vel");
  moveDeprecatedParameter<double>(nh, "max_speed_xy", "max_trans_vel");
  moveDeprecatedParameter<double>(nh, "min_speed_theta", "min_rot_vel");

  // Set the deceleration parameters to negative the acceleration if the
  // deceleration not already specified
  setDecelerationAsNeeded(nh, "x");
  setDecelerationAsNeeded(nh, "y");
  setDecelerationAsNeeded(nh, "theta");

  // TODO(crdelsey): Handle dynamic params
  // the rest of the initial values are loaded through the dynamic reconfigure mechanisms
  // dsrv_ = std::make_shared<dynamic_reconfigure::Server<KinematicParamsConfig> >(nh);
  // dynamic_reconfigure::Server<KinematicParamsConfig>::CallbackType cb =
  //   boost::bind(&KinematicParameters::reconfigureCB, this, _1, _2);
  reconfigureCB();
}

void KinematicParameters::reconfigureCB()
{
  // TODO(crdelsey): Remove hard coded parameters when dynamic_reconfigure is in
  min_vel_x_ = -0.26;
  max_vel_x_ = 0.26;
  max_vel_theta_ = 1.0;

  max_speed_xy_ = max_vel_x_;

  acc_lim_x_ = 2.5;
  acc_lim_theta_ = 3.2;
  decel_lim_x_ = -acc_lim_x_;
  decel_lim_theta_ = -acc_lim_theta_;

  // min_vel_x_ = config.min_vel_x;
  // min_vel_y_ = config.min_vel_y;
  // max_vel_x_ = config.max_vel_x;
  // max_vel_y_ = config.max_vel_y;
  // max_vel_theta_ = config.max_vel_theta;
  //
  // min_speed_xy_ = config.min_speed_xy;
  // max_speed_xy_ = config.max_speed_xy;
  min_speed_xy_sq_ = min_speed_xy_ * min_speed_xy_;
  max_speed_xy_sq_ = max_speed_xy_ * max_speed_xy_;
  // min_speed_theta_ = config.min_speed_theta;
  //
  // acc_lim_x_ = config.acc_lim_x;
  // acc_lim_y_ = config.acc_lim_y;
  // acc_lim_theta_ = config.acc_lim_theta;
  // decel_lim_x_ = config.decel_lim_x;
  // decel_lim_y_ = config.decel_lim_y;
  // decel_lim_theta_ = config.decel_lim_theta;
}

bool KinematicParameters::isValidSpeed(double x, double y, double theta)
{
  double vmag_sq = x * x + y * y;
  if (max_speed_xy_ >= 0.0 && vmag_sq > max_speed_xy_sq_) {return false;}
  if (min_speed_xy_ >= 0.0 && vmag_sq < min_speed_xy_sq_ &&
    min_speed_theta_ >= 0.0 && fabs(theta) < min_speed_theta_) {return false;}
  if (vmag_sq == 0.0 && theta == 0.0) {return false;}
  return true;
}

}  // namespace dwb_plugins
