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

#include "dwb_plugins/shared_xy_theta_iterator.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"

#define EPSILON 1E-5

using std::placeholders::_1;

namespace dwb_plugins
{
void SharedXYThetaIterator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  KinematicsHandler::Ptr kinematics,
  const std::string & plugin_name)
{
  kinematics_handler_ = kinematics;

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vx_samples", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vy_samples", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".vtheta_samples", rclcpp::ParameterValue(20));

  nh->get_parameter(plugin_name + ".vx_samples", vx_samples_);
  nh->get_parameter(plugin_name + ".vy_samples", vy_samples_);
  nh->get_parameter(plugin_name + ".vtheta_samples", vtheta_samples_);


  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".linear_min", rclcpp::ParameterValue(0.0));
  nh->get_parameter(plugin_name + ".linear_min", linear_min_);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".linear_max", rclcpp::ParameterValue(1.0));
  nh->get_parameter(plugin_name + ".linear_max", linear_max_);
  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".angular_min", rclcpp::ParameterValue(-1.0));
  nh->get_parameter(plugin_name + ".angular_min", angular_min_);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".angular_max", rclcpp::ParameterValue(1.0));
  nh->get_parameter(plugin_name + ".angular_max", angular_max_);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".z_min", rclcpp::ParameterValue(0.5));
  nh->get_parameter(plugin_name + ".z_min", min_z);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".z_max", rclcpp::ParameterValue(0.5));
  nh->get_parameter(plugin_name + ".z_max", max_z);

  nav2_util::declare_parameter_if_not_declared(
    nh,
    plugin_name + ".z_inc", rclcpp::ParameterValue(0.1));
  nh->get_parameter(plugin_name + ".z_inc", inc_z);
  cur_z=1.0;
  subscription_ = nh->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&SharedXYThetaIterator::topic_callback, this, _1));

}

void SharedXYThetaIterator::startNewIteration(
  const nav_2d_msgs::msg::Twist2D & current_velocity,
  double dt)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  x_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.x,
    kinematics.getMinX(), kinematics.getMaxX(),
    kinematics.getAccX(), kinematics.getDecelX(),
    dt, vx_samples_);
  y_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.y,
    kinematics.getMinY(), kinematics.getMaxY(),
    kinematics.getAccY(), kinematics.getDecelY(),
    dt, vy_samples_);
  th_it_ = std::make_shared<OneDVelocityIterator>(
    current_velocity.theta,
    kinematics.getMinTheta(), kinematics.getMaxTheta(),
    kinematics.getAccTheta(), kinematics.getDecelTheta(),
    dt, vtheta_samples_);
  cur_z=min_z;
  c_joy_b = joy_b;
  c_joy_x = joy_x;
  c_joy_y = joy_y;
  if(c_joy_b == false)
    cur_z = 1.0;
  if (!isValidVelocity()) {
    iterateToValidVelocity();
  }
}

bool SharedXYThetaIterator::isValidSpeed(double x, double y, double theta)
{
  KinematicParameters kinematics = kinematics_handler_->getKinematics();
  double vmag_sq = x * x + y * y;
  if (kinematics.getMaxSpeedXY() >= 0.0 && vmag_sq > kinematics.getMaxSpeedXY_SQ() + EPSILON) {
    return false;
  }
  if (kinematics.getMinSpeedXY() >= 0.0 && vmag_sq + EPSILON < kinematics.getMinSpeedXY_SQ() &&
    kinematics.getMinSpeedTheta() >= 0.0 && fabs(theta) + EPSILON < kinematics.getMinSpeedTheta())
  {
    return false;
  }
  if (vmag_sq == 0.0 && th_it_->getVelocity() == 0.0) {
    return false;
  }
  if(cur_z>max_z)
    return false;
  return true;
}

bool SharedXYThetaIterator::isValidVelocity()
{
  return isValidSpeed(
    x_it_->getVelocity(), y_it_->getVelocity(),
    th_it_->getVelocity());
}

bool SharedXYThetaIterator::hasMoreTwists()
{
  return (x_it_ && !x_it_->isFinished()) || cur_z<max_z;
}

nav_2d_msgs::msg::Twist2D SharedXYThetaIterator::nextTwist()
{
  nav_2d_msgs::msg::Twist2D velocity;
  velocity.x = (cur_z) * x_it_->getVelocity() + (1-cur_z) * c_joy_y;
  velocity.y = y_it_->getVelocity();
  velocity.theta = (cur_z) * th_it_->getVelocity() + (1-cur_z) * c_joy_x;

  iterateToValidVelocity();

  return velocity;
}

void SharedXYThetaIterator::iterateToValidVelocity()
{
  bool valid = false;
  while (!valid && hasMoreTwists()) {
    ++(*th_it_);
    if (th_it_->isFinished()) {
      th_it_->reset();
      ++(*y_it_);
      if (y_it_->isFinished()) {
        y_it_->reset();
        ++(*x_it_);
        if(x_it_->isFinished())
        {
          x_it_->reset();
          cur_z+=inc_z;
        }
      }
    }
    valid = isValidVelocity();
  }
}

void SharedXYThetaIterator::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  
  joy_x = msg->axes[0];
  joy_y = msg->axes[1];
  joy_b = msg->buttons[0];
  if(joy_x<0)
    joy_x = -(joy_x * angular_min_);
  else
    joy_x = (joy_x * angular_max_);
  if(joy_y<0)
    joy_y = -(joy_y * linear_min_);
  else
    joy_y = joy_y * linear_max_;
  
  
}

}  // namespace dwb_plugins
