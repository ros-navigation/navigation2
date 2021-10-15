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
#include <vector>
#include <string>
#include <utility>

#include "dwb_critics/joy_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

using std::placeholders::_1;
PLUGINLIB_EXPORT_CLASS(dwb_critics::JoyCritic, dwb_core::TrajectoryCritic)
#define PI 3.14159265
namespace dwb_critics
{

void JoyCritic::onInit()
{

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }



  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".linear_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".linear_scale", linear_scale_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".angular_scale", rclcpp::ParameterValue(1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".angular_scale", angular_scale_);


  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".linear_min", rclcpp::ParameterValue(-1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".linear_min", linear_min_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".linear_max", rclcpp::ParameterValue(1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".linear_max", linear_max_);
  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".angular_min", rclcpp::ParameterValue(-1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".angular_min", angular_min_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".angular_max", rclcpp::ParameterValue(1.0));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".angular_max", angular_max_);

  subscription_ = node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyCritic::topic_callback, this, _1));

  nav2_util::declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".decay", rclcpp::ParameterValue(0.9));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".decay", decay_);
  
  x_ = 0.0;
  y_ = 0.0;
  factor = 100.0;
}

double JoyCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if(factor<1)
  {
    x_raw = 0.0;
    y_raw = 0.0;
    factor = 0;
    return 0;
  }
    
  double score = 0.0;
  double li = (fabs(traj.velocity.x - y_));
  double ang = (fabs(traj.velocity.theta - x_));
  
  score = (li * linear_scale_ + ang * angular_scale_) * (factor/100);
  return score;
}


void JoyCritic::debrief(const nav_2d_msgs::msg::Twist2D & twist)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("JoyCritic"),
    "Linear differenct: %f - %f\tAngular difference: %f - %f.", twist.x, y_, twist.theta, x_);
  factor = factor * decay_;
}
void JoyCritic::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  x_raw = x_raw + (1-decay_) * (msg->axes[0] - x_raw);
  y_raw = y_raw + (1-decay_) * (msg->axes[1] - y_raw);
  button = msg->buttons[0];
  if(button == 1)
    factor = 100;
  if(x_raw<0)
    x_ = -(x_raw * angular_min_);
  else
    x_ = (x_raw * angular_max_);
  if(y_raw<0)
    y_ = -(y_raw * linear_min_);
  else
    y_ = y_raw * linear_max_;
  
}


}  // namespace dwb_critics
