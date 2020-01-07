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

#include "dwb_plugins/kinematic_parameters.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"

using std::fabs;
using nav2_util::declare_parameter_if_not_declared;
using nav_2d_utils::moveDeprecatedParameter;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace dwb_plugins
{

KinematicParameters::KinematicParameters()
{
}

void KinematicParameters::initialize(const nav2_util::LifecycleNode::SharedPtr & nh)
{
  // Special handling for renamed parameters
  moveDeprecatedParameter<double>(nh, "max_vel_theta", "max_rot_vel");
  moveDeprecatedParameter<double>(nh, "min_speed_xy", "min_trans_vel");
  moveDeprecatedParameter<double>(nh, "max_speed_xy", "max_trans_vel");
  moveDeprecatedParameter<double>(nh, "min_speed_theta", "min_rot_vel");

  declare_parameter_if_not_declared(nh, "min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "min_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "max_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "max_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "max_vel_theta", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "min_speed_xy", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "max_speed_xy", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "min_speed_theta", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "acc_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "acc_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "acc_lim_theta", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "decel_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "decel_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, "decel_lim_theta", rclcpp::ParameterValue(0.0));

  nh->get_parameter("min_vel_x", min_vel_x_);
  nh->get_parameter("min_vel_y", min_vel_y_);
  nh->get_parameter("max_vel_x", max_vel_x_);
  nh->get_parameter("max_vel_y", max_vel_y_);
  nh->get_parameter("max_vel_theta", max_vel_theta_);
  nh->get_parameter("min_speed_xy", min_speed_xy_);
  nh->get_parameter("max_speed_xy", max_speed_xy_);
  nh->get_parameter("min_speed_theta", min_speed_theta_);
  nh->get_parameter("acc_lim_x", acc_lim_x_);
  nh->get_parameter("acc_lim_y", acc_lim_y_);
  nh->get_parameter("acc_lim_theta", acc_lim_theta_);
  nh->get_parameter("decel_lim_x", decel_lim_x_);
  nh->get_parameter("decel_lim_y", decel_lim_y_);
  nh->get_parameter("decel_lim_theta", decel_lim_theta_);

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    nh->get_node_base_interface(),
    nh->get_node_topics_interface(),
    nh->get_node_graph_interface(),
    nh->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&KinematicParameters::on_parameter_event_callback, this, _1));

  min_speed_xy_sq_ = min_speed_xy_ * min_speed_xy_;
  max_speed_xy_sq_ = max_speed_xy_ * max_speed_xy_;
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

void
KinematicParameters::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "min_vel_x") {
        min_vel_x_ = value.double_value;
      } else if (name == "min_vel_y") {
        min_vel_y_ = value.double_value;
      } else if (name == "max_vel_x") {
        max_vel_x_ = value.double_value;
      } else if (name == "max_vel_y") {
        max_vel_y_ = value.double_value;
      } else if (name == "max_vel_theta") {
        max_vel_theta_ = value.double_value;
      } else if (name == "min_speed_xy") {
        min_speed_xy_ = value.double_value;
      } else if (name == "max_speed_xy") {
        max_speed_xy_ = value.double_value;
      } else if (name == "min_speed_theta") {
        min_speed_theta_ = value.double_value;
      } else if (name == "acc_lim_x") {
        acc_lim_x_ = value.double_value;
      } else if (name == "acc_lim_y") {
        acc_lim_y_ = value.double_value;
      } else if (name == "acc_lim_theta") {
        acc_lim_theta_ = value.double_value;
      } else if (name == "decel_lim_x") {
        decel_lim_x_ = value.double_value;
      } else if (name == "decel_lim_y") {
        decel_lim_y_ = value.double_value;
      } else if (name == "decel_lim_theta") {
        decel_lim_theta_ = value.double_value;
      }
    }
  }
}

}  // namespace dwb_plugins
