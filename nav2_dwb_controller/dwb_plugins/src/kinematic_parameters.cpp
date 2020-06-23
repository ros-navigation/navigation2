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

#include <memory>
#include <string>

#include "nav_2d_utils/parameters.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav_2d_utils::moveDeprecatedParameter;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace dwb_plugins
{

KinematicsHandler::KinematicsHandler()
{
  kinematics_.store(new KinematicParameters);
}

KinematicsHandler::~KinematicsHandler()
{
  delete kinematics_.load();
}

void KinematicsHandler::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  // Special handling for renamed parameters
  moveDeprecatedParameter<double>(nh, plugin_name + ".max_vel_theta", "max_rot_vel");
  moveDeprecatedParameter<double>(nh, plugin_name + ".min_speed_xy", "min_trans_vel");
  moveDeprecatedParameter<double>(nh, plugin_name + ".max_speed_xy", "max_trans_vel");
  moveDeprecatedParameter<double>(nh, plugin_name + ".min_speed_theta", "min_rot_vel");

  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".min_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".max_vel_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_vel_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".max_speed_xy",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".min_speed_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".acc_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".acc_lim_theta",
    rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(nh, plugin_name + ".decel_lim_y", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    nh, plugin_name + ".decel_lim_theta",
    rclcpp::ParameterValue(0.0));

  KinematicParameters kinematics;

  nh->get_parameter(plugin_name + ".min_vel_x", kinematics.min_vel_x_);
  nh->get_parameter(plugin_name + ".min_vel_y", kinematics.min_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_x", kinematics.max_vel_x_);
  nh->get_parameter(plugin_name + ".max_vel_y", kinematics.max_vel_y_);
  nh->get_parameter(plugin_name + ".max_vel_theta", kinematics.max_vel_theta_);
  nh->get_parameter(plugin_name + ".min_speed_xy", kinematics.min_speed_xy_);
  nh->get_parameter(plugin_name + ".max_speed_xy", kinematics.max_speed_xy_);
  nh->get_parameter(plugin_name + ".min_speed_theta", kinematics.min_speed_theta_);
  nh->get_parameter(plugin_name + ".acc_lim_x", kinematics.acc_lim_x_);
  nh->get_parameter(plugin_name + ".acc_lim_y", kinematics.acc_lim_y_);
  nh->get_parameter(plugin_name + ".acc_lim_theta", kinematics.acc_lim_theta_);
  nh->get_parameter(plugin_name + ".decel_lim_x", kinematics.decel_lim_x_);
  nh->get_parameter(plugin_name + ".decel_lim_y", kinematics.decel_lim_y_);
  nh->get_parameter(plugin_name + ".decel_lim_theta", kinematics.decel_lim_theta_);

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    nh->get_node_base_interface(),
    nh->get_node_topics_interface(),
    nh->get_node_graph_interface(),
    nh->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&KinematicsHandler::on_parameter_event_callback, this, _1));

  kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}


void
KinematicsHandler::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  KinematicParameters kinematics(*kinematics_.load());

  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".min_vel_x") {
        kinematics.min_vel_x_ = value.double_value;
      } else if (name == plugin_name_ + ".min_vel_y") {
        kinematics.min_vel_y_ = value.double_value;
      } else if (name == plugin_name_ + ".max_vel_x") {
        kinematics.max_vel_x_ = value.double_value;
      } else if (name == plugin_name_ + ".max_vel_y") {
        kinematics.max_vel_y_ = value.double_value;
      } else if (name == plugin_name_ + ".max_vel_theta") {
        kinematics.max_vel_theta_ = value.double_value;
      } else if (name == plugin_name_ + ".min_speed_xy") {
        kinematics.min_speed_xy_ = value.double_value;
        kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
      } else if (name == plugin_name_ + ".max_speed_xy") {
        kinematics.max_speed_xy_ = value.double_value;
      } else if (name == plugin_name_ + ".min_speed_theta") {
        kinematics.min_speed_theta_ = value.double_value;
        kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;
      } else if (name == plugin_name_ + ".acc_lim_x") {
        kinematics.acc_lim_x_ = value.double_value;
      } else if (name == plugin_name_ + ".acc_lim_y") {
        kinematics.acc_lim_y_ = value.double_value;
      } else if (name == plugin_name_ + ".acc_lim_theta") {
        kinematics.acc_lim_theta_ = value.double_value;
      } else if (name == plugin_name_ + ".decel_lim_x") {
        kinematics.decel_lim_x_ = value.double_value;
      } else if (name == plugin_name_ + ".decel_lim_y") {
        kinematics.decel_lim_y_ = value.double_value;
      } else if (name == plugin_name_ + ".decel_lim_theta") {
        kinematics.decel_lim_theta_ = value.double_value;
      }
    }
  }
  update_kinematics(kinematics);
}

void KinematicsHandler::update_kinematics(KinematicParameters kinematics)
{
  delete kinematics_.load();
  kinematics_.store(new KinematicParameters(kinematics));
}

}  // namespace dwb_plugins
