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
#include <vector>

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using nav2::declare_parameter_if_not_declared;
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
  auto node = node_.lock();
  logger_ = node->get_logger();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
  delete kinematics_.load();
}

void KinematicsHandler::initialize(
  const nav2::LifecycleNode::SharedPtr & nh,
  const std::string & plugin_name)
{
  node_ = nh;
  plugin_name_ = plugin_name;

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

  kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
  kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
  kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
  kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;
  auto node = node_.lock();
  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &KinematicsHandler::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &KinematicsHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}

void KinematicsHandler::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  KinematicParameters kinematics(*kinematics_.load());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_;
    kinematics.max_vel_x_ = kinematics.base_max_vel_x_;
    kinematics.max_vel_y_ = kinematics.base_max_vel_y_;
    kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      kinematics.max_speed_xy_ = kinematics.base_max_speed_xy_ * speed_limit / 100.0;
      kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * speed_limit / 100.0;
      kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * speed_limit / 100.0;
      kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      if (speed_limit < kinematics.base_max_speed_xy_) {
        kinematics.max_speed_xy_ = speed_limit;
        // Handling components and angular velocity changes:
        // Max velocities are being changed in the same proportion
        // as absolute linear speed changed in order to preserve
        // robot moving trajectories to be the same after speed change.
        const double ratio = speed_limit / kinematics.base_max_speed_xy_;
        kinematics.max_vel_x_ = kinematics.base_max_vel_x_ * ratio;
        kinematics.max_vel_y_ = kinematics.base_max_vel_y_ * ratio;
        kinematics.max_vel_theta_ = kinematics.base_max_vel_theta_ * ratio;
      }
    }
  }

  // Do not forget to update max_speed_xy_sq_ as well
  kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;

  update_kinematics(kinematics);
}

rcl_interfaces::msg::SetParametersResult KinematicsHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0 &&
        (param_name == plugin_name_ + ".max_vel_x" || param_name == plugin_name_ + ".max_vel_y" ||
        param_name == plugin_name_ + ".max_vel_theta" ||
        param_name == plugin_name_ + ".acc_lim_x" || param_name == plugin_name_ + ".acc_lim_y" ||
        param_name == plugin_name_ + ".acc_lim_theta"))
      {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >= 0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      } else if (parameter.as_double() > 0.0 &&
        (param_name == plugin_name_ + ".acc_lim_x" || param_name == plugin_name_ + ".acc_lim_y" ||
        param_name == plugin_name_ + ".acc_lim_y"))
      {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be <= 0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    }
  }
  return result;
}

void
KinematicsHandler::updateParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  KinematicParameters kinematics(*kinematics_.load());

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".min_vel_x") {
        kinematics.min_vel_x_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".min_vel_y") {
        kinematics.min_vel_y_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".max_vel_x") {
        kinematics.max_vel_x_ = parameter.as_double();
        kinematics.base_max_vel_x_ = kinematics.max_vel_x_;
      } else if (param_name == plugin_name_ + ".max_vel_y") {
        kinematics.max_vel_y_ = parameter.as_double();
        kinematics.base_max_vel_y_ = kinematics.max_vel_y_;
      } else if (param_name == plugin_name_ + ".max_vel_theta") {
        kinematics.max_vel_theta_ = parameter.as_double();
        kinematics.base_max_vel_theta_ = kinematics.max_vel_theta_;
      } else if (param_name == plugin_name_ + ".min_speed_xy") {
        kinematics.min_speed_xy_ = parameter.as_double();
        kinematics.min_speed_xy_sq_ = kinematics.min_speed_xy_ * kinematics.min_speed_xy_;
      } else if (param_name == plugin_name_ + ".max_speed_xy") {
        kinematics.max_speed_xy_ = parameter.as_double();
        kinematics.base_max_speed_xy_ = kinematics.max_speed_xy_;
      } else if (param_name == plugin_name_ + ".min_speed_theta") {
        kinematics.min_speed_theta_ = parameter.as_double();
        kinematics.max_speed_xy_sq_ = kinematics.max_speed_xy_ * kinematics.max_speed_xy_;
      } else if (param_name == plugin_name_ + ".acc_lim_x") {
        kinematics.acc_lim_x_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".acc_lim_y") {
        kinematics.acc_lim_y_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".acc_lim_theta") {
        kinematics.acc_lim_theta_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".decel_lim_x") {
        kinematics.decel_lim_x_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".decel_lim_y") {
        kinematics.decel_lim_y_ = parameter.as_double();
      } else if (param_name == plugin_name_ + ".decel_lim_theta") {
        kinematics.decel_lim_theta_ = parameter.as_double();
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
