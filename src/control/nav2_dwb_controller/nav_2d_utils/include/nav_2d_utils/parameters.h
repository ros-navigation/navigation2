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

#ifndef NAV_2D_UTILS__PARAMETERS_H_
#define NAV_2D_UTILS__PARAMETERS_H_

#include <string>
#include "rclcpp/rclcpp.hpp"

// TODO(crdelsey): Remove when code is re-enabled
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
namespace nav_2d_utils
{

/**
 * @brief Search for a parameter and load it, or use the default value
 *
 * This templated function shortens a commonly used ROS pattern in which you
 * search for a parameter and get its value if it exists, otherwise returning a default value.
 *
 * @param nh NodeHandle to start the parameter search from
 * @param param_name Name of the parameter to search for
 * @param default_value Value to return if not found
 * @return Value of parameter if found, otherwise the default_value
 */
template<class param_t>
param_t searchAndGetParam(
  const std::shared_ptr<rclcpp::Node> & nh, const std::string & param_name,
  const param_t & default_value)
{
  // TODO(crdelsey): Handle params
  // std::string resolved_name;
  // if (nh.searchParam(param_name, resolved_name))
  // {
  //   param_t value = 0;
  //   nh.param(resolved_name, value, default_value);
  //   return value;
  // }
  return default_value;
}

/**
 * @brief Load a parameter from one of two namespaces. Complain if it uses the old name.
 * @param nh NodeHandle to look for the parameter in
 * @param current_name Parameter name that is current, i.e. not deprecated
 * @param old_name Deprecated parameter name
 * @param default_value If neither parameter is present, return this value
 * @return The value of the parameter or the default value
 */
template<class param_t>
param_t loadParameterWithDeprecation(
  const std::shared_ptr<rclcpp::Node> & nh, const std::string current_name,
  const std::string old_name, const param_t & default_value)
{
  // TODO(crdelsey): Handle params
  // param_t value;
  // if (nh.hasParam(current_name))
  // {
  //   nh.getParam(current_name, value);
  //   return value;
  // }
  // if (nh.hasParam(old_name))
  // {
  //   ROS_WARN("Parameter %s is deprecated. Please use the name %s instead.",
  //     old_name.c_str(), current_name.c_str());
  //   nh.getParam(old_name, value);
  //   return value;
  // }
  return default_value;
}

/**
 * @brief If a deprecated parameter exists, complain and move to its new location
 * @param nh NodeHandle to look for the parameter in
 * @param current_name Parameter name that is current, i.e. not deprecated
 * @param old_name Deprecated parameter name
 */
template<class param_t>
void moveDeprecatedParameter(
  const std::shared_ptr<rclcpp::Node> &, const std::string current_name,
  const std::string old_name)
{
  // TODO(crdelsey): Handle params
  // if (!nh.hasParam(old_name)) return;
  //
  // param_t value;
  // ROS_WARN("Parameter %s is deprecated. Please use the name %s instead.",
  //   old_name.c_str(), current_name.c_str());
  // nh.getParam(old_name, value);
  // nh.setParam(current_name, value);
}

/**
 * @brief Move a parameter from one place to another
 *
 * For use loading old parameter structures into new places.
 *
 * If the new name already has a value, don't move the old value there.
 *
 * @param nh NodeHandle for loading/saving params
 * @param old_name Parameter name to move/remove
 * @param current_name Destination parameter name
 * @param default_value Value to save in the new name if old parameter is not found.
 * @param should_delete If true, whether to delete the parameter from the old name
 */
template<class param_t>
void moveParameter(
  const std::shared_ptr<rclcpp::Node> & nh, std::string old_name,
  std::string current_name, param_t default_value, bool should_delete = true)
{
  // TODO(crdelsey): Handle params
  // if (nh.hasParam(current_name))
  // {
  //   if (should_delete)
  //     nh.deleteParam(old_name);
  //   return;
  // }
  // XmlRpc::XmlRpcValue value;
  // if (nh.hasParam(old_name))
  // {
  //   nh.getParam(old_name, value);
  //   if (should_delete) nh.deleteParam(old_name);
  // }
  // else
  //   value = default_value;
  //
  // nh.setParam(current_name, value);
}


}  // namespace nav_2d_utils
#pragma GCC diagnostic pop

#endif  // NAV_2D_UTILS__PARAMETERS_H_
