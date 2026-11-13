// Copyright (c) 2026 Pannaga Sudarshan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

#include "nav2_smoother/smoother_server_parameter_handler.hpp"

namespace nav2_smoother
{


SmootherServerParameterHandler::SmootherServerParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node,
  rclcpp::Logger & logger)
: nav2_util::ParameterHandler<SmootherServerParameters>(node, logger)
{
  params_.costmap_topic = node->declare_or_get_parameter(
    "costmap_topic", std::string("global_costmap/costmap_raw"));
  params_.footprint_topic = node->declare_or_get_parameter(
    "footprint_topic", std::string("global_costmap/published_footprint"));
  params_.robot_base_frame = node->declare_or_get_parameter(
    "robot_base_frame", std::string("base_link"));
  params_.transform_tolerance = node->declare_or_get_parameter("transform_tolerance", 0.1);
  params_.smoother_plugins = node->declare_or_get_parameter(
    "smoother_plugins", std::vector<std::string>{"simple_smoother"});
}

rcl_interfaces::msg::SetParametersResult
SmootherServerParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_name = parameter.get_name();
    // Parameters containing a '.' belong to a plugin and are handled by its own callbacks
    if (param_name.find('.') != std::string::npos) {
      continue;
    }
    const bool is_server_param =
      param_name == "costmap_topic" || param_name == "footprint_topic" ||
      param_name == "robot_base_frame" || param_name == "transform_tolerance" ||
      param_name == "smoother_plugins";
    if (is_server_param) {
      RCLCPP_WARN(
        logger_, "The parameter '%s' cannot be changed at runtime; it requires the "
        "smoother server to be reconfigured. Ignoring parameter update.",
        param_name.c_str());
      result.successful = false;
    }
  }
  return result;
}

void
SmootherServerParameterHandler::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  // Nothing to apply: every SmootherServer parameter is rejected in validation
}

}  // namespace nav2_smoother
