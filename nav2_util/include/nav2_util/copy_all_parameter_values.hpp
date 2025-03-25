// Copyright 2023 Open Navigation LLC
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

#ifndef NAV2_UTIL__COPY_ALL_PARAMETER_VALUES_HPP_
#define NAV2_UTIL__COPY_ALL_PARAMETER_VALUES_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/parameter.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

/**
 * Copy all parameters from one source node to another destination node.
 * May throw exceptions if parameters from source are uninitialized or undeclared.
 * \param source Node to copy parameters from
 * \param destination Node to copy parameters to
 * \param override_existing_params Default false. Whether to override existing destination params
 * if both the source and destination contain the same parameter.
 */
template<typename NodeT1, typename NodeT2>
void
copy_all_parameter_values(
  const NodeT1 & source, const NodeT2 & destination, const bool override_existing_params = false)
{
  using Parameters = std::vector<rclcpp::Parameter>;
  using Descriptions = std::vector<rcl_interfaces::msg::ParameterDescriptor>;
  auto source_params = source->get_node_parameters_interface();
  auto dest_params = destination->get_node_parameters_interface();
  rclcpp::Logger logger = destination->get_node_logging_interface()->get_logger();

  std::vector<std::string> param_names = source_params->list_parameters({}, 0).names;
  Parameters params = source_params->get_parameters(param_names);
  Descriptions descriptions = source_params->describe_parameters(param_names);

  for (unsigned int idx = 0; idx != params.size(); idx++) {
    if (!dest_params->has_parameter(params[idx].get_name())) {
      dest_params->declare_parameter(
        params[idx].get_name(), params[idx].get_parameter_value(), descriptions[idx]);
    } else if (override_existing_params) {
      try {
        rcl_interfaces::msg::SetParametersResult result =
          dest_params->set_parameters_atomically({params[idx]});
        if (!result.successful) {
          // Parameter update rejected or read-only
          RCLCPP_WARN(
            logger,
            "Unable to set parameter (%s): %s!",
            params[idx].get_name().c_str(), result.reason.c_str());
        }
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_WARN(
          logger,
          "Unable to set parameter (%s): incompatable parameter type (%s)!",
          params[idx].get_name().c_str(), e.what());
      }
    }
  }
}

}  // namespace rclcpp

#endif  // NAV2_UTIL__COPY_ALL_PARAMETER_VALUES_HPP_
