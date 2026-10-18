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

#ifndef NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_PARAMETER_HANDLER_HPP_
#define NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/logger.hpp"
#include "rclcpp/parameter.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace nav2_smoother
{

/**
 * @struct nav2_smoother::SavitzkyGolaySmootherParameters
 * @brief Parameters for the SavitzkyGolaySmoother plugin
 */
struct SavitzkyGolaySmootherParameters
{
  // Whether to run additional refinement passes after the first smoothing pass
  bool do_refinement;
  // Number of additional refinement passes to run
  int refinement_num;
  // Whether to split the path at direction changes (cusps) and smooth each segment separately
  bool enforce_path_inversion;
  // Size of the Savitzky-Golay filter window; must be odd and at least 3
  int window_size;
  // Order of the polynomial fit within each window
  int poly_order;
};

/**
 * @class nav2_smoother::SavitzkyGolaySmootherParameterHandler
 * @brief Handles parameters and dynamic parameters for the SavitzkyGolaySmoother plugin
 */
class SavitzkyGolaySmootherParameterHandler
  : public nav2_util::ParameterHandler<SavitzkyGolaySmootherParameters>
{
public:
  /**
   * @brief Constructor for nav2_smoother::SavitzkyGolaySmootherParameterHandler
   * @param node Lifecycle node that owns the parameters
   * @param plugin_name Name of the plugin instance, used as the parameter namespace prefix
   * @param logger Logger to use for warnings about rejected updates
   */
  SavitzkyGolaySmootherParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    std::string & plugin_name,
    rclcpp::Logger & logger);

protected:
  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string plugin_name_;
};

}  // namespace nav2_smoother

#endif  // NAV2_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_PARAMETER_HANDLER_HPP_
