// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__NAV2_OPTIONS_HPP_
#define SMAC_PLANNER__NAV2_OPTIONS_HPP_

#include <string>

#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "smac_planner/options.hpp"

namespace smac_planner
{

SmootherParams get_smoother(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
{
  std::string local_name = name + std::string(".smoother.smoother.");

  SmootherParams smoother_params{};
  // Smoother params
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "w_curve", rclcpp::ParameterValue(1.5));
  node->get_parameter(local_name + "w_curve", smoother_params.curvature_weight);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "w_cost", rclcpp::ParameterValue(0.0));
  node->get_parameter(local_name + "w_cost", smoother_params.costmap_weight);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
  node->get_parameter(local_name + "w_dist", smoother_params.distance_weight);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "w_smooth", rclcpp::ParameterValue(15000.0));
  node->get_parameter(local_name + "w_smooth", smoother_params.smooth_weight);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "cost_scaling_factor", rclcpp::ParameterValue(10.0));
  node->get_parameter(local_name + "cost_scaling_factor", smoother_params.costmap_factor);
  return smoother_params;
}

OptimizerParams::AdvancedParams get_advanced(
  rclcpp_lifecycle::LifecycleNode * node,
  const std::string & name)
{
  std::string local_name = name + std::string(".smoother.optimizer.advanced.");

  OptimizerParams::AdvancedParams advanced{};
  // Optimizer advanced params
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "min_line_search_step_size",
    rclcpp::ParameterValue(1e-20));
  node->get_parameter(
    local_name + "min_line_search_step_size",
    advanced.min_line_search_step_size);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "max_num_line_search_step_size_iterations",
    rclcpp::ParameterValue(50));
  node->get_parameter(
    local_name + "max_num_line_search_step_size_iterations",
    advanced.max_num_line_search_step_size_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "line_search_sufficient_function_decrease",
    rclcpp::ParameterValue(1e-20));
  node->get_parameter(
    local_name + "line_search_sufficient_function_decrease",
    advanced.line_search_sufficient_function_decrease);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "max_num_line_search_direction_restarts",
    rclcpp::ParameterValue(10));
  node->get_parameter(
    local_name + "max_num_line_search_direction_restarts",
    advanced.max_num_line_search_direction_restarts);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "max_line_search_step_expansion",
    rclcpp::ParameterValue(50));
  node->get_parameter(
    local_name + "max_line_search_step_expansion",
    advanced.max_line_search_step_expansion);
  return advanced;
}

OptimizerParams get_optimizer(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
{
  std::string local_name = name + std::string(".smoother.optimizer.");

  OptimizerParams optimizer_params{};
  // Optimizer params
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
  node->get_parameter(local_name + "param_tol", optimizer_params.param_tol);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
  node->get_parameter(local_name + "fn_tol", optimizer_params.fn_tol);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
  node->get_parameter(local_name + "gradient_tol", optimizer_params.gradient_tol);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "max_iterations", rclcpp::ParameterValue(500));
  node->get_parameter(local_name + "max_iterations", optimizer_params.max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "max_time", rclcpp::ParameterValue(0.100));
  node->get_parameter(local_name + "max_time", optimizer_params.max_time);
  nav2_util::declare_parameter_if_not_declared(
    node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
  node->get_parameter(local_name + "debug_optimizer", optimizer_params.debug);

  optimizer_params.advanced = get_advanced(node, name);
  return optimizer_params;
}


}  // namespace smac_planner

#endif  // SMAC_PLANNER__NAV2_OPTIONS_HPP_
