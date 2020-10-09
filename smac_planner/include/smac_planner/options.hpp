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

#ifndef SMAC_PLANNER__OPTIONS_HPP_
#define SMAC_PLANNER__OPTIONS_HPP_

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace smac_planner
{

/**
 * @struct smac_planner::SmootherParams
 * @brief Parameters for the smoother cost function
 */
struct SmootherParams
{
  /**
   * @brief A constructor for smac_planner::SmootherParams
   */
  SmootherParams()
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.smoother.");

    // Smoother params
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_curve", rclcpp::ParameterValue(1.5));
    node->get_parameter(local_name + "w_curve", curvature_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_cost", costmap_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_dist", distance_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(15000.0));
    node->get_parameter(local_name + "w_smooth", smooth_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cost_scaling_factor", rclcpp::ParameterValue(10.0));
    node->get_parameter(local_name + "cost_scaling_factor", costmap_factor);
  }

  double smooth_weight{0.0};
  double costmap_weight{0.0};
  double distance_weight{0.0};
  double curvature_weight{0.0};
  double max_curvature{0.0};
  double costmap_factor{0.0};
  double max_time;
};

/**
 * @struct smac_planner::OptimizerParams
 * @brief Parameters for the ceres optimizer
 */
struct OptimizerParams
{
  OptimizerParams()
  : debug(false),
    max_iterations(50),
    max_time(1e4),
    param_tol(1e-8),
    fn_tol(1e-6),
    gradient_tol(1e-10)
  {
  }

  /**
   * @struct AdvancedParams
   * @brief Advanced parameters for the ceres optimizer
   */
  struct AdvancedParams
  {
    AdvancedParams()
    : min_line_search_step_size(1e-9),
      max_num_line_search_step_size_iterations(20),
      line_search_sufficient_function_decrease(1e-4),
      max_num_line_search_direction_restarts(20),
      max_line_search_step_contraction(1e-3),
      min_line_search_step_contraction(0.6),
      line_search_sufficient_curvature_decrease(0.9),
      max_line_search_step_expansion(10)
    {
    }

    /**
     * @brief Get advanced params from ROS parameter
     * @param node_ Ptr to node
     * @param name Name of plugin
     */
    void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
    {
      std::string local_name = name + std::string(".smoother.optimizer.advanced.");

      // Optimizer advanced params
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "min_line_search_step_size",
        rclcpp::ParameterValue(1e-20));
      node->get_parameter(
        local_name + "min_line_search_step_size",
        min_line_search_step_size);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_num_line_search_step_size_iterations",
        rclcpp::ParameterValue(50));
      node->get_parameter(
        local_name + "max_num_line_search_step_size_iterations",
        max_num_line_search_step_size_iterations);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "line_search_sufficient_function_decrease",
        rclcpp::ParameterValue(1e-20));
      node->get_parameter(
        local_name + "line_search_sufficient_function_decrease",
        line_search_sufficient_function_decrease);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_num_line_search_direction_restarts",
        rclcpp::ParameterValue(10));
      node->get_parameter(
        local_name + "max_num_line_search_direction_restarts",
        max_num_line_search_direction_restarts);
      nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_line_search_step_expansion",
        rclcpp::ParameterValue(50));
      node->get_parameter(
        local_name + "max_line_search_step_expansion",
        max_line_search_step_expansion);
    }


    double min_line_search_step_size;  // Ceres default: 1e-9
    int max_num_line_search_step_size_iterations;  // Ceres default: 20
    double line_search_sufficient_function_decrease;  // Ceres default: 1e-4
    int max_num_line_search_direction_restarts;  // Ceres default: 5

    double max_line_search_step_contraction;  // Ceres default: 1e-3
    double min_line_search_step_contraction;  // Ceres default: 0.6
    double line_search_sufficient_curvature_decrease;  // Ceres default: 0.9
    int max_line_search_step_expansion;  // Ceres default: 10
  };

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".smoother.optimizer.");

    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "param_tol", rclcpp::ParameterValue(1e-15));
    node->get_parameter(local_name + "param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "fn_tol", rclcpp::ParameterValue(1e-7));
    node->get_parameter(local_name + "fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "gradient_tol", rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_iterations", rclcpp::ParameterValue(500));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "max_time", rclcpp::ParameterValue(0.100));
    node->get_parameter(local_name + "max_time", max_time);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);

    advanced.get(node, name);
  }

  bool debug;
  int max_iterations;  // Ceres default: 50
  double max_time;  // Ceres default: 1e4

  double param_tol;  // Ceres default: 1e-8
  double fn_tol;  // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10

  AdvancedParams advanced;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__OPTIONS_HPP_
