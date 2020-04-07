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
#include "nav2_util/lifecycle_node.hpp"
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
   * @param smooth Smoothing term weight
   * @param costmap Costmap term weight
   * @param distance Distance term weight
   * @param curve max curvature term weight
   * @param max_curve max curvature value, as delta phi / | delta X |
   */
  SmootherParams(
    const double & smooth,
    const double & costmap,
    const double & dist,
    const double & curve,
    const double & max_curve)
  : smooth_weight(smooth),
    costmap_weight(costmap),
    distance_weight(dist),
    curvature_weight(curve),
    max_curvature(max_curve)
  {
  }

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
  void get(nav2_util::LifecycleNode::SharedPtr node_, const std::string & name)
  {
    // Smoother params
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.smoother.w_curve", rclcpp::ParameterValue(1.5));
    node_->get_parameter(name + ".smoother.smoother.w_curve", curvature_weight);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.smoother.w_cost", rclcpp::ParameterValue(0.0));
    node_->get_parameter(name + ".smoother.smoother.w_cost", costmap_weight);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.smoother.w_dist", rclcpp::ParameterValue(15000.0));
    node_->get_parameter(name + ".smoother.smoother.w_dist", distance_weight);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.smoother.w_smooth", rclcpp::ParameterValue(15000.0));
    node_->get_parameter(name + ".smoother.smoother.w_smooth", smooth_weight);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.smoother.max_curve", rclcpp::ParameterValue(7.8));
    node_->get_parameter(name + ".smoother.smoother.max_curve", max_curvature);
  }

  double smooth_weight{0.0};
  double costmap_weight{0.0};
  double distance_weight{0.0};
  double curvature_weight{0.0};
  double max_curvature{0.0};
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
  void get(nav2_util::LifecycleNode::SharedPtr node_, const std::string & name)
  {
    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.param_tol", rclcpp::ParameterValue(1e-15));
    node_->get_parameter(name + ".smoother.optimizer.param_tol", param_tol);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.fn_tol", rclcpp::ParameterValue(1e-7));
    node_->get_parameter(name + ".smoother.optimizer.fn_tol", fn_tol);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.gradient_tol", rclcpp::ParameterValue(1e-10));
    node_->get_parameter(name + ".smoother.optimizer.gradient_tol", gradient_tol);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.max_iterations", rclcpp::ParameterValue(500));
    node_->get_parameter(name + ".smoother.optimizer.max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.max_time", rclcpp::ParameterValue(0.100));
    node_->get_parameter(name + ".smoother.optimizer.max_time", max_time);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.debug_optimizer", rclcpp::ParameterValue(false));
    node_->get_parameter(name + ".smoother.optimizer.debug_optimizer", debug);

    // Optimizer advanced params
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.advanced.min_line_search_step_size",
      rclcpp::ParameterValue(1e-20));
    node_->get_parameter(name + ".smoother.optimizer.advanced.min_line_search_step_size",
      advanced.min_line_search_step_size);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.advanced.max_num_line_search_step_size_iterations",
      rclcpp::ParameterValue(50));
    node_->get_parameter(name + ".smoother.optimizer.advanced.max_num_line_search_step_size_iterations",
      advanced.max_num_line_search_step_size_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.advanced.line_search_sufficient_function_decrease",
      rclcpp::ParameterValue(1e-20));
    node_->get_parameter(name + ".smoother.optimizer.advanced.line_search_sufficient_function_decrease",
      advanced.line_search_sufficient_function_decrease);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.advanced.max_num_line_search_direction_restarts",
      rclcpp::ParameterValue(10));
    node_->get_parameter(name + ".smoother.optimizer.advanced.max_num_line_search_direction_restarts",
      advanced.max_num_line_search_direction_restarts);
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".smoother.optimizer.advanced.max_line_search_step_expansion",
      rclcpp::ParameterValue(50));
    node_->get_parameter(name + ".smoother.optimizer.advanced.max_line_search_step_expansion",
      advanced.max_line_search_step_expansion);
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
