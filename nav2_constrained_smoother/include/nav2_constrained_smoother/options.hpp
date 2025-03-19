// Copyright (c) 2021 RoboTech Vision
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

#ifndef NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_

#include <map>
#include <string>
#include <vector>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "ceres/ceres.h"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_smac_planner::SmootherParams
 * @brief Parameters for the smoother cost function
 */
struct SmootherParams
{
  /**
   * @brief A constructor for nav2_smac_planner::SmootherParams
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
    std::string local_name = name + std::string(".");

    // Smoother params
    double minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);
    max_curvature = 1.0f / minimum_turning_radius;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_curve", rclcpp::ParameterValue(30.0));
    node->get_parameter(local_name + "w_curve", curvature_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost", rclcpp::ParameterValue(0.015));
    node->get_parameter(local_name + "w_cost", costmap_weight);
    double cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_cost_cusp_multiplier", rclcpp::ParameterValue(3.0));
    node->get_parameter(local_name + "w_cost_cusp_multiplier", cost_cusp_multiplier);
    cusp_costmap_weight = costmap_weight * cost_cusp_multiplier;
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cusp_zone_length", rclcpp::ParameterValue(2.5));
    node->get_parameter(local_name + "cusp_zone_length", cusp_zone_length);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_dist", rclcpp::ParameterValue(0.0));
    node->get_parameter(local_name + "w_dist", distance_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "w_smooth", rclcpp::ParameterValue(2000000.0));
    node->get_parameter(local_name + "w_smooth", smooth_weight);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "cost_check_points", rclcpp::ParameterValue(std::vector<double>()));
    node->get_parameter(local_name + "cost_check_points", cost_check_points);
    if (cost_check_points.size() % 3 != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "constrained_smoother"),
        "cost_check_points parameter must contain values as follows: "
        "[x1, y1, weight1, x2, y2, weight2, ...]");
      throw std::runtime_error("Invalid parameter: cost_check_points");
    }
    // normalize check point weights so that their sum == 1.0
    double check_point_weights_sum = 0.0;
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      check_point_weights_sum += cost_check_points[i];
    }
    for (size_t i = 2u; i < cost_check_points.size(); i += 3) {
      cost_check_points[i] /= check_point_weights_sum;
    }
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "path_downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_downsampling_factor", path_downsampling_factor);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "path_upsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(local_name + "path_upsampling_factor", path_upsampling_factor);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "reversing_enabled", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "reversing_enabled", reversing_enabled);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "keep_goal_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_goal_orientation", keep_goal_orientation);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "keep_start_orientation", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "keep_start_orientation", keep_start_orientation);
  }

  double smooth_weight{0.0};
  double costmap_weight{0.0};
  double cusp_costmap_weight{0.0};
  double cusp_zone_length{0.0};
  double distance_weight{0.0};
  double curvature_weight{0.0};
  double max_curvature{0.0};
  double max_time{10.0};  // adjusted by action goal, not by parameters
  int path_downsampling_factor{1};
  int path_upsampling_factor{1};
  bool reversing_enabled{true};
  bool keep_goal_orientation{true};
  bool keep_start_orientation{true};
  std::vector<double> cost_check_points{};
};

/**
 * @struct nav2_smac_planner::OptimizerParams
 * @brief Parameters for the ceres optimizer
 */
struct OptimizerParams
{
  OptimizerParams()
  : debug(false),
    max_iterations(50),
    param_tol(1e-8),
    fn_tol(1e-6),
    gradient_tol(1e-10)
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node_ Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "linear_solver_type", rclcpp::ParameterValue("SPARSE_NORMAL_CHOLESKY"));
    node->get_parameter(local_name + "linear_solver_type", linear_solver_type);
    if (solver_types.find(linear_solver_type) == solver_types.end()) {
      std::stringstream valid_types_str;
      for (auto type = solver_types.begin(); type != solver_types.end(); type++) {
        if (type != solver_types.begin()) {
          valid_types_str << ", ";
        }
        valid_types_str << type->first;
      }
      RCLCPP_ERROR(
        rclcpp::get_logger("constrained_smoother"),
        "Invalid linear_solver_type. Valid values are %s", valid_types_str.str().c_str());
      throw std::runtime_error("Invalid parameter: linear_solver_type");
    }
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
      node, local_name + "max_iterations", rclcpp::ParameterValue(100));
    node->get_parameter(local_name + "max_iterations", max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node, local_name + "debug_optimizer", rclcpp::ParameterValue(false));
    node->get_parameter(local_name + "debug_optimizer", debug);
  }

  const std::map<std::string, ceres::LinearSolverType> solver_types = {
    {"DENSE_QR", ceres::DENSE_QR},
    {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY}};

  bool debug;
  std::string linear_solver_type;
  int max_iterations;  // Ceres default: 50

  double param_tol;  // Ceres default: 1e-8
  double fn_tol;  // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__OPTIONS_HPP_
