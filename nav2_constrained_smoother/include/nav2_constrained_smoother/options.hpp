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
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
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
  void get(nav2::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".");

    // Smoother params
    double minimum_turning_radius = node->declare_or_get_parameter(
      name + ".minimum_turning_radius", 0.4);
    max_curvature = 1.0f / minimum_turning_radius;
    curvature_weight = node->declare_or_get_parameter(local_name + "w_curve", 30.0);
    costmap_weight = node->declare_or_get_parameter(local_name + "w_cost", 0.015);
    double cost_cusp_multiplier = node->declare_or_get_parameter(
      local_name + "w_cost_cusp_multiplier", 3.0);
    cusp_costmap_weight = costmap_weight * cost_cusp_multiplier;
    cusp_zone_length = node->declare_or_get_parameter(local_name + "cusp_zone_length", 2.5);
    distance_weight = node->declare_or_get_parameter(local_name + "w_dist", 0.0);
    smooth_weight = node->declare_or_get_parameter(local_name + "w_smooth", 2000000.0);
    cost_check_points = node->declare_or_get_parameter(
      local_name + "cost_check_points", std::vector<double>());
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
    path_downsampling_factor = node->declare_or_get_parameter(
      local_name + "path_downsampling_factor", 1);
    path_upsampling_factor = node->declare_or_get_parameter(
      local_name + "path_upsampling_factor", 1);
    reversing_enabled = node->declare_or_get_parameter(local_name + "reversing_enabled", true);
    keep_goal_orientation = node->declare_or_get_parameter(
      local_name + "keep_goal_orientation", true);
    keep_start_orientation = node->declare_or_get_parameter(
      local_name + "keep_start_orientation", true);
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
  void get(nav2::LifecycleNode * node, const std::string & name)
  {
    std::string local_name = name + std::string(".optimizer.");

    // Optimizer params
    linear_solver_type = node->declare_or_get_parameter(
      local_name + "linear_solver_type", std::string("SPARSE_NORMAL_CHOLESKY"));
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
    param_tol = node->declare_or_get_parameter(local_name + "param_tol", 1e-15);
    fn_tol = node->declare_or_get_parameter(local_name + "fn_tol", 1e-7);
    gradient_tol = node->declare_or_get_parameter(local_name + "gradient_tol", 1e-10);
    max_iterations = node->declare_or_get_parameter(local_name + "max_iterations", 100);
    debug = node->declare_or_get_parameter(local_name + "debug_optimizer", false);
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
