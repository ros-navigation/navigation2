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
  SmootherParams() {}

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

    double min_line_search_step_size;                 // Ceres default: 1e-9
    int max_num_line_search_step_size_iterations;     // Ceres default: 20
    double line_search_sufficient_function_decrease;  // Ceres default: 1e-4
    int max_num_line_search_direction_restarts;       // Ceres default: 5

    double max_line_search_step_contraction;           // Ceres default: 1e-3
    double min_line_search_step_contraction;           // Ceres default: 0.6
    double line_search_sufficient_curvature_decrease;  // Ceres default: 0.9
    int max_line_search_step_expansion;                // Ceres default: 10
  };

  bool debug;
  int max_iterations;  // Ceres default: 50
  double max_time;     // Ceres default: 1e4

  double param_tol;     // Ceres default: 1e-8
  double fn_tol;        // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10

  AdvancedParams advanced;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__OPTIONS_HPP_
