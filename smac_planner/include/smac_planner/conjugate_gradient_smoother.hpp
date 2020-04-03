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

#ifndef SMAC_PLANNER__NON_LINEAR_CONJUGATE_GRADIENT_SMOOTHER_HPP_
#define SMAC_PLANNER__NON_LINEAR_CONJUGATE_GRADIENT_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "smac_planner/types.hpp"
#include "smac_planner/smoother_cost_function.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

// TODO separate server instance for smoothing if desireable
// TODO separate tuning server instance for refining for your needs

namespace smac_planner
{

/**
 * @class smac_planner::CGSmoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 */
class CGSmoother
{
public:
  /**
   * @brief A constructor for smac_planner::CGSmoother
   */
  CGSmoother() {}

  /**
   * @brief A destructor for smac_planner::CGSmoother
   */
  ~CGSmoother() {}

  /**
   * @brief Initialization of the smoother
   */
  void initialize(const OptimizerParams params) {
    debug_ = params.debug;

    // General Params
    options_.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    options_.line_search_type = ceres::WOLFE;
    options_.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    options_.line_search_interpolation_type = ceres::CUBIC;

    options_.max_num_iterations = params.max_iterations;
    options_.max_solver_time_in_seconds = params.max_time;

    options_.function_tolerance = params.fn_tol;
    options_.gradient_tolerance = params.gradient_tol;
    options_.parameter_tolerance = params.param_tol;

    options_.min_line_search_step_size = params.advanced.min_line_search_step_size;
    options_.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    options_.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;
    options_.max_line_search_step_contraction = params.advanced.max_line_search_step_contraction;
    options_.min_line_search_step_contraction = params.advanced.min_line_search_step_contraction;
    options_.max_num_line_search_direction_restarts =
      params.advanced.max_num_line_search_direction_restarts;
    options_.line_search_sufficient_curvature_decrease =
      params.advanced.line_search_sufficient_curvature_decrease;
    options_.max_line_search_step_expansion = params.advanced.max_line_search_step_expansion;

    if (debug_) {
      options_.minimizer_progress_to_stdout = true;
    } else {
      options_.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Initialization of the smoother
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param smoother parameters weights
   * @return If smoothing was successful
   */
  bool smooth(
    std::vector<Eigen::Vector2d> & path, MinimalCostmap * costmap,
    const SmootherParams & params)
  {
    double parameters[path.size() * 2];
    for (uint i = 0; i != path.size(); i++) {
      parameters[2 * i] = path[i][0];
      parameters[2 * i + 1] = path[i][1];
    }

    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UnconstrainedSmootherCostFunction(& path, costmap, params));
    ceres::Solve(options_, problem, parameters, &summary);

    if (debug_) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable()) {
      return false;
    }

    for (uint i = 0; i != path.size(); i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }

    return true;
  }

private:
  bool debug_;
  ceres::GradientProblemSolver::Options options_;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NON_LINEAR_CONJUGATE_GRADIENT_SMOOTHER_HPP_
