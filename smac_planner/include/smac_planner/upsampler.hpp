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

#ifndef SMAC_PLANNER__UPSAMPLER_HPP_
#define SMAC_PLANNER__UPSAMPLER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <algorithm>
#include <utility>

#include "smac_planner/types.hpp"
#include "smac_planner/upsampler_cost_function.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace smac_planner
{

// TODO reduce code duplication. there's very litle change here., maybe put smoother and upsampler together in an object?

/**
 * @class smac_planner::Upsampler
 * @brief A Conjugate Gradient 2D path upsampler implementation
 */
class Upsampler
{
public:
  /**
   * @brief A constructor for smac_planner::Smoother
   */
  Upsampler() {}

  /**
   * @brief A destructor for smac_planner::Smoother
   */
  ~Upsampler() {}

  /**
   * @brief Initialization of the smoother
   */
  void initialize(const OptimizerParams params) {
    _debug = params.debug;

    // General Params

    // 2 most valid options: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT
    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.max_num_iterations = params.max_iterations;
    _options.max_solver_time_in_seconds = params.max_time;

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;

    _options.min_line_search_step_size = params.advanced.min_line_search_step_size;
    _options.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    _options.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;
    _options.max_line_search_step_contraction = params.advanced.max_line_search_step_contraction;
    _options.min_line_search_step_contraction = params.advanced.min_line_search_step_contraction;
    _options.max_num_line_search_direction_restarts =
      params.advanced.max_num_line_search_direction_restarts;
    _options.line_search_sufficient_curvature_decrease =
      params.advanced.line_search_sufficient_curvature_decrease;
    _options.max_line_search_step_expansion = params.advanced.max_line_search_step_expansion;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Upsampling method
   * @param path Reference to path
   * @param upsample parameters weights
   * @param upsample_ratio upsample ratio
   * @return If smoothing was successful
   */
  bool upsample(
    std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params,
    const int & upsample_ratio)
  {
    const int param_ratio = upsample_ratio * 2.0;
    const int total_size = 2 * (path.size() * upsample_ratio - upsample_ratio + 1);
    double parameters[total_size];

    // Linearly upsample initial poses for optimization
    int pt;
    int next_pt;
    double dist_ratio;
    double dist_x;
    double dist_y;
    double upsample_ratio_double = static_cast<double>(upsample_ratio);
    for (uint i = 0; i != total_size / 2; i++) {
      pt = i / upsample_ratio;
      next_pt = pt + 1;

      if (i % upsample_ratio == 0) {
        parameters[param_ratio * pt] = path[pt][0];
        parameters[param_ratio * pt + 1] = path[pt][1];
        dist_x = path[next_pt][0] - path[pt][0];
        dist_y = path[next_pt][1] - path[pt][1];
      } else {
        dist_ratio = static_cast<double>(i % upsample_ratio) / upsample_ratio_double;
        parameters[2 * i] = dist_ratio * dist_x + path[pt][0];
        parameters[2 * i + 1] = dist_ratio * dist_y + path[pt][1];
      }
    }

    // Solve the upsampling problem
    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UpsamplerCostFunction(path.size(), params, upsample_ratio));
    ceres::Solve(_options, problem, parameters, &summary);

    if (_debug) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }

    path.resize(total_size / 2);
    for (uint i = 0; i != path.size(); i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }

    return true;
  }

private:
  bool _debug;
  ceres::GradientProblemSolver::Options _options;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__UPSAMPLER_HPP_
