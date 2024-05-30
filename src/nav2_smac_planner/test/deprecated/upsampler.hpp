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

#ifndef DEPRECATED__UPSAMPLER_HPP_
#define DEPRECATED__UPSAMPLER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <algorithm>
#include <utility>

#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/upsampler_cost_function.hpp"
#include "nav2_smac_planner/upsampler_cost_function_nlls.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::Upsampler
 * @brief A Conjugate Gradient 2D path upsampler implementation
 */
class Upsampler
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Upsampler
   */
  Upsampler() {}

  /**
   * @brief A destructor for nav2_smac_planner::Upsampler
   */
  ~Upsampler() {}

  /**
   * @brief Initialization of the Upsampler
   */
  void initialize(const OptimizerParams params)
  {
    _debug = params.debug;

    // General Params

    // 2 most valid options: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT
    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.max_num_iterations = params.max_iterations;  // 5000
    _options.max_solver_time_in_seconds = params.max_time;  // 5.0; // TODO

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;  // 1e-20;

    _options.min_line_search_step_size = params.advanced.min_line_search_step_size;  // 1e-30;
    _options.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    _options.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;  // 1e-30;
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
   * @return If Upsampler was successful
   */
  bool upsample(
    std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params,
    const int & upsample_ratio)
  {
    _options.max_solver_time_in_seconds = params.max_time;

    if (upsample_ratio != 2 && upsample_ratio != 4) {
      // invalid inputs
      return false;
    }

    const int param_ratio = upsample_ratio * 2.0;
    const int total_size = 2 * (path.size() * upsample_ratio - upsample_ratio + 1);
    double parameters[total_size];  // NOLINT

    // 20-4hz regularly, but dosnt work in faster cases
    // Linearly distribute initial poses for optimization
    // TODO(stevemacenski) generalize for 2x and 4x
    unsigned int next_pt;
    Eigen::Vector2d interpolated;
    std::vector<Eigen::Vector2d> temp_path;
    for (unsigned int pt = 0; pt != path.size() - 1; pt++) {
      next_pt = pt + 1;
      interpolated = (path[next_pt] + path[pt]) / 2.0;

      parameters[param_ratio * pt] = path[pt][0];
      parameters[param_ratio * pt + 1] = path[pt][1];
      temp_path.push_back(path[pt]);

      parameters[param_ratio * pt + 2] = interpolated[0];
      parameters[param_ratio * pt + 3] = interpolated[1];
      temp_path.push_back(interpolated);
    }

    parameters[total_size - 2] = path.back()[0];
    parameters[total_size - 1] = path.back()[1];
    temp_path.push_back(path.back());

    // Solve the upsampling problem
    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UpsamplerCostFunction(temp_path, params, upsample_ratio));
    ceres::Solve(_options, problem, parameters, &summary);


    path.resize(total_size / 2);
    for (int i = 0; i != total_size / 2; i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }

    // 10-15 hz, regularly
    // std::vector<Eigen::Vector2d> path_double_sampled;
    // for (int i = 0; i != path.size() - 1; i++) {  // last term should not be upsampled
    //   path_double_sampled.push_back(path[i]);
    //   path_double_sampled.push_back((path[i+1] + path[i]) / 2);
    // }

    // std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();
    // for (uint i = 1; i != path_double_sampled.size() - 1; i++) {
    //   ceres::CostFunction * cost_fn =
    //     new UpsamplerConstrainedCostFunction(path_double_sampled, params, 2, i);
    //   problem->AddResidualBlock(
    //     cost_fn, nullptr, &path_double_sampled[i][0], &path_double_sampled[i][1]);
    //   // locking initial coordinates unnecessary since there's no update between terms in NLLS
    // }

    // ceres::Solver::Summary summary;
    // _options.minimizer_type = ceres::LINE_SEARCH;
    // ceres::Solve(_options, problem.get(), &summary);

    // if (upsample_ratio == 4) {
    //   std::vector<Eigen::Vector2d> path_quad_sampled;
    //   for (int i = 0; i != path_double_sampled.size() - 1; i++) {
    //     path_quad_sampled.push_back(path_double_sampled[i]);
    //     path_quad_sampled.push_back((path_double_sampled[i+1] + path_double_sampled[i]) / 2.0);
    //   }

    //   std::unique_ptr<ceres::Problem> problem2 = std::make_unique<ceres::Problem>();
    //   for (uint i = 1; i != path_quad_sampled.size() - 1; i++) {
    //     ceres::CostFunction * cost_fn =
    //       new UpsamplerConstrainedCostFunction(path_quad_sampled, params, 4, i);
    //     problem2->AddResidualBlock(
    //       cost_fn, nullptr, &path_quad_sampled[i][0], &path_quad_sampled[i][1]);
    //   }

    //   ceres::Solve(_options, problem2.get(), &summary);

    //   path = path_quad_sampled;
    // } else {
    //   path = path_double_sampled;
    // }

    if (_debug) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }

    return true;
  }

private:
  bool _debug;
  ceres::GradientProblemSolver::Options _options;
};

}  // namespace nav2_smac_planner

#endif  // DEPRECATED__UPSAMPLER_HPP_
