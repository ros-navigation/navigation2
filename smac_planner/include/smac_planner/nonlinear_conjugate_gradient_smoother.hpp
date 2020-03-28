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

// maybe we should be planning sparser, we just want to know that its possible to get through a space as a route, then more up to the controller to decide how it should follow that route
// >> 5cm (10-15cm) jump points but at the same resolution. then optimizer to smooth, and then upsample.
//  - optimization/upsample phase will still ensure valid non-collision path.
//  - faster planning & faster optimization
//  - original search with k-d tree / multiresolution to get a "we know we can", then the optimiziation, then the sampling. Approx. Cell decomposition

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
  void initialize(const bool & debug) {
    debug_ = debug;

    // Solver options
    // OPTIMIZATION tune parameters
    options_.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    options_.line_search_type = ceres::WOLFE;                                  // WOLFE, ARMIJO (wolfe must be used for L/BFGS)
    options_.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;       // FLETCHER_REEVES, POLAK_RIBIERE, HESTENES_STIEFEL
    options_.line_search_interpolation_type = ceres::CUBIC;                    // CUBIC, QUADRATIC, BISECTION
    // options_.min_line_search_step_size = 1e-5;
    // options_.line_search_sufficient_function_decrease = ;// 1e-4 default
    // options_.max_line_search_step_contraction = ; // 1e-3 default
    // options_.min_line_search_step_contraction = ; // 0.6 default
    // options_.max_num_line_search_step_size_iterations = ;// 20 default
    // options_.max_num_line_search_direction_restarts = ; // 5 default
    // options_.line_search_sufficient_curvature_decrease = ;// 0.9 default
    // options_.max_line_search_step_expansion = ; // 10 default
    options_.max_num_iterations = 500; // 50 default
    options_.max_solver_time_in_seconds = 0.100; // 1e4 default. 100ms
    // options_.function_tolerance = 1e-6; // 1e-6 deafult, TODO results in warning 3: also cuases a crash to specifically add??
    // options_.gradient_tolerance = ; // 1e-10 default, maybe important?
    // options_.parameter_tolerance = ; // 1e-8  tol

    if (debug_) {
      options_.minimizer_progress_to_stdout = true;      
    }
  }

  /**
   * @brief Initialization of the smoother
   * @return If smoothing was successful
   */
  bool smooth(std::vector<Eigen::Vector2d> & path, const MinimalCostmap * costmap)
  {

    //OPTIMIZATION move this to planner to cut down on conversions
    double parameters[path.size() * 2];
    for (uint i = 0; i != path.size(); i++) {
      parameters[2 * i] = path[i][0];
      parameters[2 * i + 1] = path[i][1];
    }

    ceres::GradientProblemSolver::Options options;
    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UnconstrainedSmootherCostFunction(path.size(), costmap));
    ceres::Solve(options, problem, parameters, &summary);

    if (debug_) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable()) {
      return false;
    }

    //OPTIMIZATION move this to planner to cut down on conversions
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
