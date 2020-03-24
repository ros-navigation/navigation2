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

// copies / optimization

// hybridA* has a step after to interpolate up the resolution because sampling every  0.5-1m and dont want jerky motion
// once doing Hybrid A* should downsample resolution of research for that.

// should smooth term more
//  - sparser so less messy
//  - take larger neighborhood
//  - be able to drop points and resample them up later
//  - incentivize a shorter path (?)
//  - non-L2 but also angle change between them?
//  - take shorter-cuts and eliminate waypoints
// I do suspect when doing the curvature term that should help a bit

// maybe we should be planning sparser, we just want to know that its possible to get through a space as a route, then more up to the controller to decide how it should follow that route
// >> 5cm (10-15cm) jump points but at the same resolution. then optimizer to smooth, and then upsample.
//  - optimization/upsample phase will still ensure valid non-collision path.
//  - faster planning & faster optimization
//  - original search with k-d tree / multiresolution to get a "we know we can", then the optimiziation, then the sampling. Approx. Cell decomposition

// maybe quick heueristic: if i not in line with i-1, i+1, see if valid (same or lower cost), if so, do it. Mhm, might only work axially aligned?

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
    options_.minimizer_type = ceres::LINE_SEARCH;                              // TRUST_REGION, LINE_SEARCH
    options_.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT; // LBFGS, BFGS, NONLINEAR_CONJUGATE_GRADIENT
    options_.line_search_type = ceres::WOLFE;                                  // WOLFE, ARMIJO (wolfe must be used for L/BFGS)
    options_.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;         // FLETCHER_REEVES, POLAK_RIBIERE, HESTENES_STIEFEL
    // options_.max_lbfgs_rank = ;                                             // default 20 for LBFGS
    // options_.use_approximate_eigenvalue_bfgs_scaling = ;                    // default false for LBFGS and BFGS
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
    options_.max_solver_time_in_seconds = 1.0; // 1e4 default
    // options_.num_threads = ; // 1 default for evaluating jacobian
    options_.function_tolerance = 1e-6; // 1e-6 deafult, TODO results in warning 3
    // options_.gradient_tolerance = ; // 1e-10 default, maybe important?
    // options_.linear_solver_type = ; // maybe important?
    // options_.preconditioner_type = ; // 
    // options_.visibility_clustering_type = ; //
    // options_.dense_linear_algebra_library_type
    // options_.sparse_linear_algebra_library_type
    // options_.use_explicit_schur_complement
    // options_.use_post_ordering
    // options_.jacobi_scaling
    // options_.eta = 
    // options_.max_linear_solver_iterations = ; // used by  and CGNR
    // options_.min_linear_solver_iterations = ; // used by ITERATIVE_SCHUR and CGNR
    // options_.dynamic_sparsity = ; // used by SPARSE_NORMAL_CHOLESK
  }

  /**
   * @brief Initialization of the smoother
   * @return If smoothing was successful
   */
  bool smooth(std::vector<Eigen::Vector2d> & path, unsigned char * & char_costmap)
  {
    std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();

    // populate our optimization problem
    for (uint i = 1; i != path.size() - 1; i++) {
      ceres::CostFunction * cost_fn = new SmootherCostFunction(char_costmap, & path, i);
      problem->AddResidualBlock(cost_fn, nullptr, & path[i][0], & path[i][1]);
    }

    // solve our optimization problem
    ceres::Solver::Summary summary;
    ceres::Solve(options_, problem.get(), &summary);
    if (debug_) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable()) {
      return false;
    }

    return true;
  }

private:
  bool debug_;
  ceres::Solver::Options options_;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NON_LINEAR_CONJUGATE_GRADIENT_SMOOTHER_HPP_
