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

#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "smac_planner/types.hpp"

#include "ceres/ceres.h"

// doxogyn
// inline
// copies / optimization

// hybridA* has a step after to interpolate up the resolution because sampling every  0.5-1m and dont want jerky motion
// once doing Hybrid A* should downsample resolution of research for that.

// differentation give exact derivatives for each term

// do both CG and GD, and ceres to test

namespace smac_planner
{

// is there a way to have it be given the pt + 1 and pt - 1? I dont like this copy.
// TODO why arent they moving much? Maybe setup problem wrong... hill climbing for many functions to solve for {b1,b2,b3,bN} variables, not each pair set of (X,Y)
class HolonomicPathSmootherCostFunction : public ceres::SizedCostFunction<2, 1, 1>
{
public:
  HolonomicPathSmootherCostFunction(
    unsigned char * & cmap,
    std::vector<DoubleCoordinates> * pts,
    int i)
  : costmap(cmap),
    path(pts),
    index(i)
  {
  }

  virtual ~HolonomicPathSmootherCostFunction() {}

  bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override
  {
    const double & pt_x = parameters[0][0];
    const double & pt_y = parameters[1][0];

    const DoubleCoordinates & pt_minus = path->at(index - 1);
    const DoubleCoordinates & pt_plus = path->at(index + 1);

    DoubleCoordinates pt_minus_two;
    if (index - 2 > 0) {
      pt_minus_two = path->at(index - 2);
    } else 
    {
      pt_minus_two = std::pair<double, double>(0.0, 0.0);
    }

    DoubleCoordinates pt_plus_two;
    if (index + 2 < static_cast<int>(path->size())) {
      pt_plus_two = path->at(index + 2);
    } else 
    {
      pt_plus_two = std::pair<double, double>(0.0, 0.0);
    }

    double Ws = 1;

    residuals[0] =
      Ws * (pt_plus.first * pt_plus.first -
      4 * pt_plus.first * pt_x +
      2 * pt_plus.first * pt_minus.first +
      4 * pt_x * pt_x -
      4 * pt_x * pt_minus.first +
      pt_minus.first * pt_minus.first);  // objective function value x
    residuals[1] =
      Ws * (pt_plus.second * pt_plus.second -
      4 * pt_plus.second * pt_y +
      2 * pt_plus.second * pt_minus.second +
      4 * pt_y * pt_y -
      4 * pt_y * pt_minus.second +
      pt_minus.second * pt_minus.second);  // objective function value y

    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = Ws * (/*pt_minus_two.first*/ - 4 * pt_minus.first + 8 * pt_x - 4 * pt_plus.first /*+ pt_plus_two.first*/);  // x derivative
      jacobians[1][0] = Ws * (/*pt_minus_two.second*/ - 4 * pt_minus.second + 8 * pt_y - 4 * pt_plus.second /*+ pt_plus_two.second*/);  // y derivative
      /*
       [planner_server-8] W0320 18:46:12.466234 16582 line_search.cc:726]:
         Line search failed: Wolfe zoom phase passed a bracket which does not satisfy:
         bracket_low.gradient * (bracket_high.x - bracket_low.x) < 0 [4.57437311e-10 !< 0] with initial_position: 
         [x: 0.00000000e+00, value: 5.60751553e-02, gradient: -5.63590112e-04, value_is_valid: 1, gradient_is_valid: 1], 
         bracket_low: [x: 1.00914504e+00, value: 4.59722852e-02, gradient: -5.19556079e-04, value_is_valid: 1, gradient_is_valid: 1],
         bracket_high: [x: 1.00914416e+00, value: 4.59722933e-02, gradient: -5.19556115e-04, value_is_valid: 1, gradient_is_valid: 1],
         the most likely cause of which is the cost function returning inconsistent gradient & function values.

      if added the additional ones to gradient without updating residual
      */
      jacobians[0][1] = 0.0;  // TODO does this require?
      jacobians[1][1] = 0.0;  // TODO does this require?
    }

/*
Other errors:
[planner_server-8] W0320 18:48:15.573891 16679 line_search.cc:758] Line search failed: Wolfe zoom bracket width: 7.87789e-10 too small with descent_direction_max_norm: 2.31001e-02.

*/
    return true;
  }

protected:
  unsigned char * costmap;
  std::vector<DoubleCoordinates> * path;
  int index;
};

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
    // options_.function_tolerance = ; // 1e-6 deafult, maybe important?
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
  bool smooth(std::vector<DoubleCoordinates> & path, unsigned char * & char_costmap)
  {
    std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();

    // TODO remove when done.
    std::vector<DoubleCoordinates> initial_path;
    if (debug_) {
      initial_path = path;      
    }

    // populate our optimization problem
    for (uint i = 1; i != path.size() - 1; i++) {
      ceres::CostFunction * cost_fn = new HolonomicPathSmootherCostFunction(char_costmap, & path, i);
      problem->AddResidualBlock(cost_fn, nullptr, &path[i].first, &path[i].second);
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
