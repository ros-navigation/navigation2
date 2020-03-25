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

#ifndef SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_
#define SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "Eigen/Core"

// doxogyn
// inline
// copies / optimization

// [done] waypoint smoothing term (smoothing)
// max curvature term (curvature)
// smooth curvature term (new, me)
// collision obsstacle term (collision)
// smooth obstacle term (vornoi)

/*
Other errors:
[planner_server-8] W0320 18:48:15.573891 16679 line_search.cc:758] Line search failed: Wolfe zoom bracket width: 7.87789e-10 too small with descent_direction_max_norm: 2.31001e-02.
[planner_server-8] Termination:                          FAILURE (Numerical failure in line search, failed to find a valid step size, (did not run out of iterations) using initial_step_size: 1.00000e+00, initial_cost: 2.85747e-04, initial_gradient: -1.44590e-06.)
[planner_server-8] W0320 22:10:09.060544 26622 line_search.cc:584] Line search failed: Wolfe bracketing phase shrank bracket width: 3.22458e-06, to < tolerance: 1e-09, with descent_direction_max_norm: 0.000251634, and failed to find a point satisfying the strong Wolfe conditions or a bracketing containing such a point. Accepting point found satisfying Armijo condition only, to allow continuation.
[planner_server-8] W0320 22:10:09.061851 26622 line_search.cc:726] Line search failed: Wolfe zoom phase passed a bracket which does not satisfy: bracket_low.gradient * (bracket_high.x - bracket_low.x) < 0 [1.91737173e-12 !< 0] with initial_position: [x: 0.00000000e+00, value: 2.85747015e-04, gradient: -1.44589730e-06, value_is_valid: 1, gradient_is_valid: 1], bracket_low: [x: 1.07133785e+00, value: 2.80767640e-04, gradient: -1.37853074e-06, value_is_valid: 1, gradient_is_valid: 1], bracket_high: [x: 1.07133646e+00, value: 2.80767646e-04, gradient: -1.37853083e-06, value_is_valid: 1, gradient_is_valid: 1], the most likely cause of which is the cost function returning inconsistent gradient & function values.
[planner_server-8] W0320 22:10:09.063752 26622 line_search_minimizer.cc:318] Terminating: Numerical failure in line search, failed to find a valid step size, (did not run out of iterations) using initial_step_size: 1.00000e+00, initial_cost: 2.85747e-04, initial_gradient: -1.44590e-06.
warning 3: [planner_server-8] W0323 14:01:33.240247 22652 line_search_direction.cc:86] Restarting non-linear conjugate gradients: 3.00844
*/

namespace smac_planner
{

class SmootherCostFunction : public ceres::SizedCostFunction<1, 1, 1>
{
public:
  SmootherCostFunction(
    unsigned char * & cmap,
    std::vector<Eigen::Vector2d> * pts,
    int i)
  : costmap(cmap),
    pt_minus(nullptr),
    pt_plus(nullptr)
  {
    pt_minus = & pts->at(i - 1);
    pt_plus = & pts->at(i + 1);
    Wsmooth = 0.2;
    Wcurve = 2.5;
  }

  virtual ~SmootherCostFunction() {}

  bool Evaluate(
    double const * const * parameters,
    double * residuals,
    double ** jacobians) const override
  {
    Eigen::Vector2d pt(parameters[0][0], parameters[0][1]);
    residuals[0] = 0.0;

    addSmoothingResidual(Wsmooth, pt, residuals[0]);
    addMaxCurvatureResidual(Wcurve, pt, residuals[0]);

    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[1][1] = jacobians[0][0] = jacobians[0][1] = jacobians[1][0] = 0.0;
      addSmoothingJacobian(Wsmooth, pt, jacobians[0][0], jacobians[1][0]);   
      addMaxCurvatureJacobian(Wcurve, pt, jacobians[0][0], jacobians[1][0]);
    }

    return true;
  }

  inline void addSmoothingResidual(const double & weight, const Eigen::Vector2d & pt, double & r) const
  {
    const Eigen::Vector2d & pt_p = * pt_plus;
    const Eigen::Vector2d & pt_m = * pt_minus;

    r += weight * (
      pt_p.dot(pt_p)
      - 4 * pt_p.dot(pt)
      + 2 * pt_p.dot(pt_m)
      + 4 * pt.dot(pt)
      - 4 * pt.dot(pt_m)
      + pt_m.dot(pt_m));  // objective function value
  }

  inline void addSmoothingJacobian(const double & weight, const Eigen::Vector2d & pt, double & j0, double & j1) const
  {
    j0 += weight * (- 4 * pt_minus->operator[](0) + 8 * pt[0] - 4 * pt_plus->operator[](0));  // xi x component of partial-derivative
    j1 += weight * (- 4 * pt_minus->operator[](1) + 8 * pt[1] - 4 * pt_plus->operator[](1));  // xi y component of partial-derivative
  }

  inline void addMaxCurvatureResidual(const double & weight, const Eigen::Vector2d & pt, double & r) const
  {
    const Eigen::Vector2d delta_xi(pt[0] - pt_minus->operator[](0), pt[1] - pt_minus->operator[](1));
    const Eigen::Vector2d delta_xi_p(pt_plus->operator[](0) - pt[0], pt_plus->operator[](1) - pt[1]); 
    const double & delta_xi_norm = delta_xi.norm();
    const double & delta_xi_p_norm = delta_xi_p.norm();
    const double & delta_xi_by_xi_p = delta_xi_norm * delta_xi_p_norm;
    const double & delta_phi_i = std::max(-1.0, std::min(1.0, acos(delta_xi.dot(delta_xi_p) / delta_xi_by_xi_p)));
    const double & turning_rad = delta_phi_i / delta_xi_norm;

    if (turning_rad < max_turning_radius) {
      return;
    }

    const double & diff = turning_rad - max_turning_radius;
    r += weight * diff * diff;  // objective function value for quadratic penalty
    std::cout << "R: " << r << std::endl;
  }

  inline void addMaxCurvatureJacobian(const double & weight, const Eigen::Vector2d & pt, double & j0, double & j1) const
  {
    const Eigen::Vector2d delta_xi(pt[0] - pt_minus->operator[](0), pt[1] - pt_minus->operator[](1));
    const Eigen::Vector2d delta_xi_p(pt_plus->operator[](0) - pt[0], pt_plus->operator[](1) - pt[1]); 
    const double & delta_xi_norm = delta_xi.norm();
    const double & delta_xi_p_norm = delta_xi_p.norm();
    const double & delta_xi_by_xi_p = delta_xi_norm * delta_xi_p_norm;
    const double & delta_phi_i = std::max(-1.0, std::min(1.0, acos(delta_xi.dot(delta_xi_p) / delta_xi_by_xi_p)));
    const double & turning_rad = delta_phi_i / delta_xi_norm;

    if (turning_rad < max_turning_radius) {
      return;
    }

    const double & partial_delta_phi_i_wrt_cost_delta_phi_i = -1 / std::sqrt(1 - std::pow(std::cos(delta_phi_i), 2));
    const Eigen::Vector2d ones = Eigen::Vector2d(1.0, 1.0);
    Eigen::Vector2d p1(1.0,0.0);//TODO
    Eigen::Vector2d p2(0.0,1.0);//TODO
    const Eigen::Vector2d jacobian = 2 * weight * (turning_rad - max_turning_radius) * (1 / delta_xi_norm * partial_delta_phi_i_wrt_cost_delta_phi_i * (-1 * p1 - p2) );//- (ones * delta_phi_i) / (delta_xi * delta_xi));
    j0 += weight * jacobian[0];  // xi x component of partial-derivative
    j1 += weight * jacobian[1];  // xi y component of partial-derivative
    std::cout << "J: " << jacobian[0] << " " <<jacobian[1] << std::endl;
  }  

protected:
  unsigned char * costmap;
  Eigen::Vector2d * pt_minus;
  Eigen::Vector2d * pt_plus;
  double Wsmooth;
  double Wcurve;
  double max_turning_radius = 0.15151515151;  //1/r*1.1 TODO
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_



  // inline void addSmoothingResidual(const double & weight, const double & pt_x, const double & pt_y, double & r0, double & r1) const
  // {
  //   r0 +=
  //     weight * (pt_plus->first * pt_plus->first -
  //     4 * pt_plus->first * pt_x +
  //     2 * pt_plus->first * pt_minus->first +
  //     4 * pt_x * pt_x -
  //     4 * pt_x * pt_minus->first +
  //     pt_minus->first * pt_minus->first);  // objective function value x
  //   r1 +=
  //     weight * (pt_plus->second * pt_plus->second -
  //     4 * pt_plus->second * pt_y +
  //     2 * pt_plus->second * pt_minus->second +
  //     4 * pt_y * pt_y -
  //     4 * pt_y * pt_minus->second +
  //     pt_minus->second * pt_minus->second);  // objective function value y
  // }

  // inline void addSmoothingJacobian(const double & weight, const double & pt_x, const double & pt_y, double & j0, double & j1) const
  // {
  //   j0 += weight * (/*pt_minus_two.first*/ - 4 * pt_minus->first + 8 * pt_x - 4 * pt_plus->first /*+ pt_plus_two.first*/);  // x derivative
  //   j1 += weight * (/*pt_minus_two.second*/ - 4 * pt_minus->second + 8 * pt_y - 4 * pt_plus->second /*+ pt_plus_two.second*/);  // y derivative
  // }
