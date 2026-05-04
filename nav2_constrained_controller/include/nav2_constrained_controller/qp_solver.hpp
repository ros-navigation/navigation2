// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// Tiny dense QP solver for the CBF safety filter. The problem is:
//
//     min_u  ½ ||u - u_nom||²       (cost: identity Hessian)
//     s.t.   A u <= b               (linear inequalities; CBF rewritten
//                                    into ≤-form)
//            l <= u <= u_max        (box bounds)
//
// With identity Hessian this is exactly Euclidean projection of u_nom
// onto the polytope defined by the constraints. We use a simple
// primal active-set method:
//
//   1. Start at u = u_nom (unconstrained optimum).
//   2. While any constraint i has A_i u > b_i + tol:
//        a. Add the most-violated constraint to the active set.
//        b. Solve the equality-constrained KKT system (small linear
//           solve via Eigen's dense LDLT).
//        c. If the resulting u still violates any non-active constraint,
//           re-add the worst one and continue.
//        d. If the multipliers are all non-negative, we're done; else
//           drop the most-negative one.
//
// For 3 variables and ≤8 inequalities the loop converges in a handful
// of iterations. We cap iterations defensively.

#ifndef NAV2_CONSTRAINED_CONTROLLER__QP_SOLVER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__QP_SOLVER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace nav2_constrained_controller
{

struct QpResult
{
  Eigen::Vector3d u{Eigen::Vector3d::Zero()};
  bool ok{true};
  int iterations{0};
  int n_active{0};
  double deviation{0.0};   // ||u - u_nom||
};

// Solve the box-constrained QP described above.
//   A   : (m × 3)
//   b   : (m)
//   u_min, u_max : (3, 3)
//   u_nom: (3)
QpResult solveProjectionQP(
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::Vector3d & u_min,
  const Eigen::Vector3d & u_max,
  const Eigen::Vector3d & u_nom,
  int max_iter = 50,
  double tol = 1e-7);

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__QP_SOLVER_HPP_
