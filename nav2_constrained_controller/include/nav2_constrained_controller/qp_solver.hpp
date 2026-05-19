// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// Tiny dense QP solver for the CBF safety filter.
//
// Hard-constraint problem (w_slack = 0):
//
//     min_u  ½ ||u - u_nom||²
//     s.t.   A u ≤ b          (CBF constraints, ≤-form)
//            u_min ≤ u ≤ u_max  (box, always hard)
//
// Soft-constraint problem (w_slack > 0):
//
//     min_{u,ε}  ½ ||u - u_nom||²  +  (w_slack/2) ||ε||²
//     s.t.       A u ≤ b + ε        (CBF soft via per-constraint slack ε ≥ 0)
//                u_min ≤ u ≤ u_max  (box always hard)
//
// The soft formulation is always feasible regardless of obstacle configuration.
// Mathematically the slack reduces to Tikhonov regularisation of the KKT Gram
// matrix: CBF rows get diagonal += 1/w_slack, box rows keep 1e-12 (numerical).
// The active-set structure, multiplier sign checks and convergence criterion are
// identical to the hard case.  w_slack = 0 reproduces the original hard solver.
//
// Primal active-set method:
//   1. Start at u = clamp(u_nom, box).
//   2. While a non-active CBF constraint is violated OR a box constraint is violated:
//        a. Add the worst violator to the active set.
//        b. Solve the soft KKT system via Eigen LDLT.
//        c. If multipliers all ≥ 0, done; else drop the most-negative one.

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
  bool ok{true};           // false only if box bounds infeasible or LDLT failed
  int iterations{0};
  int n_active{0};
  double deviation{0.0};   // ||u* - u_nom||
  double max_slack{0.0};   // max ε_i = max λ_i/w_slack across active CBF rows
                           // 0 when all hard constraints satisfied; > 0 means
                           // the safety margin was partially relaxed this tick.
};

// Solve the (soft-)constrained projection QP.
//   A       : (m_cbf × 3) — CBF constraint rows only (box added internally)
//   b       : (m_cbf)
//   u_min, u_max : box bounds (always hard)
//   u_nom   : nominal velocity
//   w_slack : slack weight > 0 → soft CBF; 0 → hard (original behaviour)
QpResult solveProjectionQP(
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::Vector3d & u_min,
  const Eigen::Vector3d & u_max,
  const Eigen::Vector3d & u_nom,
  double w_slack = 0.0,
  int max_iter = 50,
  double tol = 1e-7);

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__QP_SOLVER_HPP_
