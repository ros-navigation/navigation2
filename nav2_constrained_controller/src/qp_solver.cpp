// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// See header for algorithm overview.

#include "nav2_constrained_controller/qp_solver.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <vector>

namespace nav2_constrained_controller
{

QpResult solveProjectionQP(
  const Eigen::MatrixXd & A_in,
  const Eigen::VectorXd & b_in,
  const Eigen::Vector3d & u_min,
  const Eigen::Vector3d & u_max,
  const Eigen::Vector3d & u_nom,
  double w_slack,
  int max_iter,
  double tol)
{
  // Rows 0..m_in-1 are CBF constraints (soft when w_slack > 0).
  // Rows m_in..m-1 are box bounds (always hard).
  const int m_in = static_cast<int>(A_in.rows());
  const int m = m_in + 6;
  Eigen::MatrixXd A(m, 3);
  Eigen::VectorXd b(m);
  if (m_in > 0) {
    A.topRows(m_in) = A_in;
    b.head(m_in) = b_in;
  }
  for (int i = 0; i < 3; ++i) {
    A.row(m_in + 2 * i)     =  Eigen::RowVector3d::Unit(i);
    b(m_in + 2 * i)         =  u_max(i);
    A.row(m_in + 2 * i + 1) = -Eigen::RowVector3d::Unit(i);
    b(m_in + 2 * i + 1)     = -u_min(i);
  }

  Eigen::Vector3d u = u_nom;
  for (int i = 0; i < 3; ++i) {
    u(i) = std::clamp(u(i), u_min(i), u_max(i));
  }

  std::vector<int> active;
  active.reserve(m_in + 6);

  QpResult res;
  res.u = u;

  // KKT solve for a working set W (equality-constrained subproblem).
  //
  // Hard case  (w_slack = 0 or box rows):
  //   (A_W A_W^T + 1e-12·I) λ = A_W u_nom - b_W
  //
  // Soft case  (w_slack > 0, CBF rows only):
  //   (A_W A_W^T + (1/w_slack)·I_cbf) λ = A_W u_nom - b_W
  //
  // The (1/w_slack) Tikhonov term is the exact reduction of the slack-variable
  // KKT system (see header). Box rows always get 1e-12 (numerical stability only).
  const bool use_slack = (w_slack > 0.0);
  const double slack_reg = use_slack ? (1.0 / w_slack) : 1e-12;

  auto solveActive = [&](const std::vector<int> & W,
    Eigen::Vector3d & u_out, Eigen::VectorXd & lam_out) -> bool
    {
      if (W.empty()) {u_out = u_nom; lam_out.resize(0); return true;}
      const int k = static_cast<int>(W.size());
      Eigen::MatrixXd Aw(k, 3);
      Eigen::VectorXd bw(k);
      for (int i = 0; i < k; ++i) {
        Aw.row(i) = A.row(W[i]);
        bw(i)     = b(W[i]);
      }
      Eigen::MatrixXd M = Aw * Aw.transpose();
      for (int i = 0; i < k; ++i) {
        // CBF rows: soft regularisation. Box rows: numerical nudge only.
        M(i, i) += (use_slack && W[i] < m_in) ? slack_reg : 1e-12;
      }
      Eigen::VectorXd lam = M.ldlt().solve(Aw * u_nom - bw);
      if (!lam.allFinite()) {return false;}
      u_out   = u_nom - Aw.transpose() * lam;
      lam_out = lam;
      return true;
    };

  for (int it = 0; it < max_iter; ++it) {
    res.iterations = it + 1;

    // Find the most-violated constraint at current u.
    // For soft mode: active CBF constraints are skipped — their violation is
    // absorbed by slack ε = λ/w_slack and does not need to be resolved by
    // adding the constraint again (it is already in the active set).
    int worst = -1;
    double worst_v = tol;
    for (int i = 0; i < m; ++i) {
      const double v = A.row(i).dot(u) - b(i);
      if (v <= worst_v) {continue;}
      // Active soft CBF constraint: slack absorbs the violation — don't re-flag.
      if (use_slack && i < m_in) {
        const bool in_active =
          std::find(active.begin(), active.end(), i) != active.end();
        if (in_active) {continue;}
      }
      worst_v = v;
      worst   = i;
    }

    if (worst < 0) {
      // No unresolved violation. Check multiplier signs for optimality.
      Eigen::Vector3d u_try;
      Eigen::VectorXd lam;
      if (!solveActive(active, u_try, lam)) {res.ok = false; break;}
      int min_idx = -1;
      double min_lam = -1e-9;
      for (int i = 0; i < lam.size(); ++i) {
        if (lam(i) < min_lam) {min_lam = lam(i); min_idx = i;}
      }
      if (min_idx < 0) {u = u_try; break;}  // KKT satisfied
      active.erase(active.begin() + min_idx);
      if (!solveActive(active, u, lam)) {res.ok = false; break;}
      continue;
    }

    if (std::find(active.begin(), active.end(), worst) == active.end()) {
      active.push_back(worst);
    }
    Eigen::Vector3d u_try;
    Eigen::VectorXd lam;
    if (!solveActive(active, u_try, lam)) {res.ok = false; break;}
    u = u_try;
  }

  // Safety net 1: at max_iter only flag infeasibility for BOX constraints.
  // CBF constraint violations are legitimately absorbed by slack.
  if (res.iterations >= max_iter) {
    for (int i = m_in; i < m; ++i) {
      if (A.row(i).dot(u) > b(i) + 1e-4) {res.ok = false; break;}
    }
  }

  // Safety net 2: hard box clamp — never return out-of-envelope velocities.
  for (int i = 0; i < 3; ++i) {
    u(i) = std::clamp(u(i), u_min(i), u_max(i));
  }

  res.u       = u;
  res.n_active = static_cast<int>(active.size());
  res.deviation = (u - u_nom).norm();

  // Compute max_slack: ε_i = λ_i / w_slack for active CBF rows.
  if (use_slack && !active.empty()) {
    Eigen::Vector3d u_tmp;
    Eigen::VectorXd lam_final;
    if (solveActive(active, u_tmp, lam_final)) {
      for (int i = 0; i < static_cast<int>(active.size()); ++i) {
        if (active[i] < m_in && i < lam_final.size()) {
          res.max_slack = std::max(res.max_slack,
            std::max(0.0, lam_final(i)) / w_slack);
        }
      }
    }
  }

  return res;
}

}  // namespace nav2_constrained_controller
