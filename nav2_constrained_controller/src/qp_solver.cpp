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
  int max_iter,
  double tol)
{
  // Stack box bounds onto the inequality matrix.
  //   u_i <= u_max_i     ->   e_i^T u <=  u_max_i
  //  -u_i <= -u_min_i    ->  -e_i^T u <= -u_min_i
  const int m_in = static_cast<int>(A_in.rows());
  const int m = m_in + 6;
  Eigen::MatrixXd A(m, 3);
  Eigen::VectorXd b(m);
  if (m_in > 0) {
    A.topRows(m_in) = A_in;
    b.head(m_in) = b_in;
  }
  for (int i = 0; i < 3; ++i) {
    A.row(m_in + 2 * i) = Eigen::RowVector3d::Unit(i);
    b(m_in + 2 * i) = u_max(i);
    A.row(m_in + 2 * i + 1) = -Eigen::RowVector3d::Unit(i);
    b(m_in + 2 * i + 1) = -u_min(i);
  }

  Eigen::Vector3d u = u_nom;
  // Project onto box first so initial guess is feasible w.r.t. the
  // box, even if the inequalities push it.
  for (int i = 0; i < 3; ++i) {
    u(i) = std::clamp(u(i), u_min(i), u_max(i));
  }

  std::vector<int> active;  // indices into A's rows
  active.reserve(8);

  QpResult res;
  res.u = u;
  res.iterations = 0;

  // Convenience: solve KKT for a given working set.
  //
  //   min ½ ||u - u_nom||²    s.t.  A_W u = b_W
  //
  // Stationarity: u - u_nom + A_W^T λ = 0 -> u = u_nom - A_W^T λ
  // Feasibility: A_W (u_nom - A_W^T λ) = b_W
  //              -> (A_W A_W^T) λ = A_W u_nom - b_W
  //
  // For small problems this is fine to solve directly.
  auto solveActive = [&](const std::vector<int> & W,
      Eigen::Vector3d & u_out, Eigen::VectorXd & lam_out) -> bool {
      if (W.empty()) {
        u_out = u_nom;
        lam_out.resize(0);
        return true;
      }
      const int k = static_cast<int>(W.size());
      Eigen::MatrixXd Aw(k, 3);
      Eigen::VectorXd bw(k);
      for (int i = 0; i < k; ++i) {
        Aw.row(i) = A.row(W[i]);
        bw(i) = b(W[i]);
      }
      // Lambda solve.
      Eigen::MatrixXd M = Aw * Aw.transpose();
      Eigen::VectorXd r = Aw * u_nom - bw;
      // Tikhonov nudge for ill-conditioned active sets.
      for (int i = 0; i < k; ++i) {M(i, i) += 1e-12;}
      Eigen::VectorXd lam = M.ldlt().solve(r);
      if (!lam.allFinite()) {return false;}
      u_out = u_nom - Aw.transpose() * lam;
      lam_out = lam;
      return true;
    };

  for (int it = 0; it < max_iter; ++it) {
    res.iterations = it + 1;

    // Find the most-violated inequality at current u.
    int worst = -1;
    double worst_v = tol;
    for (int i = 0; i < m; ++i) {
      const double v = A.row(i).dot(u) - b(i);
      if (v > worst_v) {
        worst_v = v;
        worst = i;
      }
    }

    if (worst < 0) {
      // Feasible. Now check Lagrange multipliers — if any are
      // strictly negative, dropping that constraint will improve the
      // cost.
      Eigen::Vector3d u_try;
      Eigen::VectorXd lam;
      if (!solveActive(active, u_try, lam)) {
        res.ok = false;
        break;
      }
      // Smallest multiplier (by sign) signals an active constraint
      // that is not really binding; remove it.
      int min_idx = -1;
      double min_lam = -1e-9;
      for (int i = 0; i < lam.size(); ++i) {
        if (lam(i) < min_lam) {
          min_lam = lam(i);
          min_idx = i;
        }
      }
      if (min_idx < 0) {
        // All multipliers nonneg → KKT satisfied. Done.
        u = u_try;
        break;
      }
      active.erase(active.begin() + min_idx);
      if (!solveActive(active, u, lam)) {
        res.ok = false;
        break;
      }
      continue;
    }

    // Add worst-violator to active set if not already present.
    if (std::find(active.begin(), active.end(), worst) == active.end()) {
      active.push_back(worst);
    }
    Eigen::Vector3d u_try;
    Eigen::VectorXd lam;
    if (!solveActive(active, u_try, lam)) {
      res.ok = false;
      break;
    }
    u = u_try;
  }

  res.u = u;
  res.n_active = static_cast<int>(active.size());
  res.deviation = (u - u_nom).norm();
  return res;
}

}  // namespace nav2_constrained_controller
