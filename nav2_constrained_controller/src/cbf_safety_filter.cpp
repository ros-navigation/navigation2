// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/cbf_safety_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "nav2_constrained_controller/parameter_handler.hpp"

namespace nav2_constrained_controller
{

CBFSafetyFilter::CBFSafetyFilter(const Parameters * params)
: params_(params)
{
}

std::array<Point2D, 4> CBFSafetyFilter::bodyCorners(
  double xr, double yr, double theta) const
{
  // Saradagi Eq. 1 convention, body centred on base_link.
  // Rectangle: (L/2 + dl) longitudinal half-length, db lateral half-width.
  // P1 front-right, P2 back-right, P3 back-left, P4 front-left.
  const double Lext = 0.5 * params_->footprint_length + params_->footprint_dl;
  const double db   = params_->footprint_db;
  const double ct   = std::cos(theta);
  const double st   = std::sin(theta);
  std::array<Point2D, 4> c;
  c[0].x = xr + Lext * ct + db * st;   c[0].y = yr + Lext * st - db * ct;
  c[1].x = xr - Lext * ct + db * st;   c[1].y = yr - Lext * st - db * ct;
  c[2].x = xr - Lext * ct - db * st;   c[2].y = yr - Lext * st + db * ct;
  c[3].x = xr + Lext * ct - db * st;   c[3].y = yr + Lext * st + db * ct;
  return c;
}

CbfFilterResult CBFSafetyFilter::filter(
  const EsdfGrid & esdf,
  const geometry_msgs::msg::Twist & u_nom)
{
  CbfFilterResult res;
  res.corners = bodyCorners(0.0, 0.0, 0.0);
  res.min_h   = std::numeric_limits<double>::infinity();

  // Pre-compute u_nom as Eigen for constraint debug fields (Au, margin).
  const Eigen::Vector3d u_nom_e(u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);

  // Perimeter sampling: query ESDF at points along each side of the robot body.
  // Point density is set by cbf_sample_spacing_m (m): each side gets
  //   ceil(side_length / spacing) + 1 samples, so a smaller spacing models the
  //   body polygon more finely (live-tunable for testing; footprint-proof — the
  //   counts recompute from the rectangle dims). Long sides include the corners;
  //   short sides add only their interior points (corners already placed by the
  //   long sides). Door frames / posts contact the MIDDLE of a side — denser
  //   sampling closes the gaps a sparse layout would miss.
  //
  // corner_id encodes which quadrant each sample belongs to for RViz colouring:
  //   1 = front-right (x≥0, y<0)   2 = back-right  (x<0, y<0)
  //   3 = back-left   (x<0, y≥0)   4 = front-left  (x≥0, y≥0)
  const double Lext = 0.5 * params_->footprint_length + params_->footprint_dl;
  const double db   = params_->footprint_db;
  // ── TEST: forced 1 cm perimeter sampling. Bypasses the param + 0.02 floor.
  //    Revert by deleting the hardcode line and restoring the commented line.
  // const double spacing = std::max(0.02, params_->cbf_sample_spacing_m);
  const double spacing = 0.01;

  // Samples-per-side from the target spacing (≥ 2 so the endpoints/corners exist).
  const int n_long  = std::max(2,
    static_cast<int>(std::ceil((2.0 * Lext) / spacing)) + 1);
  const int n_short = std::max(2,
    static_cast<int>(std::ceil((2.0 * db) / spacing)) + 1);

  struct Sample { Point2D p; int corner_id; };
  std::vector<Sample> samples;
  samples.reserve(static_cast<size_t>(2 * n_long + 2 * n_short));

  // Long sides (y = ±db): n_long samples each, including the corners.
  for (int k = 0; k < n_long; ++k) {
    const double t  = k / static_cast<double>(n_long - 1);
    const double px = -Lext + t * 2.0 * Lext;
    const int id_right = (px >= 0.0) ? 1 : 2;
    const int id_left  = (px >= 0.0) ? 4 : 3;
    samples.push_back({{px, -db}, id_right});
    samples.push_back({{px, +db}, id_left});
  }
  // Short sides (x = ±Lext): INTERIOR samples only (corners already added above).
  for (int k = 1; k < n_short - 1; ++k) {
    const double t  = k / static_cast<double>(n_short - 1);
    const double py = -db + t * 2.0 * db;
    samples.push_back({{ Lext, py}, (py >= 0.0) ? 4 : 1});  // front: FL or FR
    samples.push_back({{-Lext, py}, (py >= 0.0) ? 3 : 2});  // rear:  BL or BR
  }
  const int ns = static_cast<int>(samples.size());

  for (int si = 0; si < ns; ++si) {
    const Point2D & pc = samples[si].p;
    const auto q = esdf.query(pc.x, pc.y);

    if (!q.valid) {continue;}

    const double h = q.d - params_->esdf_d_safe;

    // Range gate: skip samples far from any obstacle.
    if (h > params_->wall_consideration_range) {continue;}

    // Degenerate gradient (e.g. exactly on the medial axis): skip.
    if (std::hypot(q.gx, q.gy) < 1e-6) {continue;}

    // CBF condition: ∇d^T · J_i · u ≥ -γ·h
    // J_i = [[1, 0, -py_i], [0, 1, px_i]]  at θ=0
    // QP ≤-form:  [-gx, -gy, gx·py - gy·px] · u  ≤  γ·h
    CbfConstraint c;
    const double lever = q.gx * pc.y - q.gy * pc.x;
    c.grad      = Eigen::Vector3d(-q.gx, -q.gy, lever);
    c.rhs       = params_->cbf_gamma * h;
    c.h         = h;
    c.corner_id = samples[si].corner_id;
    // Debug fields.
    c.Au          = c.grad.dot(u_nom_e);
    c.margin      = c.rhs - c.Au;
    c.gx          = q.gx; c.gy = q.gy;
    c.sample_x    = pc.x; c.sample_y = pc.y;
    c.predict_step = 0;

    res.constraints.push_back(c);
    if (h < res.min_h) {res.min_h = h;}
  }

  if (res.constraints.empty()) {res.min_h = 0.0;}

  // ── Predictive CBF ──────────────────────────────────────────────────────
  // For each future step j=1..N, predict where each perimeter sample will be
  // if the robot continues with u_nom, then add a CBF constraint based on the
  // predicted ESDF reading at that future position.
  //
  // This fixes the Lie-derivative-degeneracy failure: when the robot moves
  // parallel to a wall that narrows ahead, the reactive CBF sees L_g h ≈ 0 at
  // the current position (constraint trivially satisfied) and does nothing.
  // The predicted positions reveal the upcoming narrowing — h_pred < h_curr —
  // and generate tighter constraints that force the QP to correct NOW.
  //
  // Constraint formulation: at step j the predicted sample position is
  //   p_pred = p_curr + τ·[vx - wz·py, vy + wz·px]   where τ = j·predict_dt
  // We query the ESDF gradient (gx, gy) and distance h at p_pred, then build
  // the QP row with the CURRENT Jacobian J(p_curr) — because u is chosen NOW
  // and the Jacobian is evaluated at the current robot state (θ=0, body at origin).
  {
    const int    N   = params_->cbf_n_predict_steps;
    const double dt  = params_->cbf_predict_dt;
    const double vx  = u_nom.linear.x;
    const double vy  = u_nom.linear.y;
    const double wz  = u_nom.angular.z;

    for (int step = 1; step <= N; ++step) {
      const double tau = step * dt;

      for (int si = 0; si < ns; ++si) {
        const Point2D & pc = samples[si].p;

        // Predict sample position: p + τ · J(p) · u_nom
        const double pred_x = pc.x + tau * (vx - wz * pc.y);
        const double pred_y = pc.y + tau * (vy + wz * pc.x);

        const auto q = esdf.query(pred_x, pred_y);
        if (!q.valid) {continue;}

        const double h = q.d - params_->esdf_d_safe;
        if (h > params_->wall_consideration_range) {continue;}
        if (std::hypot(q.gx, q.gy) < 1e-6) {continue;}

        // QP row uses gradient at PREDICTED position (sees upcoming wall)
        // but lever arm at CURRENT position (robot Jacobian is at θ=0 now).
        CbfConstraint c;
        const double lever = q.gx * pc.y - q.gy * pc.x;
        c.grad      = Eigen::Vector3d(-q.gx, -q.gy, lever);
        c.rhs       = params_->cbf_gamma * h;
        c.h         = h;
        c.corner_id = samples[si].corner_id;
        // Debug fields.
        c.Au           = c.grad.dot(u_nom_e);
        c.margin       = c.rhs - c.Au;
        c.gx           = q.gx; c.gy = q.gy;
        c.sample_x     = pred_x; c.sample_y = pred_y;
        c.predict_step = step;

        res.constraints.push_back(c);
        if (h < res.min_h) {res.min_h = h;}
      }
    }
  }

  // Assemble QP: min ½||u - u_nom||²  s.t. Au ≤ b, u_min ≤ u ≤ u_max
  const int m = static_cast<int>(res.constraints.size());
  Eigen::MatrixXd A(m, 3);
  Eigen::VectorXd b(m);
  for (int i = 0; i < m; ++i) {
    A.row(i) = res.constraints[i].grad.transpose();
    b(i)     = res.constraints[i].rhs;
  }

  const Eigen::Vector3d u_min(
    -params_->v_linear_max,
    -params_->v_lateral_max,
    -params_->v_angular_max);
  const Eigen::Vector3d u_max(
    params_->v_linear_max,
    params_->v_lateral_max,
    params_->v_angular_max);
  const auto t0 = std::chrono::steady_clock::now();
  res.qp = solveProjectionQP(A, b, u_min, u_max, u_nom_e, params_->cbf_slack_weight);
  const auto t1 = std::chrono::steady_clock::now();
  res.solve_time_us =
    std::chrono::duration<double, std::micro>(t1 - t0).count();

  res.u.linear.x  = res.qp.u.x();
  res.u.linear.y  = res.qp.u.y();
  res.u.angular.z = res.qp.u.z();
  return res;
}

}  // namespace nav2_constrained_controller
