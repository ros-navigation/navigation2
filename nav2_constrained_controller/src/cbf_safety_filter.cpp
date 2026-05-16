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

  // One constraint per body corner.
  for (int i = 0; i < 4; ++i) {
    const Point2D & pc = res.corners[i];
    const auto q = esdf.query(pc.x, pc.y);

    if (!q.valid) {continue;}

    const double h = q.d - params_->esdf_d_safe;

    // Range gate: skip constraints far from any obstacle.
    if (h > params_->wall_consideration_range) {continue;}

    // Degenerate gradient (e.g. exactly on the medial axis): skip.
    if (std::hypot(q.gx, q.gy) < 1e-6) {continue;}

    // CBF condition: ∇d^T · J_i · u ≥ -γ·h
    // J_i = [[1, 0, -py_i], [0, 1, px_i]]  at θ=0
    // QP ≤-form:  [-gx, -gy, gx·py - gy·px] · u  ≤  γ·h
    //
    // The caller suppresses wz in u_nom when inside a narrow passage so
    // the lever arm term (gx·py − gy·px)·wz does not create contradictory
    // constraints at tight clearances. The lever arm is kept here so the
    // filter remains kinematically correct.
    CbfConstraint c;
    const double lever = q.gx * pc.y - q.gy * pc.x;
    c.grad      = Eigen::Vector3d(-q.gx, -q.gy, lever);
    c.rhs       = params_->cbf_gamma * h;
    c.h         = h;
    c.corner_id = i + 1;  // 1..4

    res.constraints.push_back(c);
    if (h < res.min_h) {res.min_h = h;}
  }

  if (res.constraints.empty()) {res.min_h = 0.0;}

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
  const Eigen::Vector3d u_nom_e(
    u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);

  const auto t0 = std::chrono::steady_clock::now();
  res.qp = solveProjectionQP(A, b, u_min, u_max, u_nom_e);
  const auto t1 = std::chrono::steady_clock::now();
  res.solve_time_us =
    std::chrono::duration<double, std::micro>(t1 - t0).count();

  res.u.linear.x  = res.qp.u.x();
  res.u.linear.y  = res.qp.u.y();
  res.u.angular.z = res.qp.u.z();
  return res;
}

}  // namespace nav2_constrained_controller
