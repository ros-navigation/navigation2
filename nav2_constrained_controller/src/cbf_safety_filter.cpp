// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/cbf_safety_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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
  // Saradagi Eq. 1, with Origin Autonomy convention: +x is forward,
  // robot reference (xr, yr) sits at the FRONT of the body (front
  // wheel pivot).  The four corners (front-right is P¹, then
  // back-right P², back-left P³, front-left P⁴) come from translating
  // (xr, yr) by ±dl along body-x and ±db along body-y, plus a -L
  // shift along body-x for the rear corners.
  const double L = params_->footprint_length;
  const double dl = params_->footprint_dl;
  const double db = params_->footprint_db;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  std::array<Point2D, 4> c;
  // P1: front-right
  c[0].x = xr + dl * ct - db * st;
  c[0].y = yr + dl * st + db * ct;
  // P2: back-right
  c[1].x = xr - L * ct - dl * ct - db * st;
  c[1].y = yr - L * st - dl * st + db * ct;
  // P3: back-left
  c[2].x = xr - L * ct - dl * ct + db * st;
  c[2].y = yr - L * st - dl * st - db * ct;
  // P4: front-left
  c[3].x = xr + dl * ct + db * st;
  c[3].y = yr + dl * st - db * ct;
  return c;
}

double CBFSafetyFilter::evalWall(const Wall & w, const Point2D & p)
{
  return w.lx * p.x + w.ly * p.y + w.c;
}

bool CBFSafetyFilter::emitOuterWallConstraint(
  const Wall & w, int wall_idx,
  const std::array<Point2D, 4> & corners,
  CbfConstraint & out) const
{
  // For each corner, h_c = L_w(Pc) since L_w(robot)=c >= 0 puts the
  // robot on the safe side; corners on the same side have L_w >= 0
  // (the wall is far) and corners that have CROSSED the wall produce
  // L_w < 0. We take the smallest L_w across all four corners as the
  // active barrier for this wall: that corner is the one most at risk
  // of clipping.
  double h_min = std::numeric_limits<double>::infinity();
  int worst_c = -1;
  for (int i = 0; i < 4; ++i) {
    const double h_i = evalWall(w, corners[i]);
    if (h_i < h_min) {
      h_min = h_i;
      worst_c = i;
    }
  }
  if (worst_c < 0) {return false;}

  // Skip walls way out of range (no point spending QP rows on walls
  // 5+ metres from the robot in a 1m-wide alley).
  // We use the perpendicular distance from the origin (= |c|) plus
  // the body half-length as a coarse activation gate. h_min already
  // includes the corner offset, so we test h_min directly.
  if (h_min > params_->wall_consideration_range) {return false;}

  // CBF row in QP form (≤): [lx, ly, (lx·Pcy - ly·Pcx)] · u ≤ γ·h_min
  // Derivation in cbf_safety_filter.hpp.
  const Point2D & Pc = corners[worst_c];
  const double r_term = w.lx * Pc.y - w.ly * Pc.x;
  // out.grad = Eigen::Vector3d(w.lx, w.ly, r_term);
  out.grad = Eigen::Vector3d(-w.lx, -w.ly, r_term);
  out.rhs = params_->cbf_gamma * h_min;
  out.h = h_min;
  out.corner_id = worst_c + 1;       // 1..4
  out.wall_id = wall_idx;
  out.kind = CbfConstraint::OUTER_WALL;
  return true;
}

void CBFSafetyFilter::emitInnerCornerConstraints(
  const Passage & p,
  const std::array<Point2D, 4> & corners,
  std::vector<CbfConstraint> & out) const
{
  if (!p.present) {return;}

  // Build a virtual wall through the two detected passage endpoints
  // (a, b). This is the "door line" — the line connecting the two
  // door corners. The robot must keep its near corners on the SAME
  // SIDE as the alley interior.
  //
  // We construct the line in the same normal-form as a real Wall and
  // orient the normal toward the robot origin.
  Wall door;
  door.id = -1;
  door.p1 = p.a;
  door.p2 = p.b;
  const double dx = p.b.x - p.a.x;
  const double dy = p.b.y - p.a.y;
  const double norm = std::hypot(dx, dy);
  if (norm < 1e-6) {return;}
  door.lx = -dy / norm;
  door.ly = dx / norm;
  door.c = (dy * p.a.x - dx * p.a.y) / norm;
  if (door.c < 0.0) {
    door.lx = -door.lx;
    door.ly = -door.ly;
    door.c = -door.c;
  }

  // Emit one constraint per corner closest to each end (a, b) of the
  // passage — the door corners are the inner perimeter the body must
  // not sweep through. We pick the two corners with the smallest
  // L_w(Pc) and emit a row each.
  std::array<double, 4> hs;
  for (int i = 0; i < 4; ++i) {hs[i] = evalWall(door, corners[i]);}
  // Take the two smallest.
  std::array<int, 4> idx{0, 1, 2, 3};
  std::sort(
    idx.begin(), idx.end(),
    [&hs](int a, int b) {return hs[a] < hs[b];});
  for (int k = 0; k < 2; ++k) {
    const int ci = idx[k];
    if (hs[ci] > params_->wall_consideration_range) {break;}
    CbfConstraint c;
    const Point2D & Pc = corners[ci];
    // c.grad = Eigen::Vector3d(
    //   door.lx, door.ly,
    //   door.lx * Pc.y - door.ly * Pc.x);
    c.grad = Eigen::Vector3d(
      -door.lx, -door.ly,
      door.lx * Pc.y - door.ly * Pc.x);
    c.rhs = params_->cbf_gamma * hs[ci];
    c.h = hs[ci];
    c.corner_id = ci + 1;
    c.wall_id = -1;
    c.kind = CbfConstraint::INNER_CORNER;
    out.push_back(c);
  }
}

CbfFilterResult CBFSafetyFilter::filter(
  const SceneSnapshot & snap,
  const geometry_msgs::msg::Twist & u_nom)
{
  CbfFilterResult res;

  // We work in BASE_LINK: robot pose is the origin, theta = 0. Body
  // corners are body-frame constants for a given footprint geometry.
  res.corners = bodyCorners(0.0, 0.0, 0.0);

  // 1. Outer-wall constraints, one per wall.
  for (size_t i = 0; i < snap.walls.size(); ++i) {
    CbfConstraint c;
    if (emitOuterWallConstraint(
        snap.walls[i], static_cast<int>(i), res.corners, c))
    {
      res.constraints.push_back(c);
    }
  }

  // 2. Inner-corner constraints from any detected passage.
  emitInnerCornerConstraints(snap.passage, res.corners, res.constraints);

  // 3. Track minimum h across all constraints — useful telemetry.
  res.min_h = std::numeric_limits<double>::infinity();
  for (const auto & c : res.constraints) {
    if (c.h < res.min_h) {res.min_h = c.h;}
  }
  if (res.constraints.empty()) {res.min_h = 0.0;}

  // 4. Assemble QP and solve.
  //   ∇h_i · u + γ·α(h_i) >= 0   <=>   -∇h_i · u <= γ·h_i
  // We stored grad and rhs already aligned with ≤-form (see
  // emitOuterWallConstraint), so A row = grad, b = rhs.
  const int m = static_cast<int>(res.constraints.size());
  Eigen::MatrixXd A(m, 3);
  Eigen::VectorXd b(m);
  for (int i = 0; i < m; ++i) {
    A.row(i) = res.constraints[i].grad.transpose();
    b(i) = res.constraints[i].rhs;
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

  res.u.linear.x = res.qp.u.x();
  res.u.linear.y = res.qp.u.y();
  res.u.angular.z = res.qp.u.z();
  return res;
}

}  // namespace nav2_constrained_controller
