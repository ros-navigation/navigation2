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
  // Saradagi Eq. 1, Origin Autonomy convention: +x is forward, body is
  // CENTERED on base_link (matches URDF base_link at body geometric
  // center and costmap footprint [[±0.45, ±0.375]]). The CBF rectangle
  // extends ±(L/2 + dl) in body-x and ±db in body-y. Corners P¹–P⁴ are:
  //   P¹ front-right, P² back-right, P³ back-left, P⁴ front-left.
  const double Lext = 0.5 * params_->footprint_length + params_->footprint_dl;
  const double db = params_->footprint_db;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  std::array<Point2D, 4> c;
  // P1: front-right (body-frame (+Lext, -db))
  c[0].x = xr + Lext * ct + db * st;
  c[0].y = yr + Lext * st - db * ct;
  // P2: back-right (body-frame (-Lext, -db))
  c[1].x = xr - Lext * ct + db * st;
  c[1].y = yr - Lext * st - db * ct;
  // P3: back-left (body-frame (-Lext, +db))
  c[2].x = xr - Lext * ct - db * st;
  c[2].y = yr - Lext * st + db * ct;
  // P4: front-left (body-frame (+Lext, +db))
  c[3].x = xr + Lext * ct - db * st;
  c[3].y = yr + Lext * st + db * ct;
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
  // Segment-distance CBF.
  //
  // For each (corner Pc, wall segment [p1, p2]) pair we compute the
  // closest point on the segment and build the CBF row from that
  // point's geometry. Three Voronoi regions of the segment yield
  // three constraint forms, all linear in u with the same Saradagi
  // structure (effective wall normal n, and a wz lever-arm term):
  //
  //   1. Foot of perpendicular falls inside [p1, p2] — the body is
  //      "alongside" the wall. Use the wall line's normal (lx, ly):
  //         h     = lx·Pcx + ly·Pcy + c
  //         n     = (lx, ly)
  //         lever = lx·Pcy − ly·Pcx          (lever uses corner)
  //
  //   2. Foot is past p1 — the segment endpoint p1 is the closest
  //      hazard. The CBF becomes a corner-vs-endpoint distance:
  //         h     = ‖Pc − p1‖
  //         n     = (Pc − p1) / ‖Pc − p1‖   (unit vector p1→Pc)
  //         lever = n.x·p1.y − n.y·p1.x      (lever uses endpoint —
  //                                          the moving reference)
  //
  //   3. Foot is past p2 — same as (2) with p2 substituted for p1.
  //
  // h is continuous across region boundaries (perpendicular distance
  // matches endpoint distance at the foot endpoints), so the QP sees
  // a smooth constraint. This is the natural extension of Saradagi's
  // "wall is a half-plane" assumption to finite walls; the same math
  // also covers L-bend inner corners (when L-bend support returns,
  // the inner-corner CP is just an endpoint shared by two walls).
  //
  // ≤-form for the QP:
  //   [-n.x, -n.y, lever] · u  ≤  γ·h

  const double dx = w.p2.x - w.p1.x;
  const double dy = w.p2.y - w.p1.y;
  const double seg_len = std::hypot(dx, dy);
  if (seg_len < 1e-6) {
    return false;  // degenerate segment — skip
  }
  const double tx = dx / seg_len;
  const double ty = dy / seg_len;

  double h_min = std::numeric_limits<double>::infinity();
  int worst_c = -1;
  Eigen::Vector3d worst_grad;

  for (int i = 0; i < 4; ++i) {
    const Point2D & Pc = corners[i];
    const double along = (Pc.x - w.p1.x) * tx + (Pc.y - w.p1.y) * ty;

    double h_i;
    Eigen::Vector3d grad_i;

    if (along >= 0.0 && along <= seg_len) {
      // Region 1: inside segment — line CBF.
      h_i = w.lx * Pc.x + w.ly * Pc.y + w.c;
      const double lever = w.lx * Pc.y - w.ly * Pc.x;
      grad_i = Eigen::Vector3d(-w.lx, -w.ly, lever);
    } else {
      // Regions 2/3: endpoint CBF.
      const Point2D & ref = (along < 0.0) ? w.p1 : w.p2;
      const double rdx = Pc.x - ref.x;
      const double rdy = Pc.y - ref.y;
      const double d = std::hypot(rdx, rdy);
      if (d < 1e-6) {
        // Body corner sits ON the endpoint — pathological.
        // Emit a near-zero h with an arbitrary safe-direction
        // gradient so the QP nudges away rather than NaN-ing.
        h_i = 0.0;
        grad_i = Eigen::Vector3d(-1.0, 0.0, 0.0);
      } else {
        const double n_x = rdx / d;
        const double n_y = rdy / d;
        h_i = d;
        const double lever = n_x * ref.y - n_y * ref.x;
        grad_i = Eigen::Vector3d(-n_x, -n_y, lever);
      }
    }

    if (h_i < h_min) {
      h_min = h_i;
      worst_c = i;
      worst_grad = grad_i;
    }
  }

  if (worst_c < 0) {return false;}

  // Range gate — same as before, but h is now a unified
  // distance-to-wall measure (perpendicular OR endpoint distance).
  if (h_min > params_->wall_consideration_range) {return false;}

  out.grad = worst_grad;
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

  // 2. Inner-corner constraints — gated off in current scope.
  //
  // The previous implementation built a virtual wall along the line
  // connecting the two passage endpoints (a, b) and forced the body
  // corners to stay on the alley-interior side. That is correct for
  // a real L-turn (where two walls converge at a CP and the line A–B
  // bounds the inner corner the body must not sweep through), but it
  // is WRONG for a straight-alley exit / entry, where A–B is an
  // OPENING the robot has to cross. Crossing the line drove h < 0,
  // the QP fought the nominal command, and the robot oscillated at
  // the door zone (see logs at /home/abhinash/nav2_logs/n_l/backward,
  // tick 1099+ for the canonical failure).
  //
  // Current scope is straight alleys + door entry/exit only — there
  // are no real inner corners — so we disable this path entirely.
  // Outer-wall CBFs from the alley side walls still protect against
  // corner clipping. Passage detection itself remains on for logging.
  //
  // Re-enable when L-bends come back into scope, but rewrite first to
  // gate on c.is_intersection (real geometric corner) rather than on
  // any detected passage.
  if (params_->enable_inner_corner_cbf) {
    emitInnerCornerConstraints(snap.passage, res.corners, res.constraints);
  }

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
