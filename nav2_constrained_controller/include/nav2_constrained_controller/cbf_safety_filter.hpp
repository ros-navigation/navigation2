// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// CBF safety filter (Saradagi et al., 2024 ECC, body-aware navigation
// for asymmetric holonomic robots).
//
// Robot footprint is approximated as a rectangle (L + 2*dl) x (2*db)
// expressed in base_link. The four corners P¹..P⁴ are computed from
// the robot pose (xr, yr, θr) per Eq. 1 of the paper.
//
//     P¹ = (xr + dl·cosθ - db·sinθ,   yr + dl·sinθ + db·cosθ)
//     P² = (xr - L·cosθ - dl·cosθ - db·sinθ,
//           yr - L·sinθ - dl·sinθ + db·cosθ)
//     P³ = (xr - L·cosθ - dl·cosθ + db·sinθ,
//           yr - L·sinθ - dl·sinθ - db·cosθ)
//     P⁴ = (xr + dl·cosθ + db·sinθ,   yr + dl·sinθ - db·cosθ)
//
// Wall constraint (outer-wall): for each wall L_w (lx, ly, c) in the
// scene snapshot, choose the corner most likely to violate (highest
// L_w(P^c)) and emit
//
//     h(x) = -L_w(P^c(x)) ≥ 0      (safe side of wall)
//
// In our base_link convention, the robot is at the origin, walls are
// stored with normal pointing AWAY from the robot (so L_w(0,0)=c≥0
// means robot is on the safe side). For a corner at offset (Px, Py)
// in base_link the value is L_w(Px, Py) = lx·Px + ly·Py + c, which is
// negative if the corner has crossed the wall. The CBF condition
// rewrites to:
//
//     ∇h · u + γ · h ≥ 0
//
// With holonomic kinematics (ẋr=vx, ẏr=vy, θ̇r=ωz) and corners
// computed in base_link, the time derivative of L_w(P^c) along the
// commanded velocity is:
//
//     d/dt L_w(P^c) = lx · dPx/dt + ly · dPy/dt
//
// where dPx/dt and dPy/dt are the linearised dependence of the corner
// position on (vx, vy, ωz). For corners in BASE_LINK (the robot's own
// frame, attached to the body), Px and Py are CONSTANT — the rate of
// change of the wall coefficients seen by a corner is what we need.
//
// We work consistently in BASE_LINK: the wall (lx, ly, c) shifts as
// the body moves. A unit translation +vx of the body shifts the wall
// by -vx along the body x-axis (the wall's c-coefficient changes by
// +lx·vx because the wall moves in -x relative to the body). A unit
// rotation +ωz rotates the wall normal by +ωz. The barrier h = -L_w(Pc)
// then evolves as
//
//     dh/dt = - d/dt [lx·Pcx + ly·Pcy + c]
//           = - [(lx·vx + ly·vy) + ωz · (lx·Pcy - ly·Pcx)]
//
// (the rotation term comes from differentiating the rotated normal
// applied to the constant body-frame corner; algebra in the design
// doc).
//
// Then the CBF condition ∇h·u + γ·h ≥ 0 becomes
//
//     -lx·vx - ly·vy - (lx·Pcy - ly·Pcx)·ωz + γ·h ≥ 0
//
// which we rearrange into the standard ≤-form for the QP:
//
//     [lx, ly, (lx·Pcy - ly·Pcx)] · u  ≤  γ·h
//
// (i.e. row of A, single entry of b).
//
// Inner-corner CBFs (door geometry, h5/h6 in Saradagi) are constructed
// the same way using a virtual wall passing through the inner corner
// CP and an inner-perimeter reference point. We synthesise this
// virtual wall from the detected passage CPs.

#ifndef NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_

#include <Eigen/Core>
#include <array>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

#include "nav2_constrained_controller/types.hpp"
#include "nav2_constrained_controller/qp_solver.hpp"

namespace nav2_constrained_controller
{

struct Parameters;  // fwd decl

struct CbfFilterResult
{
  geometry_msgs::msg::Twist u;     // u* (post-QP)
  std::vector<CbfConstraint> constraints;
  QpResult qp;
  std::array<Point2D, 4> corners;  // body corners in base_link
  // Smallest h across all generated constraints (closest-corner-to-
  // wall margin). Negative means we entered the unsafe set — the QP
  // will still try to minimise but the solution may be infeasible.
  double min_h{0.0};
  double solve_time_us{0.0};
};

class CBFSafetyFilter
{
public:
  explicit CBFSafetyFilter(const Parameters * params);

  // Generate constraints from the scene snapshot and minimally deviate
  // u_nom to keep all four corners on the safe side. Robot pose is
  // taken as origin (we work in base_link); the corners therefore only
  // depend on θ_r = 0 in base_link — the corners are constants of the
  // body geometry. The wall normals already live in base_link.
  CbfFilterResult filter(
    const SceneSnapshot & snap,
    const geometry_msgs::msg::Twist & u_nom);

  // Compute the four body corners in BASE_LINK at robot pose
  // (xr, yr, θr) per Saradagi Eq. 1. We almost always call this with
  // (0, 0, 0) since we operate in base_link, but the helper is
  // exposed for unit tests.
  std::array<Point2D, 4> bodyCorners(double xr, double yr, double theta) const;

private:
  // For a single wall, pick the corner most at risk (the one with the
  // largest L_w(corner) — i.e. closest to or past the wall) and emit a
  // single CBF row.
  bool emitOuterWallConstraint(
    const Wall & w, int wall_idx,
    const std::array<Point2D, 4> & corners,
    CbfConstraint & out) const;

  // Build a virtual inner-corner wall from the detected passage
  // (door corner CP) and emit a CBF constraint per facing corner. The
  // line passes through the CP and the nearer of the two passage
  // endpoints; its normal points away from the robot.
  void emitInnerCornerConstraints(
    const Passage & p,
    const std::array<Point2D, 4> & corners,
    std::vector<CbfConstraint> & out) const;

  // Evaluate L_w at a body corner.
  static double evalWall(const Wall & w, const Point2D & p);

  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_
