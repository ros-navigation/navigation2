// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// NominalController — sign-symmetric decoupled P controller.
//
// Single law, no forward/backward branching, no curvature math, no
// initial-rotation. Drives the robot toward a lookahead pose given in
// base_link.
//
//     vx_nom = sign(x_t) * s * v_lin_max  * (|x_t| / denom)
//     vy_nom =             s * v_lat_max  * (y_t  / denom)
//     wz_nom = k_yaw * ramp * shortest_angular_distance(yaw_r, yaw_t)
//
// where
//     r     = sqrt(x_t^2 + y_t^2)
//     denom = |x_t| + |y_t| + 1e-6
//     s     = min(1, r / slowdown_radius)        — distance taper
//     ramp  = clamp((slowdown_radius - r) / slowdown_radius, 0, 1)
//                                                — heading taper near goal
//     yaw_r = 0  (robot frame is at base_link origin, looking +x)
//     yaw_t = retangented yaw of the lookahead pose
//
// Lookahead is in base_link, so x_t, y_t already encode "where the
// path wants me to go relative to my current heading". Lateral target
// y_t comes straight from the path — no LiDAR D_L/D_R wall balancing
// (per the design lock-in: path is in odom via Fix 1, CBF handles
// wall safety, path-derived vy is sufficient).

#ifndef NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_constrained_controller
{

struct Parameters;  // fwd decl

struct NominalDebug
{
  double r{0.0};
  double s{1.0};
  double ramp{1.0};
  double yaw_err{0.0};
  // Pre-clamp values so we can detect when the envelope is biting.
  double vx_unclamped{0.0};
  double vy_unclamped{0.0};
  double wz_unclamped{0.0};
};

class NominalController
{
public:
  explicit NominalController(const Parameters * params);

  // target  : lookahead pose in base_link (yaw is retangented path
  //           tangent)
  // returns : (vx, vy, wz) command, clamped to actuation envelope
  geometry_msgs::msg::Twist compute(
    const geometry_msgs::msg::Pose & target,
    NominalDebug * dbg = nullptr) const;

private:
  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_
