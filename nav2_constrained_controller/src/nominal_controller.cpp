// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/nominal_controller.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

#include "nav2_constrained_controller/parameter_handler.hpp"

namespace nav2_constrained_controller
{

NominalController::NominalController(const Parameters * params)
: params_(params)
{
}

geometry_msgs::msg::Twist NominalController::compute(
  const geometry_msgs::msg::Pose & target,
  NominalDebug * dbg) const
{
  const double x_t = target.position.x;
  const double y_t = target.position.y;
  const double r = std::hypot(x_t, y_t);
  const double slowdown = params_->slowdown_radius;

  const double s = (slowdown > 0.0)
    ? std::min(1.0, r / slowdown)
    : 1.0;
  const double denom = std::abs(x_t) + std::abs(y_t) + 1e-6;

  // sign(x_t) makes the SAME law work for forward AND backward.
  // For forward path: x_t > 0 -> +vx. For backward path (where the
  // SmacLattice intermediate poses retangent to point opposite the
  // direction of motion): x_t < 0 -> -vx. No fwd/back branching.
  const double sign_xt = (x_t >= 0.0) ? 1.0 : -1.0;
  const double vx_unclamp = sign_xt * s * params_->v_linear_max *
    (std::abs(x_t) / denom);

  // y_t comes from the path's lateral component in base_link. Path is
  // in odom (Fix 1), so y_t is AMCL-free after setPlan.
  const double vy_unclamp = s * params_->v_lateral_max * (y_t / denom);

  // Heading reference: direction-to-lookahead-point, with a sign-flip
  // for backward motion so the rear faces the lookahead. We do NOT use
  // the lookahead pose's stored orientation here.
  //
  // Why: the stored orientation is set by validateOrientations() to
  // atan2(dy, dx) between consecutive path poses. For a backward-going
  // path that's ≈ ±π, which under angular-distance arithmetic asks the
  // robot to spin around 180° — the controller then saturates wz_path
  // at the angular envelope on every backward tick and bleeds rotation
  // into the blend that the body can't physically execute in narrow
  // alleys. Backtesting against run6 plan 2 confirmed 100% of backward
  // ticks were saturated and 79% had wz_path sign-flipped from what
  // the path actually wanted.
  //
  // Direction-to-lookahead is the standard pure-pursuit signal and is
  // direction-aware: forward motion targets the lookahead in front;
  // backward motion targets the lookahead behind (via the +π rear
  // shift). Sign matches the geometry the robot can actually drive.
  const double alpha = std::atan2(y_t, x_t);
  const double yaw_err = (x_t >= 0.0)
    ? alpha
    : angles::shortest_angular_distance(0.0, alpha + M_PI);

  // Yaw correction is full during traversal and tapers near the goal.
  // We reuse `s = min(1, r/slowdown_radius)` (the linear-speed taper)
  // so the lateral-speed and yaw-correction envelopes share one
  // monotone shape: at goal both are 0, far from goal both are 1.
  //
  // Previous code used `(slowdown - r)/slowdown` — the inverse shape.
  // It killed yaw correction *far* from goal, exactly when we need
  // it most for long-corridor tracking. With slowdown_radius < r,
  // ramp went to 0 and wz_nom = 0 throughout alley traversal, letting
  // un-corrected yaw drift translate into wall clips.
  const double ramp = params_->yaw_correction_ramp ? s : 1.0;
  const double wz_unclamp = params_->k_yaw * ramp * yaw_err;

  // Clamp to actuation envelope. vx is signed: clamp symmetrically.
  const double vx = std::clamp(
    vx_unclamp,
    -params_->v_linear_max,
    params_->v_linear_max);
  const double vy = std::clamp(
    vy_unclamp,
    -params_->v_lateral_max,
    params_->v_lateral_max);
  const double wz = std::clamp(
    wz_unclamp,
    -params_->v_angular_max,
    params_->v_angular_max);

  if (dbg) {
    dbg->r = r;
    dbg->s = s;
    dbg->ramp = ramp;
    dbg->yaw_err = yaw_err;
    dbg->vx_unclamped = vx_unclamp;
    dbg->vy_unclamped = vy_unclamp;
    dbg->wz_unclamped = wz_unclamp;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = wz;
  return cmd;
}

}  // namespace nav2_constrained_controller
