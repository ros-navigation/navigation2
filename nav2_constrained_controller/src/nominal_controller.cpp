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

  // Heading reference: the retangented lookahead yaw. Robot yaw is 0
  // in base_link by definition.
  const double yaw_t = tf2::getYaw(target.orientation);
  const double yaw_err = angles::shortest_angular_distance(0.0, yaw_t);

  // ramp = small near goal so we don't fight a far-end yaw the
  // lookahead picker can't actually steer toward yet.
  const double ramp = params_->yaw_correction_ramp
    ? std::clamp((slowdown - r) / std::max(slowdown, 1e-6), 0.0, 1.0)
    : 1.0;
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
