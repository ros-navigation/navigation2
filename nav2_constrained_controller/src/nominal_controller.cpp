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
  const geometry_msgs::msg::Pose & closest,
  const geometry_msgs::msg::Pose & lookahead,
  double dist_to_goal,
  NominalDebug * dbg) const
{
  // ----- direction and speed taper -----
  const double x_t = lookahead.position.x;
  const bool forward = (x_t >= 0.0);

  // Speed taper: slow to zero within slowdown_radius of the goal.
  // Fall back to lookahead distance when dist_to_goal is unavailable.
  const double r = std::hypot(x_t, lookahead.position.y);
  const double taper_dist =
    (dist_to_goal < std::numeric_limits<double>::infinity() / 2.0)
    ? dist_to_goal : r;
  const double s = std::min(
    1.0, taper_dist / std::max(1e-6, params_->slowdown_radius));

  // ----- vx: forward/backward along path -----
  const double vx = (forward ? 1.0 : -1.0) * params_->v_linear_max * s;

  // ----- vy: cross-track correction -----
  // closest.position.y is the signed lateral deviation of the nearest path
  // point from the robot's x-axis in base_link.
  // +y → path is to the left → robot moves left (vy > 0) to close the gap.
  const double cte_y = closest.position.y;
  const double vy = params_->k_lat * cte_y * s;

  // ----- wz: heading correction -----
  // Use the path tangent at the LOOKAHEAD pose (not the closest point).
  // The closest-point tangent matches the path directly under the robot —
  // safe on straight segments but blind to upcoming curvature. Using the
  // lookahead pose's tangent makes heading_err reflect where the path is
  // heading ~motion_target_dist ahead, so the rotation controller starts
  // turning into a curve before the curve reaches the robot body. This
  // prevents Stanley from saturating wz only AFTER entering the bend
  // (the run-7 alley-turn overshoot mode).
  //
  // For backward motion add π: validateOrientations() stores atan2(dy,dx)
  // which points "forward along the path" (= backward for the robot rear).
  // Adding π converts it to the rear-facing reference frame.
  const double path_yaw = tf2::getYaw(lookahead.orientation);
  const double heading_err = forward
    ? path_yaw
    : angles::shortest_angular_distance(0.0, path_yaw + M_PI);
  const double wz = params_->k_yaw * heading_err;

  if (dbg) {
    dbg->r           = r;
    dbg->s           = s;
    dbg->ramp        = s;  // kept for log compat; always equals s
    dbg->cte_y       = cte_y;
    dbg->heading_err = heading_err;
    dbg->is_forward  = forward;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = std::clamp(vx,  -params_->v_linear_max,  params_->v_linear_max);
  cmd.linear.y  = std::clamp(vy,  -params_->v_lateral_max, params_->v_lateral_max);
  cmd.angular.z = std::clamp(wz,  -params_->v_angular_max, params_->v_angular_max);
  return cmd;
}

}  // namespace nav2_constrained_controller
