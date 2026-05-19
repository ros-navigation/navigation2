// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// NominalController — Stanley-inspired path follower for omnidirectional AMR.
//
// All three DoFs are commanded simultaneously — no mode switching.
// The CBF safety filter downstream handles obstacle safety for all outputs.
//
// Control laws (robot at base_link origin, θ=0):
//
//   closest  = local_plan.poses[0]  — closest path point in base_link
//   lookahead = motion target        — first pose at ≥ motion_target_dist
//
//   s        = clamp(taper_dist / slowdown_radius, 0, 1)   — speed taper
//   taper_dist = dist_to_goal if available, else ||lookahead||
//
//   vx  = sign(lookahead.x) · v_linear_max · s
//   vy  = k_lat · cte · s           where cte = closest.position.y
//   wz  = k_yaw · heading_err       where heading_err = tf2::getYaw(closest.orientation)
//                                   (sign-flipped for backward motion)
//
// Heading reference is the path tangent at the CLOSEST point, not the lookahead
// bearing. The closest point moves monotonically along the path, so heading_err
// never oscillates when the robot spins or the path end is reached.

#ifndef NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_constrained_controller
{

struct Parameters;

struct NominalDebug
{
  double r{0.0};           // distance to lookahead (metres)
  double s{1.0};           // speed taper [0,1]
  double ramp{1.0};        // = s (kept for log-file compatibility)
  double cte_y{0.0};       // cross-track error: closest.position.y (metres)
  double heading_err{0.0}; // heading error at closest point (radians)
  bool   is_forward{true}; // true = forward motion (lookahead.x ≥ 0)
};

class NominalController
{
public:
  explicit NominalController(const Parameters * params);

  // closest  : local_plan.poses[0].pose in base_link (closest path point)
  // lookahead: motion target pose in base_link (for direction and taper)
  // dist_to_goal: live distance to goal (inf if TF unavailable — falls back to ||lookahead||)
  geometry_msgs::msg::Twist compute(
    const geometry_msgs::msg::Pose & closest,
    const geometry_msgs::msg::Pose & lookahead,
    double dist_to_goal,
    NominalDebug * dbg = nullptr) const;

private:
  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__NOMINAL_CONTROLLER_HPP_
