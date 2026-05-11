// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// LateralCentering — LiDAR-based vy override for the nominal command.
//
// Replaces the path-derived vy_nom with a closed-loop centering law
// driven by the closest flanking wall on each side of the robot.
//
// A wall is "flanking" iff:
//   - its tangent direction is roughly parallel to robot's +x
//     (|t.x| >= flanking_cos_tol), AND
//   - its segment x-range overlaps the body x-range
//     [-(L+dl), +dl] in base_link.
//
// Among flanking walls, the closest one on each side (smallest c) is
// taken. The two distances D_L (left, m.y > 0) and D_R (right,
// m.y < 0) feed three regimes:
//
//   BOTH       :  vy = k_lat * (D_L - D_R)
//   LEFT_ONLY  :  vy = k_lat * (D_L - target_half_width)
//   RIGHT_ONLY :  vy = -k_lat * (D_R - target_half_width)
//   NONE       :  caller falls back to path-derived vy_nom
//
// Sign convention: +y is robot's LEFT. If robot is too close to the
// right wall (D_L > D_R), vy > 0 pushes it left, toward the center.
//
// Hysteresis: regime switches require N consecutive ticks of the new
// regime before flipping. Low-pass smooths vy across regime jumps.

#ifndef NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_

#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

struct Parameters;  // fwd decl

enum class CenteringRegime
{
  NONE = 0,
  BOTH = 1,
  LEFT_ONLY = 2,
  RIGHT_ONLY = 3
};

struct CenteringDebug
{
  CenteringRegime regime{CenteringRegime::NONE};
  // Body-aware distance: min over 4 body corners of segment-distance
  // to the closest flanking wall on that side. 0 if no wall on side.
  double D_L{0.0};
  double D_R{0.0};
  bool has_L{false};
  bool has_R{false};
  // Alley-axis misalignment of the robot's +x against the average
  // tangent of the closest flanking walls (radians, signed). 0 means
  // robot heading is aligned with the alley axis.
  double yaw_misalign{0.0};
  double vy_raw{0.0};      // pre-LPF, pre-clamp
  double vy_smoothed{0.0}; // post-LPF, post-clamp (the actual override)
  int n_flanking{0};       // total flanking walls seen this tick
  // True if compute() returned a value that should override vy_nom.
  // False means caller keeps the path-derived vy_nom.
  bool override_active{false};
};

class LateralCentering
{
public:
  explicit LateralCentering(const Parameters * params);

  // Compute the centering vy for this tick.
  //   snap : the LiDAR scene snapshot (walls in base_link).
  // Returns the vy override. If dbg.override_active is false, the
  // caller should retain the existing (path-derived) vy_nom.
  double compute(const SceneSnapshot & snap, CenteringDebug * dbg);

  // Reset state — call on activate() and on each new setPlan() so the
  // low-pass history and regime stickiness do not bleed across plans.
  void reset();

private:
  const Parameters * params_;

  // State carried across ticks.
  CenteringRegime regime_prev_{CenteringRegime::NONE};
  int regime_pending_count_{0};      // consecutive ticks the candidate has held
  CenteringRegime regime_pending_{CenteringRegime::NONE};
  double vy_prev_{0.0};
  bool has_history_{false};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_
