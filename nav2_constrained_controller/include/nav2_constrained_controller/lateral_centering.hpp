// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// LateralCentering — LiDAR perception for two purposes:
//
//   1. Wall-relative metrics for NORMAL mode (path-tracking with small
//      additive corrections):
//        D_L, D_R              : body-aware segment-distance to closest
//                                 flanking wall on each side
//        yaw_misalign          : robot +x vs alley axis
//        wall_quality (0..1)   : confidence that the centering signal
//                                 is meaningful right now
//
//   2. Passage-relative metrics for ALIGNMENT mode (passage entry/exit):
//        passage_present_in_motion_direction
//        e_lat_passage         : signed lateral offset of body from
//                                 passage gap center
//        e_yaw_passage         : signed angular error of body heading
//                                 vs passage axis
//        alignment_error       : combined magnitude of the two errors
//                                 (normalized)
//        needs_alignment       : bool — true if the controller should
//                                 enter alignment mode this tick
//
// This class is now a metrics PRODUCER. It returns no command. The
// controller reads the metrics and decides which mode to apply.

#ifndef NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_

#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

struct Parameters;  // fwd decl

struct CenteringDebug
{
  // ---------- Wall-relative (normal-mode) ----------
  // Body-aware segment-distance to closest flanking wall on each side.
  // Min over 4 body corners, using Voronoi-region segment distance
  // (line distance when foot inside segment, endpoint distance otherwise).
  double D_L{0.0};
  double D_R{0.0};
  bool   has_L{false};
  bool   has_R{false};
  // Yaw of robot +x against averaged folded tangents of closest flanking
  // walls (radians, signed). 0 = aligned with alley axis.
  double yaw_misalign{0.0};
  int    n_flanking{0};

  // ---------- Wall quality (0..1) ----------
  // Composite of perception confidence (length, span, count) and context
  // relevance (width consistency, passage penalty). Used by the
  // controller to gate small additive corrections in normal mode.
  double wall_quality{0.0};
  // Individual factor contributions for diagnosis.
  double q_length{1.0};
  double q_span{1.0};
  double q_count{1.0};
  double q_width{1.0};
  double q_passage{1.0};   // 1 normally, 0 when passage near, drops quality

  // ---------- Passage-relative (alignment-mode) ----------
  // True iff a passage is detected AND in the motion direction AND within
  // alignment_passage_range of the body.
  bool   passage_in_motion_direction{false};
  // Distance along motion-direction axis from body origin to the
  // passage centroid (signed, positive = ahead in motion direction).
  double passage_distance{0.0};
  // Signed lateral offset of body origin from passage gap center
  // (measured along the passage's transverse axis).
  double e_lat_passage{0.0};
  // Signed angular error of robot heading vs passage axis.
  double e_yaw_passage{0.0};
  // Combined normalized alignment error in [0, inf):
  //   max(|e_yaw_passage|/yaw_scale, |e_lat_passage|/lat_scale)
  double alignment_error{0.0};
  // True if alignment mode should engage this tick.
  bool   needs_alignment{false};
};

class LateralCentering
{
public:
  explicit LateralCentering(const Parameters * params);

  // Pure metrics computation — no command output.
  //   snap          : LiDAR scene snapshot
  //   path_vx_sign  : sign of the path's commanded vx (motion direction)
  //                   used to filter "ahead vs behind" for passages
  //   dbg           : (out) all derived metrics for the controller to act on
  void compute(
    const SceneSnapshot & snap,
    double path_vx_sign,
    CenteringDebug * dbg);

  // Stateless now (no LPF, no hysteresis). reset() kept for API
  // compatibility with setPlan() but does nothing.
  void reset() {}

private:
  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__LATERAL_CENTERING_HPP_
