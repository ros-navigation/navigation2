// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// ESDF-based CBF safety filter — perimeter sampling.
//
// The robot body is represented by sample points along its full perimeter,
// not just the 4 corners. Door frames contact the MIDDLE of the long side;
// corner-only sampling produces no constraint there.
//
// For each sample point P_i at (px_i, py_i) in base_link:
//
//   h_i = d_esdf(P_i) - d_safe
//
// CBF condition (must hold to keep h non-decreasing):
//
//   ḣ_i = ∇d(P_i)^T · ṗ_i  ≥  -γ · h_i
//
// At θ=0 (base_link frame) the point velocity is:
//
//   ṗ_i = J_i · u,   J_i = [[1, 0, -py_i],
//                             [0, 1,  px_i]]
//
// Rearranged to QP ≤-form (Au ≤ b):
//
//   [-gx_i,  -gy_i,  gx_i·py_i - gy_i·px_i] · u  ≤  γ · h_i
//
// Sampling layout (5 per long side + 5 per short side = 20 rows):
//   Long sides (y=±db): 5 samples including corners.
//   Short sides (x=±Lext): 5 samples including corners (matches long-side density).
//   Dense short-side sampling catches diagonal door posts that pass between
//   the corners and the single old midpoint.
// All rows fed into a single active-set QP solving for one robot velocity u.

#ifndef NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_

#include <Eigen/Core>
#include <array>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

#include "nav2_constrained_controller/types.hpp"
#include "nav2_constrained_controller/esdf_grid.hpp"
#include "nav2_constrained_controller/qp_solver.hpp"

namespace nav2_constrained_controller
{

struct Parameters;

// High-level controller mode of the retreat overlay. Logged so we can see in
// recordings when the overlay is active and which option it picked. There is
// deliberately no GIVE_UP / failure state — controller-level stuck detection
// and Nav2 BT recovery already handle the "retreat isn't working" case; an
// extra failure state inside the overlay creates deadlock risk (zero output
// evades the stuck detector's commanded_norm threshold).
enum class RetreatState : int
{
  NORMAL = 0,    // path-following, no overlay influence
  RETREAT = 1    // overlay active: blend u_nom toward weighted-gradient retreat
};

struct CbfFilterResult
{
  geometry_msgs::msg::Twist u;          // u* (post-QP)
  std::vector<CbfConstraint> constraints;
  QpResult qp;
  std::array<Point2D, 4> corners;       // body corners in base_link
  double min_h{0.0};                    // smallest h across all constraints
  double solve_time_us{0.0};

  // Retreat overlay diagnostics. picked_candidate is now a binary indicator
  // (1 = retreat direction was synthesized, 0 = no tight samples or NORMAL
  // state). picked_score is the negated worst-sample h (positive = imminent
  // collision).
  RetreatState retreat_state{RetreatState::NORMAL};
  int picked_candidate{0};
  double min_h_react{0.0};              // worst body-sample h pre-QP, used by overlay decision
  double picked_score{0.0};             // -worst_h: positive means inside d_safe buffer
};

class CBFSafetyFilter
{
public:
  explicit CBFSafetyFilter(const Parameters * params);

  // Generate ESDF-based CBF constraints for the 4 body corners and
  // solve the min-deviation QP. Robot is at origin (base_link), θ=0.
  CbfFilterResult filter(
    const EsdfGrid & esdf,
    const geometry_msgs::msg::Twist & u_nom);

  // Compute the four body corners in base_link at pose (xr, yr, θ).
  // Called with (0,0,0) during normal operation; exposed for tests.
  std::array<Point2D, 4> bodyCorners(double xr, double yr, double theta) const;

  // Clear the predictor state AND the retreat overlay state. Call on
  // activate() and setPlan() so neither the previous-tick u* nor the retreat
  // bookkeeping leaks across plans.
  void reset()
  {
    has_last_u_            = false;
    retreat_state_         = RetreatState::NORMAL;
    retreat_ticks_         = 0;
    last_picked_candidate_ = 0;
  }

private:
  const Parameters * params_;

  // Last solved u* — used as the predictor velocity for the next tick's
  // forward-rollout of perimeter samples. Replaces u_nom-as-predictor so the
  // predicted positions reflect the trajectory the robot will actually follow.
  // On the first call (has_last_u_ = false), the filter falls back to u_nom.
  Eigen::Vector3d last_u_star_{Eigen::Vector3d::Zero()};
  bool has_last_u_{false};

  // Retreat overlay state, persistent across ticks. retreat_ticks_ counts
  // consecutive ticks in RETREAT since entry — kept as a diagnostic value
  // only; no transition depends on it (failure handling is the controller +
  // BT's job, not the filter's).
  RetreatState retreat_state_{RetreatState::NORMAL};
  int retreat_ticks_{0};
  int last_picked_candidate_{0};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_
