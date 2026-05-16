// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// ESDF-based CBF safety filter.
//
// For each of the 4 body corners P_i at (px_i, py_i) in base_link the
// barrier function is:
//
//   h_i = d_esdf(P_i) - d_safe
//
// where d_esdf is the Euclidean distance to the nearest obstacle from
// the local ESDF grid. The CBF condition requires:
//
//   ḣ_i = ∇d(P_i)^T · ṗ_i  ≥  -γ · h_i
//
// At θ=0 (base_link frame) the corner velocity is:
//
//   ṗ_i = J_i · u,   J_i = [[1, 0, -py_i],
//                             [0, 1,  px_i]]
//
// Substituting and rearranging to QP ≤-form (Au ≤ b):
//
//   [-gx_i,  -gy_i,  gx_i·py_i - gy_i·px_i] · u  ≤  γ · h_i
//
// Four corners → 4 rows fed into the same active-set QP as before.
// No wall segmentation, no geometry classification, no regime switching.

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

struct CbfFilterResult
{
  geometry_msgs::msg::Twist u;          // u* (post-QP)
  std::vector<CbfConstraint> constraints;
  QpResult qp;
  std::array<Point2D, 4> corners;       // body corners in base_link
  double min_h{0.0};                    // smallest h across all constraints
  double solve_time_us{0.0};
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

private:
  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__CBF_SAFETY_FILTER_HPP_
