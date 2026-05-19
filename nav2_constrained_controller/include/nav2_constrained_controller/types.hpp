// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_

#include <Eigen/Core>

namespace nav2_constrained_controller
{

// Single 2D point in base_link.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// CBF half-space inequality stored in QP ≤-form:
//   grad^T · u  ≤  rhs
// where grad = -∇h·J (negated so the QP sees Au ≤ b)
// and rhs = γ·h (the barrier right-hand side).
struct CbfConstraint
{
  Eigen::Vector3d grad;  // row of A in Au ≤ b
  double rhs{0.0};       // γ·h
  double h{0.0};         // raw barrier value (distance margin)
  int corner_id{-1};     // 1..4 for footprint corners

  // Debug fields (populated by the filter, not used by the QP).
  double Au{0.0};        // A·u_nom — how much nominal velocity activates this constraint
  double margin{0.0};    // rhs - Au: positive=satisfied, negative=violated before QP
  double gx{0.0};        // ESDF gradient x at the sample/predicted position
  double gy{0.0};        // ESDF gradient y
  double sample_x{0.0};  // position where ESDF was queried (current or predicted)
  double sample_y{0.0};
  int    predict_step{0}; // 0=reactive, 1=step1, 2=step2, ...
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_
