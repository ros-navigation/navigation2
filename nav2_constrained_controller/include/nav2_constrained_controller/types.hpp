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
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_
