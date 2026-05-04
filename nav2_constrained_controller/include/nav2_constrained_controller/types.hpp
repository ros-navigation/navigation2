// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_

#include <vector>
#include <Eigen/Core>

namespace nav2_constrained_controller
{

// Single 2D point in base_link.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// A wall line in base_link, in implicit normal form:
//   l_x * x + l_y * y + c = 0,
// with (l_x, l_y) the OUTWARD-pointing unit normal away from the robot
// (i.e. the half-space l_x*x + l_y*y + c >= 0 contains the robot at the
// origin and is the SAFE side; the wall sits on the line itself).
//
// p1, p2 are the two segment endpoints in base_link (used for facing
// tests and for door / passage geometry).
struct Wall
{
  double lx{0.0};
  double ly{0.0};
  double c{0.0};
  Point2D p1;
  Point2D p2;
  // Index back to the source segment in walls[]. Useful for logging
  // which wall a CBF constraint was generated from.
  int id{-1};
};

// Complete Point — geometric corner extracted from adjacent line
// segments. "Real" CPs are intersection points of connected segments;
// "endpoint" CPs are the closer-range endpoint of a disconnected pair
// (per Xiang et al. 2004).
struct CornerPoint
{
  Point2D p;
  bool is_intersection{false};
  int wall_a{-1};
  int wall_b{-1};
};

// Detected passage / doorway / alley mouth.
//   present  : whether a passage was detected this scan
//   a, b     : the two complete points bounding the opening
//   width   : ||a - b||
//   type    : 1 for Type-I (two CPs, parallel walls), 2 for Type-II
//             (one CP plus a perpendicular wall), 0 if not present
struct Passage
{
  bool present{false};
  Point2D a;
  Point2D b;
  double width{0.0};
  int type{0};
};

// One-shot snapshot of everything the LiDAR scene parser produced for
// this control tick.  All consumers (CBF, logger, future visualisers)
// read from one place.
struct SceneSnapshot
{
  std::vector<Wall> walls;
  std::vector<CornerPoint> corners;
  Passage passage;
  // True if a previously visible passage just opened up (one side of
  // the scan is now long range). Reserved for future regime / event
  // logging — not used by control law.
  bool passage_cleared{false};
  // Total number of valid LiDAR returns considered this tick.
  int num_returns{0};
  double stamp_sec{0.0};
};

// CBF half-space inequality of the form
//     a^T u + b >= 0  (equivalently, ∇h · u + γ·α(h) >= 0)
// stored after rearrangement so the QP can directly assemble its own
// canonical form.
struct CbfConstraint
{
  Eigen::Vector3d grad;   // ∇h · g(x)  (since g(x)=I, this is ∇h)
  double rhs{0.0};        // γ · α(h)
  double h{0.0};          // raw barrier value (signed distance margin)
  int corner_id{-1};      // 1..4 for footprint corners, -1 for inner CPs
  int wall_id{-1};        // index into walls[] (or -1 for inner-corner)
  enum Kind { OUTER_WALL, INNER_CORNER } kind{OUTER_WALL};
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__TYPES_HPP_
