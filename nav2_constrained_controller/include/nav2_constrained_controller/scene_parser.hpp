// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// SceneParser — turn a single LaserScan into geometric primitives in
// base_link:
//   walls[]   : line segments fitted to LiDAR returns (split-and-merge).
//   corners[] : "Complete Points" per Xiang et al. 2004:
//                 - intersection of two adjacent connected segments
//                 - or, for two adjacent disconnected segments, the
//                   endpoint with the SHORTER range (closer one — the
//                   farther endpoint is occlusion artifact).
//   passage   : Type-I (two CPs, parallel walls, alley-width-band gap)
//               or Type-II (one CP + perpendicular wall) door / mouth.
//
// We do NOT use this parser to source vy_nom (the design dropped that).
// All output is consumed by the CBF safety filter and by the logger.
//
// Inputs are expected in base_link (controller transforms scan ->
// base_link before calling). All output points / line params are in
// base_link.

#ifndef NAV2_CONSTRAINED_CONTROLLER__SCENE_PARSER_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__SCENE_PARSER_HPP_

#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

struct Parameters;  // fwd decl

class SceneParser
{
public:
  explicit SceneParser(const Parameters * params);

  // Convert LiDAR returns into the scene snapshot used by the rest of
  // the controller.
  //
  // points_base : LiDAR returns transformed into base_link, in the
  //   ORIGINAL angular order of the scan. Out-of-range / NaN returns
  //   are caller-filtered before being passed in. Empty vector is
  //   tolerated and produces an empty snapshot.
  // stamp_sec   : timestamp for the snapshot (passed through to log).
  SceneSnapshot parse(
    const std::vector<Point2D> & points_base,
    double stamp_sec) const;

private:
  // Iterative split-and-merge segmentation. Returns a list of inclusive
  // index ranges [i_begin, i_end) into the input points; each range is
  // a fitted segment.
  struct Segment
  {
    size_t i_begin{0};   // inclusive
    size_t i_end{0};     // exclusive
    Point2D p1;          // first point in segment (== points[i_begin])
    Point2D p2;          // last  point in segment (== points[i_end-1])
    // Implicit normal-form of the fitted line:
    //   lx*x + ly*y + c = 0
    // with (lx, ly) the unit normal pointing AWAY from the robot
    // origin (i.e., the half-space lx*x + ly*y + c >= 0 contains the
    // origin -> robot side is safe side).
    double lx{0.0};
    double ly{0.0};
    double c{0.0};
  };

  // Recursively split a contiguous range using max perpendicular
  // distance to the chord; merge collinear adjacents in a final pass.
  std::vector<Segment> segmentSplitAndMerge(
    const std::vector<Point2D> & pts) const;

  // Compute the line normal-form from two endpoints. The normal is
  // oriented so that the robot origin (0,0) lies on the lx*x+ly*y+c>=0
  // side of the line. If the line passes exactly through the origin
  // (degenerate; shouldn't happen for real walls) we fall back to a
  // canonical orientation.
  static void fitLineEndpoints(Segment & s);

  // Total max perpendicular distance from points[i_begin..i_end) to
  // the chord (p1, p2). Returns (max_dist, idx_of_max).
  static std::pair<double, size_t> maxPerpDist(
    const std::vector<Point2D> & pts,
    size_t i_begin, size_t i_end,
    const Point2D & a, const Point2D & b);

  // Adjacency rule: two segments are connected if their nearest
  // endpoints are within params_->segment_connect_dist; otherwise
  // disconnected.
  bool areConnected(const Segment & a, const Segment & b) const;

  // Solve the 2x2 line intersection for two normal-form lines. Returns
  // false if lines are parallel (nearly collinear normals).
  static bool intersectLines(
    const Segment & a, const Segment & b, Point2D & out);

  // Emit Wall records (the public output type) from internal Segments.
  static std::vector<Wall> toWalls(const std::vector<Segment> & segs);

  // Run the Type-I / Type-II classifier over the corner list.
  Passage classifyPassage(
    const std::vector<Wall> & walls,
    const std::vector<CornerPoint> & corners) const;

  const Parameters * params_;
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__SCENE_PARSER_HPP_
