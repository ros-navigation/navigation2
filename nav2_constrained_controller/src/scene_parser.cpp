// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/scene_parser.hpp"

#include <algorithm>
#include <cmath>
#include <stack>

#include "nav2_constrained_controller/parameter_handler.hpp"

namespace nav2_constrained_controller
{

SceneParser::SceneParser(const Parameters * params)
: params_(params)
{
}

void SceneParser::fitLineEndpoints(Segment & s)
{
  // Implicit form from two endpoints (a, b):
  //   line: dx*(y - a.y) - dy*(x - a.x) = 0
  //   ->   -dy*x + dx*y + (dy*a.x - dx*a.y) = 0
  //   ->   lx = -dy / norm,  ly = dx / norm,  c = (dy*a.x - dx*a.y)/norm
  //
  // We then orient the normal so robot origin (0,0) is on the +ve
  // side, i.e. lx*0 + ly*0 + c >= 0  <=>  c >= 0. Flip all three
  // coefficients if c < 0.
  const double dx = s.p2.x - s.p1.x;
  const double dy = s.p2.y - s.p1.y;
  const double norm = std::hypot(dx, dy);
  if (norm < 1e-9) {
    s.lx = 1.0;
    s.ly = 0.0;
    s.c = -s.p1.x;
    return;
  }
  s.lx = -dy / norm;
  s.ly = dx / norm;
  s.c = (dy * s.p1.x - dx * s.p1.y) / norm;
  if (s.c < 0.0) {
    s.lx = -s.lx;
    s.ly = -s.ly;
    s.c = -s.c;
  }
}

std::pair<double, size_t> SceneParser::maxPerpDist(
  const std::vector<Point2D> & pts,
  size_t i_begin, size_t i_end,
  const Point2D & a, const Point2D & b)
{
  // Perpendicular distance from p to chord (a,b). Identical for any
  // valid line normalisation.
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double norm = std::hypot(dx, dy);
  double best = 0.0;
  size_t best_idx = i_begin;
  if (norm < 1e-9) {return {0.0, i_begin};}
  for (size_t i = i_begin; i < i_end; ++i) {
    const double num = std::abs(dy * (pts[i].x - a.x) - dx * (pts[i].y - a.y));
    const double d = num / norm;
    if (d > best) {best = d; best_idx = i;}
  }
  return {best, best_idx};
}

std::vector<SceneParser::Segment> SceneParser::segmentSplitAndMerge(
  const std::vector<Point2D> & pts) const
{
  std::vector<Segment> out;
  const size_t n = pts.size();
  if (n < static_cast<size_t>(params_->line_min_points)) {return out;}

  // Iterative split using a work stack of [begin, end) ranges. We also
  // break a range whenever two consecutive points jump farther than
  // segment_connect_dist (radial discontinuity → object boundary).
  std::stack<std::pair<size_t, size_t>> work;
  work.push({0, n});

  while (!work.empty()) {
    auto [lo, hi] = work.top();
    work.pop();
    if (hi - lo < static_cast<size_t>(params_->line_min_points)) {continue;}

    // Detect discontinuity: split at the largest inter-point gap that
    // exceeds segment_connect_dist. This prevents a single segment
    // from straddling two physical surfaces separated by a gap.
    size_t gap_idx = 0;
    double gap_max = 0.0;
    for (size_t i = lo + 1; i < hi; ++i) {
      const double d = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
      if (d > gap_max) {gap_max = d; gap_idx = i;}
    }
    if (gap_max > params_->segment_connect_dist) {
      work.push({lo, gap_idx});
      work.push({gap_idx, hi});
      continue;
    }

    // No discontinuity: try fitting the chord (lo, hi-1) and split if
    // the worst point is farther than threshold.
    const Point2D & a = pts[lo];
    const Point2D & b = pts[hi - 1];
    auto [d_max, idx] = maxPerpDist(pts, lo, hi, a, b);
    if (d_max > params_->line_split_threshold &&
      idx > lo && idx + 1 < hi)
    {
      work.push({lo, idx + 1});
      work.push({idx, hi});
      continue;
    }

    // Otherwise accept this chord as a segment.
    Segment s;
    s.i_begin = lo;
    s.i_end = hi;
    s.p1 = pts[lo];
    s.p2 = pts[hi - 1];
    fitLineEndpoints(s);
    out.push_back(s);
  }

  // Sort by start index so adjacency tests downstream are simple.
  std::sort(
    out.begin(), out.end(),
    [](const Segment & a, const Segment & b) {return a.i_begin < b.i_begin;});

  // Optional merge pass: collinear neighbours with similar normals
  // and contiguous indices.
  std::vector<Segment> merged;
  for (const auto & s : out) {
    if (!merged.empty()) {
      auto & last = merged.back();
      const double cos_n = last.lx * s.lx + last.ly * s.ly;
      const bool collinear = std::abs(cos_n) > params_->parallel_cos_tol &&
        std::abs(last.c - s.c) < params_->line_split_threshold;
      const bool adjacent = (s.i_begin <= last.i_end + 1);
      if (collinear && adjacent) {
        last.i_end = s.i_end;
        last.p2 = s.p2;
        fitLineEndpoints(last);
        continue;
      }
    }
    merged.push_back(s);
  }
  return merged;
}

bool SceneParser::areConnected(
  const Segment & a, const Segment & b) const
{
  // Check the smallest endpoint-endpoint distance across all four
  // combinations.
  const double d11 = std::hypot(a.p1.x - b.p1.x, a.p1.y - b.p1.y);
  const double d12 = std::hypot(a.p1.x - b.p2.x, a.p1.y - b.p2.y);
  const double d21 = std::hypot(a.p2.x - b.p1.x, a.p2.y - b.p1.y);
  const double d22 = std::hypot(a.p2.x - b.p2.x, a.p2.y - b.p2.y);
  const double d = std::min({d11, d12, d21, d22});
  return d <= params_->segment_connect_dist;
}

bool SceneParser::intersectLines(
  const Segment & a, const Segment & b, Point2D & out)
{
  // Solve  [lx_a ly_a] [x]   [-c_a]
  //         [lx_b ly_b] [y] = [-c_b]
  const double det = a.lx * b.ly - a.ly * b.lx;
  if (std::abs(det) < 1e-9) {return false;}
  out.x = (-a.c * b.ly - (-b.c) * a.ly) / det;
  out.y = (a.lx * (-b.c) - b.lx * (-a.c)) / det;
  return true;
}

std::vector<Wall> SceneParser::toWalls(const std::vector<Segment> & segs)
{
  std::vector<Wall> out;
  out.reserve(segs.size());
  for (size_t i = 0; i < segs.size(); ++i) {
    Wall w;
    w.id = static_cast<int>(i);
    w.lx = segs[i].lx;
    w.ly = segs[i].ly;
    w.c = segs[i].c;
    w.p1 = segs[i].p1;
    w.p2 = segs[i].p2;
    out.push_back(w);
  }
  return out;
}

Passage SceneParser::classifyPassage(
  const std::vector<Wall> & walls,
  const std::vector<CornerPoint> & corners) const
{
  Passage out;
  if (corners.empty()) {return out;}

  const double w_min = params_->alley_width_min - params_->alley_width_tol;
  const double w_max = params_->alley_width_max + params_->alley_width_tol;

  // Type-I: pair of CPs whose separation falls in the width band, and
  // whose owning walls have nearly-parallel normals.
  double best_score = -1.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    for (size_t j = i + 1; j < corners.size(); ++j) {
      const auto & ca = corners[i];
      const auto & cb = corners[j];
      const double w =
        std::hypot(ca.p.x - cb.p.x, ca.p.y - cb.p.y);
      if (w < w_min || w > w_max) {continue;}
      // Look up at least one wall on each side; if either is missing
      // (orphan CP), skip the parallelism test and accept on width
      // alone with lower score.
      double cos_n = 1.0;
      const int wa = (ca.wall_a >= 0) ? ca.wall_a : ca.wall_b;
      const int wb = (cb.wall_a >= 0) ? cb.wall_a : cb.wall_b;
      if (wa >= 0 && wb >= 0 &&
        wa < static_cast<int>(walls.size()) &&
        wb < static_cast<int>(walls.size()))
      {
        cos_n = walls[wa].lx * walls[wb].lx + walls[wa].ly * walls[wb].ly;
      }
      if (std::abs(cos_n) < params_->parallel_cos_tol) {continue;}
      // Score: prefer width nearer the alley centre and nearer to the
      // robot.
      const double mid_alley =
        (params_->alley_width_min + params_->alley_width_max) * 0.5;
      const double r = 0.5 * (
        std::hypot(ca.p.x, ca.p.y) +
        std::hypot(cb.p.x, cb.p.y));
      const double score = -std::abs(w - mid_alley) - 0.05 * r;
      if (score > best_score) {
        best_score = score;
        out.present = true;
        out.a = ca.p;
        out.b = cb.p;
        out.width = w;
        out.type = 1;
      }
    }
  }
  if (out.present) {return out;}

  // Type-II: one CP plus a wall whose normal is roughly perpendicular
  // to the line joining the CP and a sample point on the wall, with
  // the perpendicular foot at distance in the alley-width band.
  for (const auto & c : corners) {
    for (const auto & w : walls) {
      // Wall-direction unit vector.
      const double dx = w.p2.x - w.p1.x;
      const double dy = w.p2.y - w.p1.y;
      const double norm = std::hypot(dx, dy);
      if (norm < 1e-6) {continue;}
      const double tx = dx / norm;
      const double ty = dy / norm;
      // Perpendicular distance from CP to the wall line.
      const double dperp = std::abs(w.lx * c.p.x + w.ly * c.p.y + w.c);
      if (dperp < w_min || dperp > w_max) {continue;}
      // The CP -> wall vector should be roughly along the wall normal,
      // i.e. the dot of CP-to-foot-direction with wall tangent is
      // small. Equivalently, the line from CP perpendicular to the
      // wall hits within or near the wall segment.
      const double along = (c.p.x - w.p1.x) * tx + (c.p.y - w.p1.y) * ty;
      if (along < -0.1 * norm || along > 1.1 * norm) {continue;}
      // The foot of perpendicular.
      const Point2D foot{
        w.p1.x + along * tx,
        w.p1.y + along * ty};
      const double r = 0.5 * (
        std::hypot(c.p.x, c.p.y) +
        std::hypot(foot.x, foot.y));
      const double mid_alley =
        (params_->alley_width_min + params_->alley_width_max) * 0.5;
      const double score = -std::abs(dperp - mid_alley) - 0.05 * r;
      if (score > best_score) {
        best_score = score;
        out.present = true;
        out.a = c.p;
        out.b = foot;
        out.width = dperp;
        out.type = 2;
      }
    }
  }
  return out;
}

SceneSnapshot SceneParser::parse(
  const std::vector<Point2D> & points_base,
  double stamp_sec) const
{
  SceneSnapshot snap;
  snap.stamp_sec = stamp_sec;
  snap.num_returns = static_cast<int>(points_base.size());

  if (points_base.empty()) {return snap;}

  // 1. Line segmentation.
  const auto segs = segmentSplitAndMerge(points_base);
  snap.walls = toWalls(segs);

  // 2. Complete Points: walk adjacent segments in scan order. Two
  // adjacent segments either share an endpoint (connected → real
  // intersection CP) or are disconnected (gap → endpoint with the
  // SHORTER range to the LiDAR origin is the CP, the further is
  // discarded as an occlusion artifact).
  for (size_t i = 0; i + 1 < segs.size(); ++i) {
    const auto & a = segs[i];
    const auto & b = segs[i + 1];
    if (areConnected(a, b)) {
      Point2D p;
      if (intersectLines(a, b, p)) {
        CornerPoint cp;
        cp.p = p;
        cp.is_intersection = true;
        cp.wall_a = static_cast<int>(i);
        cp.wall_b = static_cast<int>(i + 1);
        snap.corners.push_back(cp);
      }
    } else {
      // Disconnected: pick the endpoint of either segment with shorter
      // range. (In the scan, the relevant endpoints are a.p2 and
      // b.p1 — the ones nearest each other in scan index.)
      const Point2D ea = a.p2;
      const Point2D eb = b.p1;
      const double ra = std::hypot(ea.x, ea.y);
      const double rb = std::hypot(eb.x, eb.y);
      CornerPoint cp;
      cp.is_intersection = false;
      if (ra <= rb) {
        cp.p = ea;
        cp.wall_a = static_cast<int>(i);
        cp.wall_b = -1;
      } else {
        cp.p = eb;
        cp.wall_a = -1;
        cp.wall_b = static_cast<int>(i + 1);
      }
      snap.corners.push_back(cp);
    }
  }

  // 3. Passage classifier.
  snap.passage = classifyPassage(snap.walls, snap.corners);

  return snap;
}

}  // namespace nav2_constrained_controller
