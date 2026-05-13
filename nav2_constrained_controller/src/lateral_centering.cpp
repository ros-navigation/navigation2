// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0
//
// LateralCentering — pure metrics producer. See header for full spec.
// Outputs wall-relative metrics (D_L, D_R, yaw_misalign, wall_quality)
// and passage-relative metrics (e_lat_passage, e_yaw_passage,
// alignment_error, needs_alignment). The controller consumes these
// and decides between normal mode (small additive corrections) and
// alignment mode (LiDAR-driven docking-style alignment at passages).

#include "nav2_constrained_controller/lateral_centering.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "nav2_constrained_controller/parameter_handler.hpp"

namespace nav2_constrained_controller
{

LateralCentering::LateralCentering(const Parameters * params)
: params_(params)
{
}

namespace
{

struct WallClass
{
  bool   is_flanking{false};
  bool   is_left{false};
  double d_body{0.0};       // segment-distance min over 4 body corners
  double tx_folded{0.0};    // +x-leaning tangent
  double ty_folded{0.0};
  double seg_len{0.0};
  double seg_xmin{0.0};
  double seg_xmax{0.0};
};

// Segment-distance from a single body corner to a wall.
// Voronoi regions: line distance if foot inside segment, else endpoint
// distance. Mirrors cbf_safety_filter.cpp::emitOuterWallConstraint.
double segmentDistance(const Wall & w, const Point2D & Pc)
{
  const double dx = w.p2.x - w.p1.x;
  const double dy = w.p2.y - w.p1.y;
  const double seg_len = std::hypot(dx, dy);
  if (seg_len < 1e-6) {
    return std::hypot(Pc.x - w.p1.x, Pc.y - w.p1.y);
  }
  const double tx = dx / seg_len;
  const double ty = dy / seg_len;
  const double along = (Pc.x - w.p1.x) * tx + (Pc.y - w.p1.y) * ty;
  if (along >= 0.0 && along <= seg_len) {
    return std::abs(w.lx * Pc.x + w.ly * Pc.y + w.c);
  }
  const Point2D & ref = (along < 0.0) ? w.p1 : w.p2;
  return std::hypot(Pc.x - ref.x, Pc.y - ref.y);
}

WallClass classifyWall(const Wall & w, const Parameters * p)
{
  WallClass wc;
  const double dx = w.p2.x - w.p1.x;
  const double dy = w.p2.y - w.p1.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {return wc;}
  wc.seg_len = len;

  const double tx = dx / len;
  const double ty = dy / len;

  if (std::abs(tx) < p->flanking_cos_tol) {return wc;}

  const double body_xmin = -(p->footprint_length + p->footprint_dl);
  const double body_xmax = p->footprint_dl;
  const double seg_xmin = std::min(w.p1.x, w.p2.x);
  const double seg_xmax = std::max(w.p1.x, w.p2.x);
  wc.seg_xmin = seg_xmin;
  wc.seg_xmax = seg_xmax;
  const double LONG_HORIZON = 1.5;
  if (seg_xmax < body_xmin - LONG_HORIZON ||
      seg_xmin > body_xmax + LONG_HORIZON)
  {
    return wc;
  }

  const double L = p->footprint_length;
  const double dl = p->footprint_dl;
  const double db = p->footprint_db;
  const std::array<Point2D, 4> corners = {{
    {+dl,        +db},   // FL
    {-(L + dl), +db},    // BL
    {-(L + dl), -db},    // BR
    {+dl,        -db}    // FR
  }};

  double d_body = std::numeric_limits<double>::infinity();
  for (const auto & Pc : corners) {
    const double d = segmentDistance(w, Pc);
    if (d < d_body) {d_body = d;}
  }
  if (d_body > p->max_centering_range) {return wc;}

  const double foot_y = -w.c * w.ly;
  wc.is_left = (foot_y > 0.0);
  if (tx >= 0.0) {wc.tx_folded = tx;  wc.ty_folded = ty;}
  else            {wc.tx_folded = -tx; wc.ty_folded = -ty;}
  wc.is_flanking = true;
  wc.d_body = d_body;
  return wc;
}

inline double smoothstep01(double x)
{
  // x ∈ [0, 1] → 0..1 smoothstep. Outside [0,1] clamped.
  if (x <= 0.0) {return 0.0;}
  if (x >= 1.0) {return 1.0;}
  return x * x * (3.0 - 2.0 * x);
}

}  // namespace

void LateralCentering::compute(
  const SceneSnapshot & snap,
  double path_vx_sign,
  CenteringDebug * dbg)
{
  if (dbg) {*dbg = CenteringDebug{};}

  const auto * p = params_;

  // ---------- Step 1: classify flanking walls and pick closest per side ----------
  double D_L = std::numeric_limits<double>::infinity();
  double D_R = std::numeric_limits<double>::infinity();
  bool   has_L = false, has_R = false;
  int    n_flanking = 0;
  double tL_x = 0, tL_y = 0, tR_x = 0, tR_y = 0;
  double len_L = 0, len_R = 0;
  double span_L = 0, span_R = 0;

  const double body_xmin = -(p->footprint_length + p->footprint_dl);
  const double body_xmax = p->footprint_dl;
  const double body_len  = body_xmax - body_xmin;

  for (const auto & w : snap.walls) {
    const WallClass wc = classifyWall(w, p);
    if (!wc.is_flanking) {continue;}
    ++n_flanking;
    const double overlap_lo = std::max(wc.seg_xmin, body_xmin);
    const double overlap_hi = std::min(wc.seg_xmax, body_xmax);
    const double overlap = std::max(0.0, overlap_hi - overlap_lo);
    if (wc.is_left) {
      if (wc.d_body < D_L) {
        D_L = wc.d_body;
        has_L = true;
        tL_x = wc.tx_folded;
        tL_y = wc.ty_folded;
        len_L = wc.seg_len;
        span_L = overlap;
      }
    } else {
      if (wc.d_body < D_R) {
        D_R = wc.d_body;
        has_R = true;
        tR_x = wc.tx_folded;
        tR_y = wc.ty_folded;
        len_R = wc.seg_len;
        span_R = overlap;
      }
    }
  }

  if (dbg) {
    dbg->D_L = has_L ? D_L : 0.0;
    dbg->D_R = has_R ? D_R : 0.0;
    dbg->has_L = has_L;
    dbg->has_R = has_R;
    dbg->n_flanking = n_flanking;
  }

  // ---------- Step 2: yaw misalign vs alley axis ----------
  double axis_x = 0.0, axis_y = 0.0;
  if (has_L && has_R)      {axis_x = 0.5 * (tL_x + tR_x); axis_y = 0.5 * (tL_y + tR_y);}
  else if (has_L)          {axis_x = tL_x; axis_y = tL_y;}
  else if (has_R)          {axis_x = tR_x; axis_y = tR_y;}
  double yaw_misalign = 0.0;
  if (std::hypot(axis_x, axis_y) > 1e-6) {
    yaw_misalign = std::atan2(axis_y, axis_x);
  }
  if (dbg) {dbg->yaw_misalign = yaw_misalign;}

  // ---------- Step 3: wall quality composite ----------
  // (a) length factor: long walls are credible; short ones aren't.
  const double min_len = std::max(p->wall_quality_min_length, 1e-6);
  const double len_pair = std::min(len_L > 0 ? len_L : 0.0,
                                    len_R > 0 ? len_R : 0.0);
  const double q_length = (has_L && has_R)
    ? smoothstep01(len_pair / min_len) : 0.0;

  // (b) span factor: walls should cover the body's x-range.
  const double min_span = p->wall_quality_min_span_ratio * body_len;
  const double span_pair = std::min(span_L, span_R);
  const double q_span = (has_L && has_R)
    ? smoothstep01(span_pair / std::max(min_span, 1e-6)) : 0.0;

  // (c) count factor: both walls visible.
  const double q_count = (has_L && has_R) ? 1.0 : 0.0;

  // (d) width consistency: D_L + D_R + body_width should approximate
  //     expected alley width.
  const double body_width = 2.0 * p->footprint_db;
  const double observed_width = (has_L ? D_L : 0.0) + (has_R ? D_R : 0.0) + body_width;
  const double width_err = std::abs(observed_width - p->wall_quality_expected_width);
  const double q_width = (has_L && has_R)
    ? 1.0 - smoothstep01(width_err / std::max(p->wall_quality_width_tol, 1e-6))
    : 0.0;

  // (e) passage penalty: drops quality near detected passages.
  double q_passage = 1.0;
  if (snap.passage.present) {
    const double mid_x = 0.5 * (snap.passage.a.x + snap.passage.b.x);
    const double mid_y = 0.5 * (snap.passage.a.y + snap.passage.b.y);
    const double dist = std::hypot(mid_x, mid_y);
    q_passage = smoothstep01(dist / std::max(p->passage_penalty_range, 1e-6));
  }

  const double wall_quality = q_length * q_span * q_count * q_width * q_passage;
  if (dbg) {
    dbg->q_length  = q_length;
    dbg->q_span    = q_span;
    dbg->q_count   = q_count;
    dbg->q_width   = q_width;
    dbg->q_passage = q_passage;
    dbg->wall_quality = wall_quality;
  }

  // ---------- Step 4: passage-relative metrics (alignment mode) ----------
  if (!snap.passage.present) {
    // Nothing more to compute. Leave alignment fields at defaults.
    return;
  }

  // Gap center in body frame.
  const double gx = 0.5 * (snap.passage.a.x + snap.passage.b.x);
  const double gy = 0.5 * (snap.passage.a.y + snap.passage.b.y);

  // Passage axis: perpendicular to (B - A), pointing along the corridor.
  // (B - A) is the door-line direction; the corridor axis is perpendicular.
  // Choose the perpendicular direction whose projection onto the motion
  // direction is positive.
  const double bx = snap.passage.b.x - snap.passage.a.x;
  const double by = snap.passage.b.y - snap.passage.a.y;
  const double blen = std::hypot(bx, by);
  if (blen < 1e-6) {return;}
  // Two candidate axis directions perpendicular to (B-A):
  double ax1 = -by / blen, ay1 = bx / blen;
  double ax2 =  by / blen, ay2 = -bx / blen;
  // Pick the one whose x-component matches path_vx_sign (motion direction
  // in body frame). path_vx_sign > 0 means forward; choose +x-leaning axis.
  double ax = ax1, ay = ay1;
  const double sign_target = (path_vx_sign >= 0.0) ? 1.0 : -1.0;
  if (sign_target * ax1 < sign_target * ax2) {ax = ax2; ay = ay2;}

  // Signed distance from body origin to gap center along motion axis.
  // Positive = passage is AHEAD in motion direction.
  const double passage_dist = gx * ax + gy * ay;

  // Lateral offset of body origin from gap center, along the transverse axis.
  // Transverse axis is (B - A) normalized (perpendicular to motion axis).
  const double mx = bx / blen;
  const double my = by / blen;
  const double e_lat = -(gx * mx + gy * my);
  // e_lat > 0 means body is on the +transverse side of gap center, so we
  // negate to interpret as "where the body should move to reach gap" —
  // robot must move BY +e_lat along transverse to reach gap center.

  // Yaw error: difference between body's +x direction and passage axis.
  // Body +x in body frame is (1, 0). Passage axis is (ax, ay).
  const double e_yaw = std::atan2(ay, ax);

  if (dbg) {
    dbg->passage_distance = passage_dist;
    dbg->e_lat_passage = e_lat;
    dbg->e_yaw_passage = e_yaw;
  }

  // Passage in motion direction iff passage_dist > 0 and within range.
  const bool in_motion_dir =
    (passage_dist > -0.05) &&  // small tolerance for "right at the door"
    (passage_dist < p->alignment_passage_range);

  if (!in_motion_dir) {return;}

  if (dbg) {dbg->passage_in_motion_direction = true;}

  // ---------- Step 5: alignment error and needs_alignment trigger ----------
  const double yaw_err_norm = std::abs(e_yaw) /
    std::max(p->alignment_yaw_scale, 1e-6);
  const double lat_err_norm = std::abs(e_lat) /
    std::max(p->alignment_lat_scale, 1e-6);
  const double align_err = std::max(yaw_err_norm, lat_err_norm);

  const bool aligned =
    (std::abs(e_yaw) < p->alignment_yaw_tol) &&
    (std::abs(e_lat) < p->alignment_lat_tol);

  if (dbg) {
    dbg->alignment_error = align_err;
    dbg->needs_alignment = !aligned;
  }
}

}  // namespace nav2_constrained_controller
