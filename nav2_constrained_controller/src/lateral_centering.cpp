// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

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

void LateralCentering::reset()
{
  regime_prev_ = CenteringRegime::NONE;
  regime_pending_ = CenteringRegime::NONE;
  regime_pending_count_ = 0;
  vy_prev_ = 0.0;
  has_history_ = false;
}

namespace
{

// Per-wall classification result.
struct WallClass
{
  bool is_flanking{false};
  bool is_left{false};   // valid only if is_flanking
  double d_body{0.0};    // body-aware: min over 4 corners of seg-dist
  // Wall tangent folded into "+x-leaning" direction (tx >= 0). Used
  // for alley-axis estimation. Valid only if is_flanking.
  double tx_folded{0.0};
  double ty_folded{0.0};
};

// Perpendicular distance from a body corner to a wall LINE (not
// segment). We use line distance, not segment-distance, for the
// centering signal because alley walls are physically continuous —
// the LiDAR resolves only a forward fragment when the body sits
// behind the visible segment, but the wall extends through the
// body's x-range and that perpendicular distance is the actual
// clearance we care about.
//
// (Segment-distance is still the right answer for the CBF's
// safety-margin h, where assuming a wall exists where no segment is
// visible would be wrong. Two different questions, two different
// distances.)
double lineDistance(const Wall & w, const Point2D & Pc)
{
  return std::abs(w.lx * Pc.x + w.ly * Pc.y + w.c);
}

WallClass classifyWall(const Wall & w, const Parameters * p)
{
  WallClass wc;

  const double dx = w.p2.x - w.p1.x;
  const double dy = w.p2.y - w.p1.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {return wc;}

  const double tx = dx / len;
  const double ty = dy / len;

  // Flanking test: tangent's x-component dominates (wall runs along
  // the robot's forward axis). Sign of tx doesn't matter — the wall
  // is the same physical surface whether traversed +x or -x.
  if (std::abs(tx) < p->flanking_cos_tol) {return wc;}

  // Longitudinal proximity gate. We use the wall's LINE for the
  // centering distance (it extrapolates beyond the visible segment),
  // so we must guard against a distant unrelated wall projecting
  // across the body. Require the segment to be within
  // LONG_HORIZON of the body's x-range. Inside the alley the visible
  // fragment is usually 0.3–0.5 m ahead of the body; 1.5 m gives
  // plenty of margin without inviting far walls.
  const double body_xmin = -(p->footprint_length + p->footprint_dl);
  const double body_xmax = p->footprint_dl;
  const double seg_xmin = std::min(w.p1.x, w.p2.x);
  const double seg_xmax = std::max(w.p1.x, w.p2.x);
  const double LONG_HORIZON = 1.5;
  if (seg_xmax < body_xmin - LONG_HORIZON ||
      seg_xmin > body_xmax + LONG_HORIZON)
  {
    return wc;
  }

  // Body corners in base_link, at theta=0 (we always evaluate at
  // robot frame, robot origin).
  const double L = p->footprint_length;
  const double dl = p->footprint_dl;
  const double db = p->footprint_db;
  const std::array<Point2D, 4> corners = {{
    {+dl, +db},          // front-left
    {-(L + dl), +db},    // back-left
    {-(L + dl), -db},    // back-right
    {+dl, -db}           // front-right
  }};

  // Body-aware distance: MIN over the 4 corners of the perpendicular
  // distance to the wall LINE. Min naturally picks the corner closest
  // to the wall, which is the right answer regardless of where the
  // visible segment fragment lies along the wall.
  double d_body = std::numeric_limits<double>::infinity();
  (void)tx; (void)ty; (void)len;  // not needed for line distance
  for (const auto & Pc : corners) {
    const double d = lineDistance(w, Pc);
    if (d < d_body) {d_body = d;}
  }

  if (d_body > p->max_centering_range) {return wc;}

  // Side label from the wall's perpendicular foot relative to origin.
  // Foot of perpendicular from (0,0) onto the wall line is
  //   foot = -c * (lx, ly)
  // (since (lx, ly) is the unit normal pointing AWAY from the robot
  // and c >= 0 by parser convention).
  const double foot_y = -w.c * w.ly;
  wc.is_left = (foot_y > 0.0);

  // Fold tangent into "+x-leaning" direction so all flanking walls
  // contribute coherently to the alley-axis average.
  if (tx >= 0.0) {
    wc.tx_folded = tx;
    wc.ty_folded = ty;
  } else {
    wc.tx_folded = -tx;
    wc.ty_folded = -ty;
  }

  wc.is_flanking = true;
  wc.d_body = d_body;
  return wc;
}

}  // namespace

double LateralCentering::compute(
  const SceneSnapshot & snap, CenteringDebug * dbg)
{
  if (dbg) {*dbg = CenteringDebug{};}

  if (!params_->enable_lateral_centering) {
    if (dbg) {dbg->override_active = false;}
    return 0.0;
  }

  // Per-side aggregates over all flanking walls in 360°.
  double D_L = std::numeric_limits<double>::infinity();
  double D_R = std::numeric_limits<double>::infinity();
  bool has_L = false;
  bool has_R = false;
  int n_flanking = 0;
  // Tangent of the closest flanking wall on each side — used for
  // alley-axis (yaw_misalign) estimation only.
  double tL_x = 0.0, tL_y = 0.0;
  double tR_x = 0.0, tR_y = 0.0;

  for (const auto & w : snap.walls) {
    const WallClass wc = classifyWall(w, params_);
    if (!wc.is_flanking) {continue;}
    ++n_flanking;
    if (wc.is_left) {
      if (wc.d_body < D_L) {
        D_L = wc.d_body;
        has_L = true;
        tL_x = wc.tx_folded;
        tL_y = wc.ty_folded;
      }
    } else {
      if (wc.d_body < D_R) {
        D_R = wc.d_body;
        has_R = true;
        tR_x = wc.tx_folded;
        tR_y = wc.ty_folded;
      }
    }
  }

  // Alley-axis estimate: average tangent of the two closest flanking
  // walls (one per side). Falls back to whichever side is visible.
  // yaw_misalign = angle of (alley_axis) against robot +x.
  double axis_x = 0.0, axis_y = 0.0;
  if (has_L && has_R) {
    axis_x = 0.5 * (tL_x + tR_x);
    axis_y = 0.5 * (tL_y + tR_y);
  } else if (has_L) {
    axis_x = tL_x;
    axis_y = tL_y;
  } else if (has_R) {
    axis_x = tR_x;
    axis_y = tR_y;
  }
  double yaw_misalign = 0.0;
  if (std::hypot(axis_x, axis_y) > 1e-6) {
    yaw_misalign = std::atan2(axis_y, axis_x);
  }

  if (dbg) {
    dbg->D_L = has_L ? D_L : 0.0;
    dbg->D_R = has_R ? D_R : 0.0;
    dbg->has_L = has_L;
    dbg->has_R = has_R;
    dbg->n_flanking = n_flanking;
    dbg->yaw_misalign = yaw_misalign;
  }

  // Regime: only BOTH activates. require_both_walls disables single-
  // side modes (a stray wall in free space must not hijack vy).
  CenteringRegime candidate;
  if (has_L && has_R) {
    candidate = CenteringRegime::BOTH;
  } else if (!params_->require_both_walls && has_L) {
    candidate = CenteringRegime::LEFT_ONLY;
  } else if (!params_->require_both_walls && has_R) {
    candidate = CenteringRegime::RIGHT_ONLY;
  } else {
    candidate = CenteringRegime::NONE;
  }

  // Hysteresis: candidate must hold N consecutive ticks to switch.
  CenteringRegime active = regime_prev_;
  if (candidate == regime_prev_) {
    regime_pending_ = candidate;
    regime_pending_count_ = 0;
  } else {
    if (candidate == regime_pending_) {
      ++regime_pending_count_;
    } else {
      regime_pending_ = candidate;
      regime_pending_count_ = 1;
    }
    if (regime_pending_count_ >= params_->regime_switch_hyst_ticks) {
      active = candidate;
      regime_prev_ = candidate;
      regime_pending_count_ = 0;
    }
  }

  if (dbg) {dbg->regime = active;}

  if (active == CenteringRegime::NONE) {
    if (dbg) {dbg->override_active = false;}
    has_history_ = false;
    vy_prev_ = 0.0;
    return 0.0;
  }

  // Centering law over body-aware clearances.
  double vy_raw = 0.0;
  const double k = params_->k_lat;
  const double t = params_->target_half_width;

  switch (active) {
    case CenteringRegime::BOTH:
      if (has_L && has_R) {
        // Balance body clearance on both sides. Positive imbalance
        // (further from left, closer to right) → +vy (move left).
        vy_raw = k * (D_L - D_R);
      } else if (has_L) {
        vy_raw = k * (D_L - t);
      } else if (has_R) {
        vy_raw = -k * (D_R - t);
      }
      break;
    case CenteringRegime::LEFT_ONLY:
      if (has_L) {vy_raw = k * (D_L - t);}
      break;
    case CenteringRegime::RIGHT_ONLY:
      if (has_R) {vy_raw = -k * (D_R - t);}
      break;
    default:
      break;
  }

  // Low-pass + clamp.
  const double alpha = std::clamp(params_->vy_centering_lpf_alpha, 0.0, 1.0);
  double vy_smoothed = has_history_
    ? alpha * vy_raw + (1.0 - alpha) * vy_prev_
    : vy_raw;
  vy_smoothed = std::clamp(
    vy_smoothed,
    -params_->v_lateral_max,
    params_->v_lateral_max);

  vy_prev_ = vy_smoothed;
  has_history_ = true;

  if (dbg) {
    dbg->vy_raw = vy_raw;
    dbg->vy_smoothed = vy_smoothed;
    dbg->override_active = true;
  }
  return vy_smoothed;
}

}  // namespace nav2_constrained_controller
