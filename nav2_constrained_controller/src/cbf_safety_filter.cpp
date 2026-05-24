// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/cbf_safety_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "nav2_constrained_controller/parameter_handler.hpp"

namespace nav2_constrained_controller
{

CBFSafetyFilter::CBFSafetyFilter(const Parameters * params)
: params_(params)
{
}

std::array<Point2D, 4> CBFSafetyFilter::bodyCorners(
  double xr, double yr, double theta) const
{
  // Saradagi Eq. 1 convention, body centred on base_link.
  // Rectangle: (L/2 + dl) longitudinal half-length, db lateral half-width.
  // P1 front-right, P2 back-right, P3 back-left, P4 front-left.
  const double Lext = 0.5 * params_->footprint_length + params_->footprint_dl;
  const double db   = params_->footprint_db;
  const double ct   = std::cos(theta);
  const double st   = std::sin(theta);
  std::array<Point2D, 4> c;
  c[0].x = xr + Lext * ct + db * st;   c[0].y = yr + Lext * st - db * ct;
  c[1].x = xr - Lext * ct + db * st;   c[1].y = yr - Lext * st - db * ct;
  c[2].x = xr - Lext * ct - db * st;   c[2].y = yr - Lext * st + db * ct;
  c[3].x = xr + Lext * ct - db * st;   c[3].y = yr + Lext * st + db * ct;
  return c;
}

CbfFilterResult CBFSafetyFilter::filter(
  const EsdfGrid & esdf,
  const geometry_msgs::msg::Twist & u_nom)
{
  CbfFilterResult res;
  res.corners = bodyCorners(0.0, 0.0, 0.0);
  res.min_h   = std::numeric_limits<double>::infinity();

  // Pre-compute u_nom as Eigen for constraint debug fields (Au, margin).
  const Eigen::Vector3d u_nom_e(u_nom.linear.x, u_nom.linear.y, u_nom.angular.z);

  // Perimeter sampling: query ESDF at multiple points along each side of the
  // robot body, not just the 4 corners. Door frames contact the MIDDLE of the
  // long side — a corner-only approach produces no constraint there.
  //
  // Layout (θ=0, base_link frame):
  //   Long sides  (y = ±db): kNLongSide samples each, endpoints = corners.
  //   Short sides (x = ±Lext): 1 midpoint each (corners already in long sides).
  //
  // corner_id encodes which quadrant each sample belongs to for RViz colouring:
  //   1 = front-right (x≥0, y<0)   2 = back-right  (x<0, y<0)
  //   3 = back-left   (x<0, y≥0)   4 = front-left  (x≥0, y≥0)
  //  -1 = short-side midpoints (contribute to min_h / QP but not corner colour)
  //
  // Total: 2*kNLongSide + 2*kNShortSide = 20 samples.
  // Short sides now match long-side density: door posts are diagonal and contact
  // between the single midpoint and the corners — a critical blind spot.
  static constexpr int kNLongSide  = 5;  // samples per long side  (incl. corners)
  static constexpr int kNShortSide = 5;  // samples per short side (incl. corners)

  const double Lext = 0.5 * params_->footprint_length + params_->footprint_dl;
  const double db   = params_->footprint_db;

  struct Sample { Point2D p; int corner_id; };
  std::array<Sample, 2 * kNLongSide + 2 * kNShortSide> samples;
  int ns = 0;

  // Long sides (y = ±db), 5 samples each including corners.
  for (int k = 0; k < kNLongSide; ++k) {
    const double t  = k / static_cast<double>(kNLongSide - 1);
    const double px = -Lext + t * 2.0 * Lext;
    const int id_right = (px >= 0.0) ? 1 : 2;
    const int id_left  = (px >= 0.0) ? 4 : 3;
    samples[ns++] = {{px, -db}, id_right};
    samples[ns++] = {{px, +db}, id_left};
  }
  // Short sides (x = ±Lext), 5 samples each including corners already above.
  // Only add the INTERMEDIATE samples (skip endpoints = corners, already added).
  for (int k = 1; k < kNShortSide - 1; ++k) {
    const double t  = k / static_cast<double>(kNShortSide - 1);
    const double py = -db + t * 2.0 * db;
    samples[ns++] = {{ Lext, py}, (py >= 0.0) ? 4 : 1};  // front: FL or FR
    samples[ns++] = {{-Lext, py}, (py >= 0.0) ? 3 : 2};  // rear:  BL or BR
  }

  // ── Retreat overlay: scan body for min_h_react ─────────────────────────
  // Walk all body samples once up-front to find the worst (smallest h). This
  // drives the RETREAT state-machine and identifies the gradient at the worst
  // sample for candidate construction. The reactive loop below also queries
  // these samples but range-gates by wall_consideration_range — the overlay
  // needs the true min_h regardless of range gating.
  double min_h_react = std::numeric_limits<double>::infinity();
  Point2D worst_pc{0.0, 0.0};
  double worst_gx = 0.0, worst_gy = 0.0;
  for (int si = 0; si < ns; ++si) {
    const Point2D & pc = samples[si].p;
    const auto q = esdf.query(pc.x, pc.y);
    if (!q.valid) {continue;}
    const double h = q.d - params_->esdf_d_safe;
    if (h < min_h_react) {
      min_h_react = h;
      worst_pc = pc;
      worst_gx = q.gx;
      worst_gy = q.gy;
    }
  }
  res.min_h_react = std::isfinite(min_h_react) ? min_h_react : 0.0;

  // ── Retreat overlay: state machine ─────────────────────────────────────
  // Three states: NORMAL (path-following), RETREAT (overlay overrides u_nom
  // with best candidate), GIVE_UP (zero output → Nav2 BT recovery). Hysteresis
  // between NORMAL and RETREAT via h_enter < h_exit prevents flapping. GIVE_UP
  // triggers when RETREAT has been active for cbf_retreat_give_up_ticks
  // without min_h improving by cbf_retreat_progress_eps from its entry value.
  if (params_->cbf_retreat_enabled && std::isfinite(min_h_react)) {
    const double h_enter = params_->cbf_retreat_h_enter;
    const double h_exit  = params_->cbf_retreat_h_exit;

    switch (retreat_state_) {
      case RetreatState::NORMAL:
        if (min_h_react < h_enter) {
          retreat_state_     = RetreatState::RETREAT;
          retreat_ticks_     = 0;
          min_h_at_entry_    = min_h_react;
        }
        break;
      case RetreatState::RETREAT:
        if (min_h_react >= h_exit) {
          retreat_state_ = RetreatState::NORMAL;
          retreat_ticks_ = 0;
        } else {
          retreat_ticks_++;
          if (retreat_ticks_ >= params_->cbf_retreat_give_up_ticks &&
              min_h_react < min_h_at_entry_ + params_->cbf_retreat_progress_eps)
          {
            retreat_state_ = RetreatState::GIVE_UP;
          }
        }
        break;
      case RetreatState::GIVE_UP:
        // Exit when the body is comfortably clear again — typically after the
        // BT has run a recovery and the robot is repositioned. Also cleared on
        // setPlan() via reset(), so a fresh plan always starts in NORMAL.
        if (min_h_react >= h_exit) {
          retreat_state_ = RetreatState::NORMAL;
          retreat_ticks_ = 0;
        }
        break;
    }
  }
  res.retreat_state = retreat_state_;

  // ── Retreat overlay: GIVE_UP short-circuit ─────────────────────────────
  // No QP, no candidates — emit zero twist and update predictor cache to
  // match. Stuck-detection in ConstrainedController will see the zero output
  // and trigger BT recovery.
  if (retreat_state_ == RetreatState::GIVE_UP) {
    res.u.linear.x = res.u.linear.y = res.u.angular.z = 0.0;
    res.min_h           = res.min_h_react;
    res.qp.ok           = true;
    res.qp.u            = Eigen::Vector3d::Zero();
    res.qp.deviation    = 0.0;
    res.qp.n_active     = 0;
    res.qp.iterations   = 0;
    res.qp.max_slack    = 0.0;
    res.picked_candidate = 0;
    res.picked_score     = 0.0;
    last_u_star_ = Eigen::Vector3d::Zero();
    has_last_u_  = true;
    return res;
  }

  // ── Retreat overlay: candidate selection ──────────────────────────────
  // In RETREAT state, build 9 candidate velocities and score each by ESDF
  // lookahead at all 16 body samples. The candidate with the highest predicted
  // min_h wins. Blend with u_nom by how critical min_h is (linear in
  // [h_enter, 0]) and feed the blended result to the QP as the target.
  //
  // Candidates (all in base_link, body velocity):
  //   1: u_nom                                — status quo
  //   2: 0.5·u_nom                            — slow follow
  //   3: +∇d_center · v_retreat               — move along centre ESDF gradient
  //   4: +∇d_worst  · v_retreat               — move along worst-sample gradient
  //   5: (−vx_nom, −vy_nom, +wz_nom)          — reverse translation, keep rotation
  //   6: (0, 0, +w_retreat)                   — pure CCW yaw
  //   7: (0, 0, −w_retreat)                   — pure CW yaw
  //   8: (0, +v_retreat, 0)                   — pure left strafe
  //   9: (0, −v_retreat, 0)                   — pure right strafe
  Eigen::Vector3d u_qp_target_e = u_nom_e;
  int picked_idx_1based = 0;
  double picked_score   = 0.0;

  if (params_->cbf_retreat_enabled && retreat_state_ == RetreatState::RETREAT) {
    const double v_ret  = params_->cbf_retreat_speed;
    const double w_ret  = params_->cbf_retreat_rotation_speed;
    const double tau_l  = params_->cbf_retreat_lookahead_s;
    const double d_safe = params_->esdf_d_safe;
    const double h_enter = params_->cbf_retreat_h_enter;

    // Candidate 3 ingredients: ESDF gradient at robot centre (base_link origin).
    // Falls through to zero if the centre cell is outside the grid or has a
    // degenerate gradient (e.g., uniform-field cell).
    Eigen::Vector3d retreat_center = Eigen::Vector3d::Zero();
    {
      const auto qc = esdf.query(0.0, 0.0);
      if (qc.valid) {
        const double nc = std::hypot(qc.gx, qc.gy);
        if (nc > 1e-6) {
          retreat_center.x() = (qc.gx / nc) * v_ret;
          retreat_center.y() = (qc.gy / nc) * v_ret;
        }
      }
    }
    // Candidate 4 ingredients: gradient at the worst body sample. Already in
    // hand from the scan above.
    Eigen::Vector3d retreat_worst = Eigen::Vector3d::Zero();
    {
      const double nw = std::hypot(worst_gx, worst_gy);
      if (nw > 1e-6) {
        retreat_worst.x() = (worst_gx / nw) * v_ret;
        retreat_worst.y() = (worst_gy / nw) * v_ret;
      }
    }

    const std::array<Eigen::Vector3d, 9> candidates = {
      u_nom_e,                                                          // 1
      0.5 * u_nom_e,                                                    // 2
      retreat_center,                                                   // 3
      retreat_worst,                                                    // 4
      Eigen::Vector3d(-u_nom_e.x(), -u_nom_e.y(), u_nom_e.z()),         // 5
      Eigen::Vector3d(0.0, 0.0,  w_ret),                                // 6
      Eigen::Vector3d(0.0, 0.0, -w_ret),                                // 7
      Eigen::Vector3d(0.0,  v_ret, 0.0),                                // 8
      Eigen::Vector3d(0.0, -v_ret, 0.0),                                // 9
    };

    // Score = min h_pred over all 16 body samples after τ_l of motion with u_c.
    // Pick the candidate with the highest score (largest body clearance).
    double best_score = -std::numeric_limits<double>::infinity();
    int best_idx = 0;
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
      const auto & u_c = candidates[ci];
      const double vx_c = u_c.x();
      const double vy_c = u_c.y();
      const double wz_c = u_c.z();

      double score = std::numeric_limits<double>::infinity();
      for (int si = 0; si < ns; ++si) {
        const Point2D & pc = samples[si].p;
        const double pred_x = pc.x + tau_l * (vx_c - wz_c * pc.y);
        const double pred_y = pc.y + tau_l * (vy_c + wz_c * pc.x);
        const auto q = esdf.query(pred_x, pred_y);
        if (!q.valid) {continue;}
        const double h_pred = q.d - d_safe;
        if (h_pred < score) {score = h_pred;}
      }
      if (score > best_score) {
        best_score = score;
        best_idx   = static_cast<int>(ci);
      }
    }

    // Smooth blend: at min_h_react = h_enter, blend = 0 (pure u_nom); at
    // min_h_react = 0, blend = 1 (pure candidate). Negative min_h saturates
    // to blend = 1 (we want full override when already inside the buffer).
    const double blend = std::clamp(
      (h_enter - min_h_react) / std::max(1e-6, h_enter), 0.0, 1.0);
    u_qp_target_e = (1.0 - blend) * u_nom_e + blend * candidates[best_idx];

    picked_idx_1based      = best_idx + 1;
    picked_score           = best_score;
    last_picked_candidate_ = picked_idx_1based;
  }
  res.picked_candidate = picked_idx_1based;
  res.picked_score     = picked_score;

  // From here on the existing QP code treats u_qp_target_e as the optimisation
  // target (= u_nom in NORMAL, = blended retreat target in RETREAT).
  for (int si = 0; si < ns; ++si) {
    const Point2D & pc = samples[si].p;
    const auto q = esdf.query(pc.x, pc.y);

    if (!q.valid) {continue;}

    const double h = q.d - params_->esdf_d_safe;

    // Range gate: skip samples far from any obstacle.
    if (h > params_->wall_consideration_range) {continue;}

    // Degenerate gradient (e.g. exactly on the medial axis): skip.
    if (std::hypot(q.gx, q.gy) < 1e-6) {continue;}

    // CBF condition: ∇d^T · J_i · u ≥ -γ·h
    // J_i = [[1, 0, -py_i], [0, 1, px_i]]  at θ=0
    // QP ≤-form:  [-gx, -gy, gx·py - gy·px] · u  ≤  γ·h
    CbfConstraint c;
    const double lever = q.gx * pc.y - q.gy * pc.x;
    c.grad      = Eigen::Vector3d(-q.gx, -q.gy, lever);
    c.rhs       = params_->cbf_gamma * h;
    c.h         = h;
    c.corner_id = samples[si].corner_id;
    // Debug fields evaluated against the QP target (= u_nom if NORMAL).
    c.Au          = c.grad.dot(u_qp_target_e);
    c.margin      = c.rhs - c.Au;
    c.gx          = q.gx; c.gy = q.gy;
    c.sample_x    = pc.x; c.sample_y = pc.y;
    c.predict_step = 0;

    res.constraints.push_back(c);
    if (h < res.min_h) {res.min_h = h;}
  }

  if (res.constraints.empty()) {res.min_h = 0.0;}

  // ── Predictive CBF ──────────────────────────────────────────────────────
  // For each future step j=1..N, predict where each perimeter sample will be
  // if the robot continues with the PREDICTOR velocity (last tick's u*, or
  // u_nom on the first tick), then add a CBF constraint based on the predicted
  // ESDF reading at that future position.
  //
  // Predictor choice: last_u_star_ instead of u_nom. The QP is going to
  // re-shape u_nom into u* anyway — using u* from the previous tick as the
  // predictor makes the rollout reflect the trajectory the robot will actually
  // follow, not a hypothetical "what if we ignored safety" trajectory. The
  // previous tick's u* is a one-tick-stale estimate of this tick's u*, but at
  // 10Hz with smooth costmaps this is a tight approximation. Avoids the
  // chicken-and-egg of fixed-point iteration with no measurable cost.
  //
  // Rotated Jacobian: the predicted-step Jacobian uses rotation θ̂_j =
  // wz_predict · j · dt to account for body rotation across the prediction
  // horizon. At θ̂_j ≠ 0 the world-frame velocity of a body-fixed sample is
  // R(θ̂_j) · J_body(px,py) · u; rotating the gradient by θ̂_j (same as
  // pre-multiplying J by R) gives an equivalent linear-in-u QP row. When
  // wz_predict = 0 this degenerates to the previous (constant-θ) formulation.
  //
  // Constraint formulation: at step j the predicted sample position is
  //   p_pred = p_curr + τ·[vx_p − wz_p·py, vy_p + wz_p·px]
  // where (vx_p, vy_p, wz_p) is the predictor velocity and τ = j·predict_dt.
  // We query the ESDF gradient (gx, gy) at p_pred, rotate it by θ̂_j, then
  // build the QP row using the rotated gradient and the body-fixed (px, py).
  {
    const int    N   = params_->cbf_n_predict_steps;
    const double dt  = params_->cbf_predict_dt;

    // Predictor velocity: last-tick u* if we have one, otherwise fall back to
    // u_nom (first call after setPlan/activate).
    const Eigen::Vector3d u_pred = has_last_u_ ? last_u_star_ : u_nom_e;
    const double vx_p = u_pred.x();
    const double vy_p = u_pred.y();
    const double wz_p = u_pred.z();

    for (int step = 1; step <= N; ++step) {
      const double tau    = step * dt;
      const double theta  = wz_p * tau;
      const double cos_t  = std::cos(theta);
      const double sin_t  = std::sin(theta);

      for (int si = 0; si < ns; ++si) {
        const Point2D & pc = samples[si].p;

        // Predict sample position: p + τ · J(p) · u_pred
        const double pred_x = pc.x + tau * (vx_p - wz_p * pc.y);
        const double pred_y = pc.y + tau * (vy_p + wz_p * pc.x);

        const auto q = esdf.query(pred_x, pred_y);
        if (!q.valid) {continue;}

        const double h = q.d - params_->esdf_d_safe;
        if (h > params_->wall_consideration_range) {continue;}
        if (std::hypot(q.gx, q.gy) < 1e-6) {continue;}

        // Rotate gradient by θ̂_j: ∇d · R(θ̂_j). Equivalent to applying the
        // body→world rotation to J before dotting with ∇d. Keeps the QP row
        // linear in u while respecting accumulated yaw across the horizon.
        const double gx_eff = q.gx * cos_t + q.gy * sin_t;
        const double gy_eff = -q.gx * sin_t + q.gy * cos_t;

        // QP row uses effective gradient at PREDICTED position (sees upcoming
        // wall, accounts for predicted body rotation) and body-fixed (px, py).
        CbfConstraint c;
        const double lever = gx_eff * pc.y - gy_eff * pc.x;
        c.grad      = Eigen::Vector3d(-gx_eff, -gy_eff, lever);
        c.rhs       = params_->cbf_gamma * h;
        c.h         = h;
        c.corner_id = samples[si].corner_id;
        // Debug fields evaluated against the QP target (= u_nom if NORMAL,
        // blended retreat target if RETREAT). Log raw (un-rotated) gradient.
        c.Au           = c.grad.dot(u_qp_target_e);
        c.margin       = c.rhs - c.Au;
        c.gx           = q.gx; c.gy = q.gy;
        c.sample_x     = pred_x; c.sample_y = pred_y;
        c.predict_step = step;

        res.constraints.push_back(c);
        if (h < res.min_h) {res.min_h = h;}
      }
    }
  }

  // Assemble QP: min ½||u - u_nom||²  s.t. Au ≤ b, u_min ≤ u ≤ u_max
  const int m = static_cast<int>(res.constraints.size());
  Eigen::MatrixXd A(m, 3);
  Eigen::VectorXd b(m);
  for (int i = 0; i < m; ++i) {
    A.row(i) = res.constraints[i].grad.transpose();
    b(i)     = res.constraints[i].rhs;
  }

  const Eigen::Vector3d u_min(
    -params_->v_linear_max,
    -params_->v_lateral_max,
    -params_->v_angular_max);
  const Eigen::Vector3d u_max(
    params_->v_linear_max,
    params_->v_lateral_max,
    params_->v_angular_max);
  const auto t0 = std::chrono::steady_clock::now();
  res.qp = solveProjectionQP(A, b, u_min, u_max, u_qp_target_e, params_->cbf_slack_weight);
  const auto t1 = std::chrono::steady_clock::now();
  res.solve_time_us =
    std::chrono::duration<double, std::micro>(t1 - t0).count();

  res.u.linear.x  = res.qp.u.x();
  res.u.linear.y  = res.qp.u.y();
  res.u.angular.z = res.qp.u.z();

  // Save u* for next tick's predictor. On QP infeasibility the controller
  // overrides res.u to zero downstream — but we still cache res.qp.u (the QP's
  // best attempt) so the next-tick predictor has a sensible velocity rather
  // than a hard zero, which would collapse all predicted positions onto the
  // current body samples and silence the predictive horizon.
  last_u_star_  = res.qp.u;
  has_last_u_   = true;
  return res;
}

}  // namespace nav2_constrained_controller
