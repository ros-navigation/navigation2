// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/esdf_critic.hpp"

#include <cmath>
#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_constrained_controller/esdf_grid.hpp"
#include "nav2_constrained_controller/parameter_handler.hpp"

namespace mppi::critics
{

void EsdfCritic::initialize()
{
  // parent_name_ = "ConstraintFollowPath.mppi"  →  controller_name_ = "ConstraintFollowPath"
  const auto pos = parent_name_.rfind(".mppi");
  controller_name_ = (pos != std::string::npos)
    ? parent_name_.substr(0, pos)
    : parent_name_;

  // Read footprint from the controller's parameter namespace so it stays in sync.
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(enabled_,        "enabled",          true);
  getParam(cost_weight_,    "cost_weight",       20.0f);
  getParam(collision_cost_, "collision_cost",    5000.0f);
  getParam(near_dist_,      "near_dist",         0.15f);
  getParam(sigma_,          "sigma",             0.06f);

  // Footprint parameters — read from the controller's root namespace.
  // Default values match the YAML; override via controller-level params if needed.
  auto getCtrlParam = parameters_handler_->getParamGetter(controller_name_);
  float footprint_length = 0.90f;
  float footprint_dl     = 0.05f;
  float footprint_db     = 0.375f;
  getCtrlParam(footprint_length, "footprint_length", 0.90f);
  getCtrlParam(footprint_dl,     "footprint_dl",     0.05f);
  getCtrlParam(footprint_db,     "footprint_db",     0.375f);
  getCtrlParam(d_safe_,          "esdf_d_safe",      0.02f);

  Lext_ = 0.5f * footprint_length + footprint_dl;
  db_   = footprint_db;

  RCLCPP_INFO(logger_,
    "EsdfCritic initialized for controller='%s' "
    "Lext=%.3f db=%.3f d_safe=%.3f near_dist=%.3f weight=%.1f collision=%.0f",
    controller_name_.c_str(), Lext_, db_, d_safe_, near_dist_,
    cost_weight_, collision_cost_);
}

void EsdfCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {return;}

  const auto entry = nav2_constrained_controller::EsdfRegistry::instance().get(controller_name_);
  if (!entry.grid || !entry.grid->hasData()) {return;}

  const nav2_constrained_controller::EsdfGrid & esdf = *entry.grid;

  // Pre-compute rotation from odom frame to base_link frame.
  const float cos_neg = std::cos(-static_cast<float>(entry.robot_yaw));
  const float sin_neg = std::sin(-static_cast<float>(entry.robot_yaw));
  const float rx      = static_cast<float>(entry.robot_x);
  const float ry      = static_cast<float>(entry.robot_y);
  const float ryaw    = static_cast<float>(entry.robot_yaw);

  // Perimeter sample layout — mirrors cbf_safety_filter.cpp exactly so the
  // critic sees the same contact points as the CBF.
  // 5 samples per long side (y=±db) + 1 midpoint per short side = 12 total.
  struct Sample { float lx, ly; };  // in robot body frame (θ=0)
  static constexpr int kNLong  = 5;
  static constexpr int kNShort = 5;  // matches CBF — catches diagonal door posts
  std::array<Sample, 2 * kNLong + 2 * (kNShort - 2)> samples;
  {
    int ns = 0;
    for (int k = 0; k < kNLong; ++k) {
      const float t_k = k / static_cast<float>(kNLong - 1);
      const float px  = -Lext_ + t_k * 2.0f * Lext_;
      samples[ns++] = {px, -db_};
      samples[ns++] = {px, +db_};
    }
    for (int k = 1; k < kNShort - 1; ++k) {
      const float t_k = k / static_cast<float>(kNShort - 1);
      const float py  = -db_ + t_k * 2.0f * db_;
      samples[ns++] = { Lext_, py};
      samples[ns++] = {-Lext_, py};
    }
  }
  constexpr int kNSamples = 2 * kNLong + 2 * (kNShort - 2);  // = 16

  const size_t B = data.trajectories.x.shape(0);
  const size_t T = data.trajectories.x.shape(1);

  for (size_t b = 0; b < B; ++b) {
    float traj_cost = 0.0f;

    for (size_t t = 0; t < T; t += step_stride_) {
      // Predicted pose in odom frame.
      const float wx   = data.trajectories.x(b, t);
      const float wy   = data.trajectories.y(b, t);
      const float wyaw = data.trajectories.yaws(b, t);

      // Convert to base_link-relative coords (robot at origin, θ=0).
      const float dx   = wx - rx;
      const float dy   = wy - ry;
      const float bx   = dx * cos_neg - dy * sin_neg;
      const float by   = dx * sin_neg + dy * cos_neg;
      const float byaw = wyaw - ryaw;
      const float cby  = std::cos(byaw);
      const float sby  = std::sin(byaw);

      // Query ESDF at all 12 perimeter samples transformed by the predicted pose.
      float min_h = 1e6f;
      for (int si = 0; si < kNSamples; ++si) {
        const float sx = samples[si].lx;
        const float sy = samples[si].ly;
        // Rotate sample from body to base_link.
        const float px = bx + sx * cby - sy * sby;
        const float py = by + sx * sby + sy * cby;

        const auto q = esdf.query(static_cast<double>(px),
                                  static_cast<double>(py));
        if (!q.valid) {continue;}
        const float h = static_cast<float>(q.d) - d_safe_;
        if (h < min_h) {min_h = h;}
      }

      if (min_h < 0.0f) {
        traj_cost += collision_cost_;
      } else if (min_h < near_dist_) {
        traj_cost += cost_weight_ * std::exp(-min_h / sigma_);
      }
    }

    data.costs(b) += traj_cost;
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::EsdfCritic,
  mppi::critics::CriticFunction)
