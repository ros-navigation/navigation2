// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#include "nav2_constrained_controller/esdf_grid.hpp"
#include "nav2_constrained_controller/parameter_handler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_constrained_controller
{

static constexpr float kInf = 1e15f;

EsdfGrid::EsdfGrid(const Parameters * params)
: params_(params)
{
  n_ = static_cast<int>(
    std::round(2.0 * params_->esdf_grid_size_m / params_->esdf_grid_resolution));
  n_ = std::max(n_, 4);
  res_ = params_->esdf_grid_resolution;
  half_ = 0.5 * n_ * res_;

  dist_.resize(n_ * n_, 0.0f);
  gx_.resize(n_ * n_, 0.0f);
  gy_.resize(n_ * n_, 0.0f);
}

int EsdfGrid::toIdx(double v) const
{
  return static_cast<int>(std::floor((v + half_) / res_));
}

void EsdfGrid::update(const std::vector<Point2D> & points_base)
{
  // Build binary occupancy: 0.0 = obstacle source, kInf = free.
  std::vector<float> occ(n_ * n_, kInf);

  for (const auto & p : points_base) {
    const int ci = toIdx(p.x);
    const int cj = toIdx(p.y);
    if (ci < 0 || ci >= n_ || cj < 0 || cj >= n_) {continue;}
    occ[ci + cj * n_] = 0.0f;
  }

  // 2D EDT → squared distances in cells².
  computeEDT(occ);

  // Convert cells² → metres and store. Compute gradients via central
  // differences (one-sided at boundaries).
  const float res_f = static_cast<float>(res_);
  for (int i = 0; i < n_ * n_; ++i) {
    dist_[i] = std::sqrt(occ[i]) * res_f;
  }

  for (int cj = 0; cj < n_; ++cj) {
    for (int ci = 0; ci < n_; ++ci) {
      const int idx = ci + cj * n_;

      const float dxp = (ci + 1 < n_) ? dist_[(ci + 1) + cj * n_] : dist_[idx];
      const float dxm = (ci - 1 >= 0) ? dist_[(ci - 1) + cj * n_] : dist_[idx];
      const int wci   = (ci + 1 < n_ && ci - 1 >= 0) ? 2 : 1;

      const float dyp = (cj + 1 < n_) ? dist_[ci + (cj + 1) * n_] : dist_[idx];
      const float dym = (cj - 1 >= 0) ? dist_[ci + (cj - 1) * n_] : dist_[idx];
      const int wcj   = (cj + 1 < n_ && cj - 1 >= 0) ? 2 : 1;

      gx_[idx] = (dxp - dxm) / (wci * res_f);
      gy_[idx] = (dyp - dym) / (wcj * res_f);
    }
  }

  has_data_ = true;
}

EsdfGrid::QueryResult EsdfGrid::query(double x, double y) const
{
  QueryResult r;
  if (!has_data_) {return r;}

  const double fx = (x + half_) / res_;
  const double fy = (y + half_) / res_;
  const int ci0 = static_cast<int>(std::floor(fx));
  const int cj0 = static_cast<int>(std::floor(fy));

  if (ci0 < 0 || ci0 + 1 >= n_ || cj0 < 0 || cj0 + 1 >= n_) {
    return r;
  }

  const float dx = static_cast<float>(fx - ci0);
  const float dy = static_cast<float>(fy - cj0);

  auto interp = [&](const std::vector<float> & field) -> double {
    const float v00 = field[ci0 + cj0 * n_];
    const float v10 = field[(ci0 + 1) + cj0 * n_];
    const float v01 = field[ci0 + (cj0 + 1) * n_];
    const float v11 = field[(ci0 + 1) + (cj0 + 1) * n_];
    return static_cast<double>(
      (1 - dx) * (1 - dy) * v00 +
      dx * (1 - dy) * v10 +
      (1 - dx) * dy * v01 +
      dx * dy * v11);
  };

  r.d = interp(dist_);
  r.gx = interp(gx_);
  r.gy = interp(gy_);
  r.valid = true;
  return r;
}

void EsdfGrid::computeEDT(std::vector<float> & occ) const
{
  // Two-pass 2D EDT (Felzenszwalb & Huttenlocher, TPAMI 2012).
  // Pass 1: per column — EDT along cj axis (y-direction).
  // Pass 2: per row   — EDT along ci axis (x-direction).
  std::vector<float> strip(n_), result(n_);

  for (int ci = 0; ci < n_; ++ci) {
    for (int cj = 0; cj < n_; ++cj) {strip[cj] = occ[ci + cj * n_];}
    edt1d(strip, result, n_);
    for (int cj = 0; cj < n_; ++cj) {occ[ci + cj * n_] = result[cj];}
  }

  for (int cj = 0; cj < n_; ++cj) {
    for (int ci = 0; ci < n_; ++ci) {strip[ci] = occ[ci + cj * n_];}
    edt1d(strip, result, n_);
    for (int ci = 0; ci < n_; ++ci) {occ[ci + cj * n_] = result[ci];}
  }
}

void EsdfGrid::edt1d(
  const std::vector<float> & f,
  std::vector<float> & d,
  int n)
{
  // Felzenszwalb-Huttenlocher 1D parabola lower-envelope algorithm.
  // Computes d[q] = min_p { f[p] + (q-p)² } for all q in [0, n).
  // f[p] = 0 at obstacle cells, kInf at free cells.
  //
  // z[0] = -∞ guarantees the while loop exits when k == 0, so k never
  // goes negative.
  std::vector<int> v(n);
  std::vector<float> z(n + 1);

  constexpr float NEG_INF = -1e30f;
  constexpr float POS_INF = 1e30f;

  int k = 0;
  v[0] = 0;
  z[0] = NEG_INF;
  z[1] = POS_INF;

  for (int q = 1; q < n; ++q) {
    // Compute intersection of parabola at q with rightmost parabola.
    auto s = [&]() -> float {
      const int r = v[k];
      return ((f[q] + static_cast<float>(q * q)) -
        (f[r] + static_cast<float>(r * r))) /
        (2.0f * static_cast<float>(q - r));
    };
    float sv = s();
    while (sv <= z[k]) {
      --k;
      sv = s();
    }
    ++k;
    v[k] = q;
    z[k] = sv;
    z[k + 1] = POS_INF;
  }

  d.resize(n);
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q)) {++k;}
    const float dq = static_cast<float>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

}  // namespace nav2_constrained_controller
