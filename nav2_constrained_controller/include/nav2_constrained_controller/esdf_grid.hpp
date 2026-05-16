// Copyright (c) 2026 Origin Autonomy
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONSTRAINED_CONTROLLER__ESDF_GRID_HPP_
#define NAV2_CONSTRAINED_CONTROLLER__ESDF_GRID_HPP_

#include <vector>
#include "nav2_constrained_controller/types.hpp"

namespace nav2_constrained_controller
{

struct Parameters;

// Local 2D ESDF built fresh every control tick from the current 3D
// LiDAR point cloud projected onto the XY plane in base_link frame.
//
// The grid is square, centred at the base_link origin. After calling
// update(), query(x, y) returns the Euclidean distance to the nearest
// obstacle and the gradient of that distance field (unit vector pointing
// away from the obstacle), both via bilinear interpolation.
//
// Distance transform: Felzenszwalb-Huttenlocher 2D EDT, O(n) in grid
// cells. Gradient: central differences on the distance field.
class EsdfGrid
{
public:
  struct QueryResult
  {
    double d{0.0};     // Euclidean distance to nearest obstacle (metres)
    double gx{0.0};   // ∂d/∂x (unit vector component away from obstacle)
    double gy{0.0};   // ∂d/∂y
    bool valid{false}; // false if position is outside grid bounds
  };

  explicit EsdfGrid(const Parameters * params);

  // Rebuild ESDF from 2D points already in base_link frame.
  // Points outside the grid extent are silently ignored.
  void update(const std::vector<Point2D> & points_base);

  // Query distance and gradient at world position (x, y) in base_link.
  // Uses bilinear interpolation over the distance and gradient grids.
  QueryResult query(double x, double y) const;

  bool hasData() const {return has_data_;}
  void reset() {has_data_ = false;}

  // Accessors for visualization (read-only).
  int gridSize()    const {return n_;}
  double resolution() const {return res_;}
  double halfSize() const {return half_;}
  const std::vector<float> & distGrid() const {return dist_;}

private:
  const Parameters * params_;

  int n_;       // cells per side
  double res_;  // metres per cell
  double half_; // n_ * res_ / 2.0 — half-width in metres

  // Flat n_×n_ arrays. Index: idx = ci + cj * n_
  // ci = x-cell (column), cj = y-cell (row).
  std::vector<float> dist_;  // Euclidean distance field (metres)
  std::vector<float> gx_;   // gradient x
  std::vector<float> gy_;   // gradient y
  bool has_data_{false};

  // Convert world coordinate to cell index (may be out of range).
  int toIdx(double v) const;

  // 2D Euclidean distance transform in-place on a flat n_×n_ array.
  // Input: occ[idx] = 0.0f if occupied, kInf if free.
  // Output: occ[idx] = squared distance in cells².
  void computeEDT(std::vector<float> & occ) const;

  // 1D Felzenszwalb-Huttenlocher EDT pass.
  // f[i]: input (source=0, free=kInf). d[i]: output (squared distance).
  static void edt1d(
    const std::vector<float> & f,
    std::vector<float> & d,
    int n);
};

}  // namespace nav2_constrained_controller

#endif  // NAV2_CONSTRAINED_CONTROLLER__ESDF_GRID_HPP_
