// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
#define nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Core"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_ceres_costaware_smoother/options.hpp"

#define EPSILON_DOUBLE 0.0001
#define EPSILON (T)EPSILON_DOUBLE

namespace nav2_ceres_costaware_smoother
{

template <typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(
  Eigen::Matrix<T, 2, 1> p1,
  Eigen::Matrix<T, 2, 1> p,
  Eigen::Matrix<T, 2, 1> p2,
  int forced_dot_sign = 0)
{
  Eigen::Matrix<T, 2, 1> d1 = p - p1;
  Eigen::Matrix<T, 2, 1> d2 = p2 - p;
  
  if (forced_dot_sign < 0 || (forced_dot_sign == 0 && d1.dot(d2) < (T)0)) {
    d2 = -d2;
    p2 = p + d2;
  }

  T det = d1[0]*d2[1] - d1[1]*d2[0];
  if (ceres::abs(det) < (T)1e-4) { // straight line
    return Eigen::Matrix<T, 2, 1>((T)std::numeric_limits<double>::infinity(), (T)std::numeric_limits<double>::infinity());
  }

  // circle center is at the intersection of the mirror axes of the segments: http://paulbourke.net/geometry/circlesphere/
  // intersection: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Intersection%20of%20two%20lines
  Eigen::Matrix<T, 2, 1> mid1 = (p1 + p)/(T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (p + p2)/(T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  T det1 = (mid1[0] + n1[0])*mid1[1] - (mid1[1] + n1[1])*mid1[0];
  T det2 = (mid2[0] + n2[0])*mid2[1] - (mid2[1] + n2[1])*mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1*n2[0] - det2*n1[0])/det, (det1*n2[1] - det2*n1[1])/det);
  return center;
}

template <typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(
  Eigen::Matrix<T, 2, 1> p1,
  Eigen::Matrix<T, 2, 1> p,
  Eigen::Matrix<T, 2, 1> p2,
  int forced_dot_sign = 0)
{
  Eigen::Matrix<T, 2, 1> center = arcCenter(p1, p, p2, forced_dot_sign);
  if (ceres::IsInfinite(center[0])) {  // straight line
    Eigen::Matrix<T, 2, 1> d1 = p - p1;
    Eigen::Matrix<T, 2, 1> d2 = p2 - p;
    
    if (forced_dot_sign < 0 || (forced_dot_sign == 0 && d1.dot(d2) < (T)0)) {
      d2 = -d2;
      p2 = p + d2;
      // // changed from forward movement to reverse or vice versa - use average direction towards p
      // Eigen::Matrix<T, 2, 1> tangent_dir = d1.norm()*d1 - d2.norm()*d2;
      // return tangent_dir;
    }
    
    return Eigen::Matrix<T, 2, 1>(p2[0] - p1[0], p2[1] - p1[1]);
  }
  
  return Eigen::Matrix<T, 2, 1>(center[1] - p[1], p[0] - center[0]); // prependicular to (p - center)
}

/**
 * @struct nav2_ceres_costaware_smoother::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, collision, and avoid obstacles.
 */
class SmootherCostFunction
{
public:
  /**
   * @brief A constructor for nav2_ceres_costaware_smoother::SmootherCostFunction
   * @param original_path Original position of the path node
   * @param next_to_last_length_ratio Ratio of next path segment compared to previous.
   *  Negative if one of them represents reversing motion.
   * @param reversing Whether the path segment after this node represents reversing motion.
   * @param costmap A costmap to get values for collision and obstacle avoidance
   * @param params Optimization weights and parameters 
   * @param costmap_weight Costmap cost weight. Can be params.costmap_weight or params.dir_change_costmap_weight
   */
  SmootherCostFunction(
    const Eigen::Vector2d &_original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params,
    double costmap_weight)
  : _original_pos(_original_pos),
    _next_to_last_length_ratio(next_to_last_length_ratio),
    _reversing(reversing),
    _costmap(costmap),
    _params(params),
    _costmap_weight(costmap_weight)
  {
    _costmap_grid.reset(new ceres::Grid2D<u_char>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0, costmap->getSizeInCellsX()));
    _interpolate_costmap.reset(new ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>(*_costmap_grid));
  }

  ceres::CostFunction* AutoDiff() {
    return (new ceres::AutoDiffCostFunction<SmootherCostFunction, 4, 2, 2, 2>(this));
  }

  void setCostmapWeight(double costmap_weight) {
    _costmap_weight = costmap_weight;
  }

  /**
   * @struct CurvatureComputations
   * @brief Cache common computations between the curvature terms to minimize recomputations
   */
  template <typename T>
  struct CurvatureComputations
  {
    /**
     * @brief A constructor for nav2_smac_planner::CurvatureComputations
     */
    CurvatureComputations()
    {
      valid = true;
    }

    bool valid;
    /**
     * @brief Check if result is valid for penalty
     * @return is valid (non-nan, non-inf, and turning angle > max)
     */
    bool isValid()
    {
      return valid;
    }

    Eigen::Matrix<T, 2, 1> delta_xi{0.0, 0.0};
    Eigen::Matrix<T, 2, 1> delta_xi_p{0.0, 0.0};
    T delta_xi_norm{0};
    T delta_xi_p_norm{0};
    T delta_phi_i{0};
    T turning_rad{0};
    T ki_minus_kmax{0};
  };

  /**
   * @brief Smoother cost function evaluation
   * @param p X, Y coords of current point
   * @param p_p1 X, Y coords of next point
   * @param p_m1 X, Y coords of previous point
   * @param p_residual array of output residuals (smoothing, curvature, distance, cost)
   * @return if successful in computing values
   */
  template <typename T>
  bool operator()(const T* const p, const T* const p_p1, const T* const p_m1, T* p_residual) const {
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi(p);
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi_p1(p_p1);
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi_m1(p_m1);
    Eigen::Map<Eigen::Matrix<T, 4, 1> > residual(p_residual);
    residual.setZero();

    // cache some computations between the residual and jacobian
    CurvatureComputations<T> curvature_params;

    // compute cost
    addSmoothingResidual<T>(_params.smooth_weight, xi, xi_p1, xi_m1, residual[0]);//cost_raw);
    addCurvatureResidual<T>(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, residual[1]);//cost_raw);
    addDistanceResidual<T>(_params.distance_weight, xi, _original_pos.template cast<T>(), residual[2]);//cost_raw);
    addCostResidual<T>(_costmap_weight, xi, xi_p1, xi_m1, residual[3]);//cost_raw);

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_p,
    const Eigen::Matrix<T, 2, 1> & pt_m,
    T & r) const
  {
    Eigen::Matrix<T, 2, 1> d_pt_p = pt_p - pt;
    Eigen::Matrix<T, 2, 1> d_pt_m = pt - pt_m;
    Eigen::Matrix<T, 2, 1> d_pt_diff = _next_to_last_length_ratio*d_pt_p - d_pt_m;
    r += (T)weight * d_pt_diff.dot(d_pt_diff);    // objective function value
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_p,
    const Eigen::Matrix<T, 2, 1> & pt_m,
    CurvatureComputations<T> & curvature_params,
    T & r) const
  {
    curvature_params.valid = true;
    // curvature_params.delta_xi = Eigen::Matrix<T, 2, 1>(pt[0] - pt_m[0], pt[1] - pt_m[1]);
    // curvature_params.delta_xi_p = Eigen::Matrix<T, 2, 1>(pt_p[0] - pt[0], pt_p[1] - pt[1]);
    // if (_next_to_last_length_ratio < 0)
    //   curvature_params.delta_xi_p = -curvature_params.delta_xi_p;
    // curvature_params.delta_xi_norm = curvature_params.delta_xi.norm();
    // curvature_params.delta_xi_p_norm = curvature_params.delta_xi_p.norm();
    // if (curvature_params.delta_xi_norm < EPSILON || curvature_params.delta_xi_p_norm < EPSILON ||
    //   ceres::IsNaN(curvature_params.delta_xi_p_norm) || ceres::IsNaN(curvature_params.delta_xi_norm) ||
    //   ceres::IsInfinite(curvature_params.delta_xi_p_norm) || ceres::IsInfinite(curvature_params.delta_xi_norm))
    // {
    //   // ensure we have non-nan values returned
    //   curvature_params.valid = false;
    //   return;
    // }

    // const T & delta_xi_by_xi_p =
    //   curvature_params.delta_xi_norm * curvature_params.delta_xi_p_norm;
    // T projection =
    //   curvature_params.delta_xi.dot(curvature_params.delta_xi_p) / delta_xi_by_xi_p;
    // if (ceres::abs((T)1 - projection) < EPSILON || ceres::abs(projection + (T)1) < EPSILON) {
    //   projection = (T)1.0;
    // }

    // curvature_params.delta_phi_i = ceres::acos(projection);
    // curvature_params.turning_rad = curvature_params.delta_phi_i / curvature_params.delta_xi_norm;

    Eigen::Matrix<T, 2, 1> center = arcCenter(pt_m, pt, pt_p, _next_to_last_length_ratio < 0 ? -1 : 1);
    if (ceres::IsInfinite(center[0])) {
      // Quadratic penalty need not apply
      curvature_params.valid = false;
      return;
    }
    T turning_rad = (pt - center).norm();
    curvature_params.ki_minus_kmax = (T)1.0/turning_rad - _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= EPSILON) {
      // Quadratic penalty need not apply
      curvature_params.valid = false;
      return;
    }

    r += (T)weight *
      curvature_params.ki_minus_kmax * curvature_params.ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & xi,
    const Eigen::Matrix<T, 2, 1> & xi_original,
    T & r) const
  {
    r += (T)weight * (xi - xi_original).dot(xi - xi_original);  // objective function value
  }

  /**
   * @brief Cost function term for steering away from costs
   * @param weight Weight to apply to function
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCostResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> &pt,
    const Eigen::Matrix<T, 2, 1> &pt_p1,
    const Eigen::Matrix<T, 2, 1> &pt_m1,
    T & r) const
  {
    double origx = _costmap->getOriginX();
    double origy = _costmap->getOriginY();
    double res = _costmap->getResolution();

    if (!_params.cost_check_points.empty()) {
      Eigen::Matrix<T, 2, 1> dir = tangentDir(pt_m1, pt, pt_p1, _next_to_last_length_ratio < 0 ? -1 : 1);
      dir.normalize();
      if (((pt_p1 - pt).dot(dir) < (T)0) != _reversing)
        dir = -dir;
      Eigen::Matrix<T, 3, 3> transform;
      transform << dir[0], -dir[1], pt[0],
                  dir[1], dir[0], pt[1],
                  (T)0, (T)0, (T)1;
      for (size_t i = 0; i < _params.cost_check_points.size(); i += 3) {
        Eigen::Matrix<T, 3, 1> ccpt((T)_params.cost_check_points[i], (T)_params.cost_check_points[i+1], (T)1);
        auto ccptWorld = transform*ccpt;
        T interpx = (ccptWorld[0] - (T)origx) / (T)res - (T)0.5;
        T interpy = (ccptWorld[1] - (T)origy) / (T)res - (T)0.5;
        T value;
        _interpolate_costmap->Evaluate(interpy, interpx, &value);

        r += (T)weight * (T)_params.cost_check_points[i+2] * value * value;
      }
    }
    else {
      T interpx = (pt[0] - (T)origx) / (T)res - (T)0.5;
      T interpy = (pt[1] - (T)origy) / (T)res - (T)0.5;
      T value;
      _interpolate_costmap->Evaluate(interpy, interpx, &value);
      
      r += (T)weight * value * value;  // objective function value
    }
  }

  const Eigen::Vector2d &_original_pos;
  double _next_to_last_length_ratio;
  bool _reversing;
  nav2_costmap_2d::Costmap2D * _costmap{nullptr};
  SmootherParams _params;
  double _costmap_weight;
  std::unique_ptr<ceres::Grid2D<u_char>> _costmap_grid;
  std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> _interpolate_costmap;
};

}  // namespace nav2_smac_planner

#endif  // nav2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
