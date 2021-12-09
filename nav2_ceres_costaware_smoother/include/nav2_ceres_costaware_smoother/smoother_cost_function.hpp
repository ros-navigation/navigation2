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

#ifndef NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
#define NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_

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
#include "nav2_ceres_costaware_smoother/utils.hpp"

namespace nav2_ceres_costaware_smoother
{

template <typename T>
struct DoubleOnlyLogger {
  // template<typename... ArgTypes>
  static void log(const char*, T, T, T, T, T, T, T) {
  }
};

template <>
void DoubleOnlyLogger<double>::log(const char* format, double a1, double a2, double a3, double a4, double a5, double a6, double a7) {
  RCLCPP_INFO(rclcpp::get_logger("ceres_smoother"), format, a1, a2, a3, a4, a5, a6, a7);
}

/**
 * @struct nav2_ceres_costaware_smoother::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, distance from original and obstacle avoidance.
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
   * @param costmap_weight Costmap cost weight. Can be params.costmap_weight or params.cusp_costmap_weight
   */
  SmootherCostFunction(
    const Eigen::Vector2d &_original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> &costmap_interpolator,
    const SmootherParams & params,
    double costmap_weight)
  : _original_pos(_original_pos),
    _next_to_last_length_ratio(next_to_last_length_ratio),
    _reversing(reversing),
    _params(params),
    _costmap_weight(costmap_weight),
    _costmap_origin(costmap->getOriginX(), costmap->getOriginY()),
    _costmap_resolution(costmap->getResolution()),
    _costmap_interpolator(costmap_interpolator)
  {
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
   * @param pt_p Point Xi+1 for calculating Xi's cost
   * @param pt_m Point Xi-1 for calculating Xi's cost
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
   * @param pt_p Point Xi+1 for calculating Xi's cost
   * @param pt_m Point Xi-1 for calculating Xi's cost
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
    Eigen::Matrix<T, 2, 1> center = arcCenter(pt_m, pt, pt_p, _next_to_last_length_ratio < 0 ? -1 : 1);
    if (ceres::IsInfinite(center[0])) {
      return;
    }
    T turning_rad = (pt - center).norm();
    curvature_params.ki_minus_kmax = (T)1.0/turning_rad - _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= (T)EPSILON) {
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
    // if (weight != 0)
    //   DoubleOnlyLogger<T>::log("w: %lf, xi: %lf %lf, orig: %lf %lf, dist: %lf, res: %lf",
    //     (T)weight, xi[0], xi[1], xi_original[0], xi_original[1], (xi - xi_original).norm(), (T)weight * (xi - xi_original).squaredNorm());
    r += (T)weight * (xi - xi_original).squaredNorm();  // objective function value
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
    if (_params.cost_check_points.empty()) {
      Eigen::Matrix<T, 2, 1> interp_pos = (pt - _costmap_origin.template cast<T>()) / (T)_costmap_resolution;
      T value;
      _costmap_interpolator->Evaluate(interp_pos[0] - (T)0.5, interp_pos[1] - (T)0.5, &value);
      
      r += (T)weight * value * value;  // objective function value
    }
    else {
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
        auto ccpt_world = (transform*ccpt).template block<2, 1>(0, 0);
        Eigen::Matrix<T, 2, 1> interp_pos = (ccpt_world - _costmap_origin.template cast<T>()) / (T)_costmap_resolution;
        T value;
        _costmap_interpolator->Evaluate(interp_pos[0] - (T)0.5, interp_pos[1] - (T)0.5, &value);

        r += (T)weight * (T)_params.cost_check_points[i+2] * value * value;
      }
    }
  }

  const Eigen::Vector2d _original_pos;
  double _next_to_last_length_ratio;
  bool _reversing;
  SmootherParams _params;
  double _costmap_weight;
  Eigen::Vector2d _costmap_origin;
  double _costmap_resolution;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> _costmap_interpolator;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_CERES_COSTAWARE_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
