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

#ifndef NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_

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
#include "nav2_constrained_smoother/options.hpp"
#include "nav2_constrained_smoother/utils.hpp"

namespace nav2_constrained_smoother
{

/**
 * @struct nav2_constrained_smoother::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, distance from original and obstacle avoidance.
 */
class SmootherCostFunction
{
public:
  /**
   * @brief A constructor for nav2_constrained_smoother::SmootherCostFunction
   * @param original_path Original position of the path node
   * @param next_to_last_length_ratio Ratio of next path segment compared to previous.
   *  Negative if one of them represents reversing motion.
   * @param reversing Whether the path segment after this node represents reversing motion.
   * @param costmap A costmap to get values for collision and obstacle avoidance
   * @param params Optimization weights and parameters
   * @param costmap_weight Costmap cost weight. Can be params.costmap_weight or params.cusp_costmap_weight
   */
  SmootherCostFunction(
    const Eigen::Vector2d & original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
    const SmootherParams & params,
    double costmap_weight)
  : original_pos_(original_pos),
    next_to_last_length_ratio_(next_to_last_length_ratio),
    reversing_(reversing),
    params_(params),
    costmap_weight_(costmap_weight),
    costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
    costmap_resolution_(costmap->getResolution()),
    costmap_interpolator_(costmap_interpolator)
  {
  }

  ceres::CostFunction * AutoDiff()
  {
    return new ceres::AutoDiffCostFunction<SmootherCostFunction, 4, 2, 2, 2>(this);
  }

  void setCostmapWeight(double costmap_weight)
  {
    costmap_weight_ = costmap_weight;
  }

  double getCostmapWeight()
  {
    return costmap_weight_;
  }

  /**
   * @brief Smoother cost function evaluation
   * @param pt X, Y coords of current point
   * @param pt_next X, Y coords of next point
   * @param pt_prev X, Y coords of previous point
   * @param pt_residual array of output residuals (smoothing, curvature, distance, cost)
   * @return if successful in computing values
   */
  template<typename T>
  bool operator()(
    const T * const pt, const T * const pt_next, const T * const pt_prev,
    T * pt_residual) const
  {
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi(pt);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(pt_next);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(pt_prev);
    Eigen::Map<Eigen::Matrix<T, 4, 1>> residual(pt_residual);
    residual.setZero();

    // compute cost
    addSmoothingResidual<T>(params_.smooth_weight, xi, xi_next, xi_prev, residual[0]);
    addCurvatureResidual<T>(params_.curvature_weight, xi, xi_next, xi_prev, residual[1]);
    addDistanceResidual<T>(
      params_.distance_weight, xi,
      original_pos_.template cast<T>(), residual[2]);
    addCostResidual<T>(costmap_weight_, xi, xi_next, xi_prev, residual[3]);

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  template<typename T>
  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    Eigen::Matrix<T, 2, 1> d_next = pt_next - pt;
    Eigen::Matrix<T, 2, 1> d_prev = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d_diff = next_to_last_length_ratio_ * d_next - d_prev;
    r += (T)weight * d_diff.dot(d_diff);    // objective function value
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt_next Point Xi+1 for calculating Xi's cost
   * @param pt_prev Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   */
  template<typename T>
  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    Eigen::Matrix<T, 2, 1> center = arcCenter(
      pt_prev, pt, pt_next,
      next_to_last_length_ratio_ < 0);
    if (CERES_ISINF(center[0])) {
      return;
    }
    T turning_rad = (pt - center).norm();
    T ki_minus_kmax = (T)1.0 / turning_rad - params_.max_curvature;

    if (ki_minus_kmax <= (T)EPSILON) {
      return;
    }

    r += (T)weight * ki_minus_kmax * ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template<typename T>
  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & xi,
    const Eigen::Matrix<T, 2, 1> & xi_original,
    T & r) const
  {
    r += (T)weight * (xi - xi_original).squaredNorm();  // objective function value
  }

  /**
   * @brief Cost function term for steering away from costs
   * @param weight Weight to apply to function
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param r Residual (cost) of term
   */
  template<typename T>
  inline void addCostResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next,
    const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    if (params_.cost_check_points.empty()) {
      Eigen::Matrix<T, 2, 1> interp_pos =
        (pt - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;
      T value;
      costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);
      r += (T)weight * value * value;  // objective function value
    } else {
      Eigen::Matrix<T, 2, 1> dir = tangentDir(
        pt_prev, pt, pt_next,
        next_to_last_length_ratio_ < 0);
      dir.normalize();
      if (((pt_next - pt).dot(dir) < (T)0) != reversing_) {
        dir = -dir;
      }
      Eigen::Matrix<T, 3, 3> transform;
      transform << dir[0], -dir[1], pt[0],
        dir[1], dir[0], pt[1],
        (T)0, (T)0, (T)1;
      for (size_t i = 0; i < params_.cost_check_points.size(); i += 3) {
        Eigen::Matrix<T, 3, 1> ccpt((T)params_.cost_check_points[i],
          (T)params_.cost_check_points[i + 1], (T)1);
        auto ccpt_world = (transform * ccpt).template block<2, 1>(0, 0);
        Eigen::Matrix<T, 2,
          1> interp_pos = (ccpt_world - costmap_origin_.template cast<T>()) /
          (T)costmap_resolution_;
        T value;
        costmap_interpolator_->Evaluate(interp_pos[1] - (T)0.5, interp_pos[0] - (T)0.5, &value);

        r += (T)weight * (T)params_.cost_check_points[i + 2] * value * value;
      }
    }
  }

  const Eigen::Vector2d original_pos_;
  double next_to_last_length_ratio_;
  bool reversing_;
  SmootherParams params_;
  double costmap_weight_;
  Eigen::Vector2d costmap_origin_;
  double costmap_resolution_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> costmap_interpolator_;
};

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__SMOOTHER_COST_FUNCTION_HPP_
