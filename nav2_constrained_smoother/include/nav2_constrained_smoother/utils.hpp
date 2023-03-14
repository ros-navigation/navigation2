// Copyright (c) 2021 RoboTech Vision
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
// limitations under the License.

#ifndef NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_

#include <limits>
#include "Eigen/Core"

#define EPSILON 0.0001

/**
 * Compatibility with different ceres::isinf() and ceres::IsInfinite() API
 * used in Ceres Solver 2.1.0+ and 2.0.0- versions respectively
 */
#if defined(USE_OLD_CERES_API)
  #define CERES_ISINF(x) ceres::IsInfinite(x)
#else
  #define CERES_ISINF(x) ceres::isinf(x)
#endif

namespace nav2_constrained_smoother
{

/**
 * @brief Center of an arc between three points
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result position of the center or Vector2(inf, inf) for straight lines and 180 deg turns
 */
template<typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
  Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

  if (is_cusp) {
    d2 = -d2;
    pt_next = pt + d2;
  }

  T det = d1[0] * d2[1] - d1[1] * d2[0];
  if (ceres::abs(det) < (T)1e-4) {  // straight line
    return Eigen::Matrix<T, 2, 1>(
      (T)std::numeric_limits<double>::infinity(), (T)std::numeric_limits<double>::infinity());
  }

  // circle center is at the intersection of mirror axes of the segments:
  // http://paulbourke.net/geometry/circlesphere/
  // line intersection:
  // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Intersection%20of%20two%20lines
  Eigen::Matrix<T, 2, 1> mid1 = (pt_prev + pt) / (T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (pt + pt_next) / (T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  T det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0];
  T det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1 * n2[0] - det2 * n1[0]) / det,
    (det1 * n2[1] - det2 * n1[1]) / det);
  return center;
}

/**
 * @brief Direction of a line which contains pt and is tangential to arc
 * between pt_prev, pt, pt_next
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc, lying on the tangential line
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result Tangential line direction.
 * Note: the sign of tangentDir is undefined here, should be assigned in post-process
 * depending on movement direction. Also, for speed reasons, direction vector is not normalized.
 */
template<typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt, pt_next, is_cusp);
  if (CERES_ISINF(center[0])) {  // straight line
    Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

    if (is_cusp) {
      d2 = -d2;
      pt_next = pt + d2;
    }

    Eigen::Matrix<T, 2, 1> result(pt_next[0] - pt_prev[0], pt_next[1] - pt_prev[1]);
    if (result[0] == 0.0 && result[1] == 0.0) {  // a very rare edge situation
      return Eigen::Matrix<T, 2, 1>(d1[1], -d1[0]);
    }
    return result;
  }

  // tangent is prependicular to (pt - center)
  // Note: not determining + or - direction here, this should be handled at the caller side
  return Eigen::Matrix<T, 2, 1>(center[1] - pt[1], pt[0] - center[0]);
}

}  // namespace nav2_constrained_smoother

#endif  // NAV2_CONSTRAINED_SMOOTHER__UTILS_HPP_
