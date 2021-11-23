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

#ifndef NAV2_CERES_COSTAWARE_SMOOTHER__UTILS_HPP_
#define NAV2_CERES_COSTAWARE_SMOOTHER__UTILS_HPP_

#include "Eigen/Core"

#define EPSILON 0.0001

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

  // circle center is at the intersection of mirror axes of the segments: http://paulbourke.net/geometry/circlesphere/
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
    }
    
    return Eigen::Matrix<T, 2, 1>(p2[0] - p1[0], p2[1] - p1[1]);
  }
  
  return Eigen::Matrix<T, 2, 1>(center[1] - p[1], p[0] - center[0]); // prependicular to (p - center)
}

}  // namespace nav2_ceres_costaware_smoother

#endif  // NAV2_CERES_COSTAWARE_SMOOTHER__UTILS_HPP_