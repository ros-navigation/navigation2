/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcutils/logging_macros.h"

namespace
{
// === helpers ===============================================================

constexpr double kEpsilon = 1e-8;
constexpr double kClosedPathTol = 0.01;  // [m]

inline double
squaredDistance(const geometry_msgs::msg::Point & a,
                const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline double
squaredDistanceToSegmentXY(const geometry_msgs::msg::Point & p,
                           const geometry_msgs::msg::Point & a,
                           const geometry_msgs::msg::Point & b)
{
  const double len2 = squaredDistance(a, b);
  if (len2 < kEpsilon) {
    return squaredDistance(p, a);                     // a == b
  }

  const double dot = (p.x - a.x) * (b.x - a.x) +
                     (p.y - a.y) * (b.y - a.y);
  const double t   = std::clamp(dot / len2, 0.0, 1.0);

  const double proj_x = a.x + t * (b.x - a.x);
  const double proj_y = a.y + t * (b.y - a.y);

  const double dx = p.x - proj_x;
  const double dy = p.y - proj_y;
  return dx * dx + dy * dy;
}

inline bool
isClosed(const nav_msgs::msg::Path & path)
{
  return path.poses.size() > 2 &&
         squaredDistance(path.poses.front().pose.position,
                         path.poses.back().pose.position) <
           kClosedPathTol * kClosedPathTol;
}
}  // namespace

namespace nav2_util
{

/**
 * @brief  Lateral (XY) distance from a robot pose to a path.
 *
 * Implements the “iterative local minimum” strategy requested in
 * https://github.com/ros-navigation/navigation2/issues/5037:
 *   • First invocation (closest_idx == nullptr) -> global scan  
 *   • Subsequent calls start at *closest_idx for loop-stable tracking
 *
 * @param pose         Robot pose in the same frame as @p path
 * @param path         Path to evaluate
 * @param closest_idx  IN/OUT latch index; may be nullptr
 * @return             Shortest distance [m] or +inf for empty path
 *
 * @throws std::invalid_argument if pose/path frame_ids differ
 */
double
distanceFromPath(const geometry_msgs::msg::PoseStamped & pose,
                 const nav_msgs::msg::Path &             path,
                 size_t *                                closest_idx /* = nullptr */)
{
  using std::numeric_limits;

  // ---- sanity -------------------------------------------------------------
  if (pose.header.frame_id != path.header.frame_id) {
    throw std::invalid_argument(
      "distanceFromPath(): pose and path frame_ids differ (" +
      pose.header.frame_id + " vs " + path.header.frame_id + ")");
  }

  // ---- trivial cases ------------------------------------------------------
  if (path.poses.empty()) {
    return numeric_limits<double>::infinity();
  }

  if (path.poses.size() == 1) {
    return std::hypot(
      pose.pose.position.x - path.poses.front().pose.position.x,
      pose.pose.position.y - path.poses.front().pose.position.y);
  }

  // ---- search range -------------------------------------------------------
  const size_t n = path.poses.size();
  size_t start   = 0;

  if (closest_idx && *closest_idx < n - 1) {
    start = *closest_idx;                         // local scan
  }

  double best_sq  = numeric_limits<double>::max();
  size_t best_idx = start;

  // ---- segments i … n-2 ---------------------------------------------------
  for (size_t i = start; i < n - 1; ++i) {
    const double d_sq = squaredDistanceToSegmentXY(
      pose.pose.position,
      path.poses[i].pose.position,
      path.poses[i + 1].pose.position);

    if (d_sq < best_sq) {
      best_sq  = d_sq;
      best_idx = i;
    }
  }

  // ---- seam of closed loop -----------------------------------------------
  if (isClosed(path)) {
    const double d_sq = squaredDistanceToSegmentXY(
      pose.pose.position,
      path.poses.back().pose.position,
      path.poses.front().pose.position);

    if (d_sq < best_sq) {
      best_sq  = d_sq;
      best_idx = n - 1;   // virtual segment “last → first”
    }
  }

  // ---- write-back ---------------------------------------------------------
  if (closest_idx != nullptr) {
    *closest_idx = (best_idx >= n - 1) ? 0 : best_idx;
  }

  return std::sqrt(best_sq);
}

}  // namespace nav2_util
