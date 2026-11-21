// Copyright (c) 2026 Origin
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

#ifndef NAV2_BEHAVIOR_TREE__UTILS__FOOTPRINT_UTILS_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__FOOTPRINT_UTILS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree::utils
{

// Clipper 6.4.2 (c) Angus Johnson 2010-2017, Boost Software License 1.0: boost.org/LICENSE_1_0.txt
/**
 * @brief Mitered outward offset that keeps edge i parallel to footprint edge i
 * @param footprint Footprint polygon, either winding
 * @param delta Offset distance in metres
 * @return Inflated footprint, empty unless convex, simple and with no corner under 60 degrees
 */
inline std::vector<geometry_msgs::msg::Point> inflateFootprint(
  const std::vector<geometry_msgs::msg::Point> & footprint, double delta)
{
  const size_t len = footprint.size();
  if (len < 3) {
    return {};
  }
  if (std::fabs(delta) < 1.0e-20) {
    return footprint;
  }

  // Clipper's Area(), positive for counter-clockwise
  double area = 0.0;
  for (size_t i = 0, j = len - 1; i < len; j = i++) {
    area += (footprint[j].x + footprint[i].x) * (footprint[j].y - footprint[i].y);
  }
  area = -area * 0.5;

  // Negate delta for clockwise instead of reversing, so edges keep their indices
  if (area < 0.0) {
    delta = -delta;
  }

  // GetUnitNormal() of each edge
  struct Normal
  {
    double x{0.0};
    double y{0.0};
  };
  std::vector<Normal> normals(len);
  for (size_t j = 0; j < len; ++j) {
    const auto & pt1 = footprint[j];
    const auto & pt2 = footprint[(j + 1) % len];
    double dx = pt2.x - pt1.x;
    double dy = pt2.y - pt1.y;
    // Clipper dropped repeated points, so the old code skipped such footprints
    if (std::hypot(dx, dy) < 1.0e-6) {
      return {};
    }
    const double f = 1.0 / std::sqrt(dx * dx + dy * dy);
    dx *= f;
    dy *= f;
    normals[j] = {dy, -dx};
  }

  constexpr double kMiterLimit = 0.5;  // Clipper's MiterLimit of 2.0
  constexpr double kCollinearTolerance = 1.0e-6;  // One Clipper unit at a 1e6 scale

  std::vector<geometry_msgs::msg::Point> inflated;
  inflated.reserve(len);
  double total_turn = 0.0;
  size_t k = len - 1;
  for (size_t j = 0; j < len; k = j++) {
    // OffsetPoint(): sine and cosine between the normals of edges k and j
    const double sin_a = normals[k].x * normals[j].y - normals[j].x * normals[k].y;
    const double cos_a = normals[k].x * normals[j].x + normals[j].y * normals[k].y;
    total_turn += std::atan2(sin_a, cos_a);

    geometry_msgs::msg::Point point;
    if (std::fabs(sin_a * delta) < kCollinearTolerance && cos_a > 0) {
      // Collinear edges, offset along the shared normal
      point.x = footprint[j].x + normals[k].x * delta;
      point.y = footprint[j].y + normals[k].y * delta;
      inflated.push_back(point);
      continue;
    }

    // Concave or over the miter limit, where Clipper would add vertices
    const double r = 1 + (normals[j].x * normals[k].x + normals[j].y * normals[k].y);
    if (sin_a * delta < 0 || r < kMiterLimit) {
      return {};
    }

    // DoMiter()
    const double q = delta / r;
    point.x = footprint[j].x + (normals[k].x + normals[j].x) * q;
    point.y = footprint[j].y + (normals[k].y + normals[j].y) * q;
    inflated.push_back(point);
  }

  // Turns adding up to more than one full turn mean the footprint crosses itself
  if (std::fabs(std::fabs(total_turn) - 2.0 * M_PI) > 1.0e-6) {
    return {};
  }
  return inflated;
}

/**
 * @brief Opposite edge of each edge, by casting a ray from its midpoint through the centroid
 * @param footprint Footprint polygon
 * @return Opposite edge index per edge, or the edge count if none was found
 */
inline std::vector<size_t> buildOppositePairs(
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  const size_t n = footprint.size();
  std::vector<size_t> opposite(n, n);
  if (n == 0) {
    return opposite;
  }

  // Centroid
  double cx = 0.0, cy = 0.0;
  for (const auto & pt : footprint) {
    cx += pt.x;
    cy += pt.y;
  }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);

  for (size_t i = 0; i < n; ++i) {
    const size_t i_next = (i + 1) % n;

    // Ray from the midpoint of edge i through the centroid
    const double mx = (footprint[i].x + footprint[i_next].x) * 0.5;
    const double my = (footprint[i].y + footprint[i_next].y) * 0.5;
    const double dx = cx - mx;
    const double dy = cy - my;

    double best_t = 1e18;
    size_t best_edge = n;

    for (size_t j = 0; j < n; ++j) {
      if (j == i) {
        continue;
      }
      const size_t j_next = (j + 1) % n;

      // Intersect the ray with edge j
      const double ex = footprint[j_next].x - footprint[j].x;
      const double ey = footprint[j_next].y - footprint[j].y;

      const double denom = dx * ey - dy * ex;
      if (std::abs(denom) < 1e-12) {
        continue;  // parallel
      }

      const double t = ((footprint[j].x - mx) * ey - (footprint[j].y - my) * ex) / denom;
      const double s = ((footprint[j].x - mx) * dy - (footprint[j].y - my) * dx) / denom;

      // First hit past the centroid (t > 1) that lies on the edge (s in [0, 1])
      if (t > 1.0 && s >= 0.0 && s <= 1.0 && t < best_t) {
        best_t = t;
        best_edge = j;
      }
    }

    opposite[i] = best_edge;
  }

  return opposite;
}

/**
 * @brief Whether two footprints differ in vertex count or position
 * @param a First footprint
 * @param b Second footprint
 * @return true if the footprints differ
 */
inline bool footprintChanged(
  const std::vector<geometry_msgs::msg::Point> & a,
  const std::vector<geometry_msgs::msg::Point> & b)
{
  constexpr double kFootprintEpsilon = 1e-9;
  if (a.size() != b.size()) {
    return true;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::fabs(a[i].x - b[i].x) > kFootprintEpsilon ||
      std::fabs(a[i].y - b[i].y) > kFootprintEpsilon)
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief Undo the robot pose a costmap places its published footprint at
 * @param footprint_msg Footprint published by a costmap
 * @param tf TF buffer
 * @param robot_base_frame Robot base frame
 * @param transform_tolerance TF lookup timeout in seconds
 * @param footprint Output footprint in the robot frame
 * @return false if the robot pose lookup fails
 */
inline bool footprintToRobotFrame(
  const geometry_msgs::msg::PolygonStamped & footprint_msg,
  tf2_ros::Buffer & tf, const std::string & robot_base_frame, double transform_tolerance,
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, tf, footprint_msg.header.frame_id, robot_base_frame,
      transform_tolerance, footprint_msg.header.stamp))
  {
    return false;
  }

  std::vector<geometry_msgs::msg::Point> published;
  published.reserve(footprint_msg.polygon.points.size());
  for (const auto & point : footprint_msg.polygon.points) {
    published.push_back(nav2_costmap_2d::toPoint(point));
  }

  const double x = robot_pose.pose.position.x;
  const double y = robot_pose.pose.position.y;
  const double theta = tf2::getYaw(robot_pose.pose.orientation);

  std::vector<geometry_msgs::msg::Point> translated;
  nav2_costmap_2d::transformFootprint(-x, -y, 0, published, translated);
  nav2_costmap_2d::transformFootprint(0, 0, -theta, translated, footprint);
  return true;
}

}  // namespace nav2_behavior_tree::utils

#endif  // NAV2_BEHAVIOR_TREE__UTILS__FOOTPRINT_UTILS_HPP_
