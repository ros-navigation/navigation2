// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include <algorithm>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::PathAlignCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 */
class PathAlignCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
   * @brief Update cached path data when path changes
   *
   * Precomputes and caches path segment geometry including segment vectors,
   * lengths, and cumulative arc-length distances. Only recomputes when the path
   * actually changes (detected via size and key point checks).
   *
   * @param path Reference path to cache
   * @param path_segments_count Number of segments to consider from path start
   */
  void updatePathCache(const models::Path & path, size_t path_segments_count);

  /**
   * @brief Score trajectories using arc-length matching along path
   *
   * Integrates distance traveled along each trajectory and matches it to equivalent
   * arc-length positions on the reference path. Penalizes Euclidean distance between
   * trajectory points and their corresponding path points. Optionally includes
   * orientation difference when use_path_orientations_ is enabled.
   *
   * @param data Critic data containing trajectories and path information
   * @param path_pts_valid Validity flags for each path point (collision-free or not)
   */
  void scoreArcLength(CriticData & data, std::vector<bool> & path_pts_valid);

  /**
   * @brief Score trajectories using geometric distance to path segments
   *
   * Samples points along each trajectory and computes their perpendicular distance
   * to the nearest path segment. Uses a hint-based search within a configurable
   * window for efficiency. Only scores points within lookahead_distance_ of the
   * trajectory start.
   *
   * @param data Critic data containing trajectories and path information
   * @param path_pts_valid Validity flags for each path point (collision-free or not)
   */
  void scoreGeometric(CriticData & data, std::vector<bool> & path_pts_valid);

  /**
   * @brief Compute minimum distance from point to any path segment
   *
   * Uses a search window around the hint index to find the nearest segment,
   * exploiting path locality. The window is defined in arc-length distance
   * along the path. Updates the hint for subsequent calls.
   *
   * @param px Point x coordinate
   * @param py Point y coordinate
   * @param closest_seg_idx [in/out] Search hint, updated to closest segment found
   * @return Euclidean distance to closest segment
   */
  float computeMinDistanceToPath(float px, float py, Eigen::Index & closest_seg_idx);

  /**
   * @brief Compute squared distance from a point to a line segment
   *
   * Projects the point onto the segment (clamped to segment bounds) and returns
   * the squared distance. Uses precomputed segment geometry for efficiency.
   *
   * @param px X-coordinate of the query point
   * @param py Y-coordinate of the query point
   * @param seg_idx Index of the path segment
   * @return Squared distance from point to segment
   *
   * @note Returns squared distance to avoid sqrt() computation
   * @note Handles degenerate (zero-length) segments by returning distance to start point
   */
  inline float distSqToSegment(float px, float py, Eigen::Index seg_idx) const
  {
    const float dpx = px - path_x_cache_(seg_idx);
    const float dpy = py - path_y_cache_(seg_idx);

    const float inv_len_sq = segment_inv_len_sq_(seg_idx);

    if (inv_len_sq < 1e-6f) [[unlikely]] {
      return dpx * dpx + dpy * dpy;
    }

    const float dx = segment_dx_(seg_idx);
    const float dy = segment_dy_(seg_idx);
    const float t = std::clamp((dpx * dx + dpy * dy) * inv_len_sq, 0.0f, 1.0f);

    const float dist_x = dpx - t * dx;
    const float dist_y = dpy - t * dy;

    return dist_x * dist_x + dist_y * dist_y;
  }

  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{false};
  bool score_arc_length_{true};
  bool use_geometric_alignment_;
  double search_window_;
  float lookahead_distance_;
  float early_termination_distance_;

  unsigned int power_{0};
  float weight_{0};
  float arc_length_weight_{0};
  float geometric_weight_{0};

  // Caching variables
  size_t path_size_cache_{0};
  Eigen::ArrayXf path_x_cache_;
  Eigen::ArrayXf path_y_cache_;
  Eigen::ArrayXf segment_dx_;
  Eigen::ArrayXf segment_dy_;
  Eigen::ArrayXf segment_len_sq_;
  Eigen::ArrayXf segment_inv_len_sq_;
  Eigen::ArrayXf segment_lengths_;
  Eigen::ArrayXf cumulative_distances_;
  Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> closest_indices_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
