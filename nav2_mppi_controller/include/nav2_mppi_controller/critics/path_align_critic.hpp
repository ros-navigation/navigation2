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

#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{
/**
 * @class mppi::critics::ConstraintCritic
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
   * @brief Score trajectories using arc-length matching
   */
  void scoreArcLength(CriticData & data, std::vector<bool> & path_pts_valid);

  /**
   * @brief Score trajectories using geometric distance to path segments
   */
  void scoreGeometric(CriticData & data, std::vector<bool> & path_pts_valid);

  /**
   * @brief Update cached path data when path changes
   */
  void updatePathCache(const models::Path & path, size_t path_segments_count);

  /**
   * @brief Compute minimum distance from point to any path segment
   * @param px Point x coordinate
   * @param py Point y coordinate
   * @param closest_seg_idx In/out: hint for search, updated to closest segment
   * @return Distance to closest segment
   */
  float computeMinDistanceToPath(float px, float py, Eigen::Index & closest_seg_idx);

  /**
   * @brief Compute squared distance from point to a specific path segment
   * @param px Point x coordinate
   * @param py Point y coordinate
   * @param seg_idx Segment index
   * @return Squared distance to segment
   */
  float distSqToSegment(float px, float py, Eigen::Index seg_idx);

  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_geometric_alignment_{false};
  bool score_arc_length_{true};
  double search_window_{2.0};
  bool use_path_orientations_{false};
  unsigned int power_{0};
  float weight_{0};

  // Caching variables
  size_t path_size_cache_{0};
  Eigen::ArrayXf path_x_cache_;
  Eigen::ArrayXf path_y_cache_;
  Eigen::ArrayXf segment_dx_;
  Eigen::ArrayXf segment_dy_;
  Eigen::ArrayXf segment_len_sq_;
  Eigen::ArrayXf segment_lengths_;
  Eigen::ArrayXf cumulative_distances_;
  Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> closest_indices_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_