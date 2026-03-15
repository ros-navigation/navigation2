// Copyright (c) 2025 Berkan Tali
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

class PathHugCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  void updateCumulativeDistances(const models::Path & path, size_t num_segments);

  float computeMinDistanceToPath(
    float px, float py,
    const models::Path & path,
    size_t num_segments,
    Eigen::Index & path_hint,
    int & segments_searched);

  unsigned int power_{1};
  float weight_{10.0f};
  float max_path_occupancy_ratio_{0.07f};
  size_t offset_from_furthest_{20};
  int trajectory_point_step_{4};
  float threshold_to_consider_{0.5f};
  float search_window_{0.15f};
  float lookahead_distance_{0.3f};

  // Path caching
  size_t prev_path_size_{0};
  float prev_path_start_x_{0.0f};
  float prev_path_start_y_{0.0f};
  float prev_path_end_x_{0.0f};
  float prev_path_end_y_{0.0f};

  Eigen::Index cached_robot_hint_{0};
  std::vector<float> cumulative_distances_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
