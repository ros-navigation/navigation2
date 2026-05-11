// Copyright (c) 2026 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLE_BYPASS_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLE_BYPASS_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ObstacleBypassCritic
 * @brief Critic objective function for steering around dynamic obstacles
 * blocking the path. Uses the costmap to determine which side of the
 * obstacle is best to bypass and how far to offset from the path.
 */
class ObstacleBypassCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  /**
   * @brief Determine the best side and distance to bypass an obstacle using costmap
   * @param path_x X position on path at obstacle
   * @param path_y Y position on path at obstacle
   * @param path_yaw Yaw of path tangent at obstacle
   * @param[out] signed_offset Signed offset distance (+ left, - right)
   * @return true if a valid bypass target was found
   */
  bool determineBestBypassSide(
    float path_x, float path_y, float path_yaw, float & signed_offset);

  size_t offset_from_furthest_{0};
  float threshold_to_consider_{0};
  float min_distance_occupancy_check_{0};
  float max_path_occupancy_ratio_{0};
  float bypass_offset_dist_{0};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLE_BYPASS_CRITIC_HPP_
