// Copyright (c) 2025
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__SYMMETRIC_GOAL_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__SYMMETRIC_GOAL_ANGLE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::SymmetricGoalAngleCritic
 * @brief Critic objective function for driving towards goal orientation
 * for symmetric robots. Scores trajectories based on the minimum angular
 * distance to either the goal orientation OR the goal orientation + 180°.
 * This allows symmetric robots (that can drive equally well in either direction)
 * to avoid unnecessary 180° rotations at the goal.
 */
class SymmetricGoalAngleCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * considering both the goal orientation and the flipped orientation (goal + 180°).
   * Always uses the minimum distance to either orientation.
   *
   * @param data CriticData containing trajectories and path information
   */
  void score(CriticData & data) override;

protected:
  float threshold_to_consider_{0};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__SYMMETRIC_GOAL_ANGLE_CRITIC_HPP_
