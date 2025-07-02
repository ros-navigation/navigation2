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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_

#include <cstddef>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mppi::critics
{

/**
 * @class PathHugCritic
 * @brief  Penalises trajectories that drift away from the centre-line of the
 *         global plan, encouraging the controller to “hug” the path.
 *
 * Tunables (declare / read in initialize()):
 *   • weight   — linear weight applied to the distance metric
 *   • power    — exponent; 1.0 → linear, 2.0 → quadratic penalty
 */
class PathHugCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  double  power_  {1.0};
  double  weight_ {1.0};
  size_t  closest_path_idx_ {0};   ///< latch for iterative local search

private:
  nav_msgs::msg::Path cached_path_; ///< copy of the last global plan
  unsigned int last_path_size_ {0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_HUG_CRITIC_HPP_
