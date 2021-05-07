// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__SMOOTHER_HPP_
#define NAV2_SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_smac_planner/types.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param min_turning_radius Minimum turning radius (m)
   */
  void initialize(const double & min_turning_radius)
  {

  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time)
  {
    return false;
  }
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMOOTHER_HPP_
