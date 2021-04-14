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

#ifndef NAV2_SMAC_PLANNER__TYPES_HPP_
#define NAV2_SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <utility>
#include <string>

namespace nav2_smac_planner
{

typedef std::pair<float, unsigned int> NodeHeuristicPair;

/**
 * @struct nav2_smac_planner::SearchInfo
 * @brief Search properties and penalties
 */
struct SearchInfo
{
  float minimum_turning_radius;
  float non_straight_penalty;
  float change_penalty;
  float reverse_penalty;
  float cost_penalty;
  float analytic_expansion_ratio;
  std::string lattice_filepath;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__TYPES_HPP_
