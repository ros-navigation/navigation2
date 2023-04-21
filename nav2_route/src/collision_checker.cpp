// Copyright (c) 2023 Joshua Wallace
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

#include "nav2_route/collision_checker.hpp"

#include <iostream>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_route
{
CollisionChecker::CollisionChecker(nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
}

bool CollisionChecker::inCollision(const unsigned int & i, const bool traverse_unknown)
{
  unsigned char cost = costmap_->getCost(i);

  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    (cost == nav2_costmap_2d::NO_INFORMATION && !traverse_unknown) )
  {
    std::cout << "In collision" << std::endl;
    return true;
  }
  return false;
}
nav2_costmap_2d::Costmap2D * CollisionChecker::getCostmap()
{
  return costmap_;
}

}  // namespace nav2_route
