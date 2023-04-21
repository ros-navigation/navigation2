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

#ifndef NAV2_ROUTE__COLLISION_CHECKER_HPP_
#define NAV2_ROUTE__COLLISION_CHECKER_HPP_

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_route
{

class CollisionChecker
{
public:
  explicit CollisionChecker(nav2_costmap_2d::Costmap2D * costmap);

  bool inCollision(const unsigned int & i, const bool traverse_unknown);

  nav2_costmap_2d::Costmap2D * getCostmap();

private:
  nav2_costmap_2d::Costmap2D * costmap_;
};
}  // namespace nav2_route

#endif  // NAV2_ROUTE__COLLISION_CHECKER_HPP_
