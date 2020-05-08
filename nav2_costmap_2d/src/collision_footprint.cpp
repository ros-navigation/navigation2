// Copyright (c) 2020 Shivang Patel
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

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_costmap_2d/collision_footprint.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/exceptions.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d
{

CollisionFootprint::CollisionFootprint(std::shared_ptr<Costmap2D> costmap)
: costmap_(costmap)
{}

double CollisionFootprint::pointCost(int x, int y) const
{
  return costmap_->getCost(x, y);
}

void CollisionFootprint::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  costmap_->worldToMap(wx, wy, mx, my);
}

double CollisionFootprint::footprintCostWithPose(
  double x, double y, double theta,
  const Footprint footprint)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (footprint[i].x * cos_th - footprint[i].y * sin_th);
    new_pt.y = y + (footprint[i].x * sin_th + footprint[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(oriented_footprint);
}

}  // namespace nav2_costmap_2d
