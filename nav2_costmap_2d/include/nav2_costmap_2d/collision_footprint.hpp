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

#ifndef NAV2_COSTMAP_2D__COLLISION_FOOTPRINT_HPP_
#define NAV2_COSTMAP_2D__COLLISION_FOOTPRINT_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/collision_base.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/robot_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_costmap_2d
{
class CollisionFootprint : public CollisionBase
{
public:
  CollisionFootprint(std::shared_ptr<Costmap2D> costmap);
  double footprintCostWithPose(double x, double y, double theta, const Footprint footprint);
  void worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) override;
  double pointCost(int x, int y) const override;

private:
  std::shared_ptr<Costmap2D> costmap_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COLLISION_FOOTPRINT_HPP_
