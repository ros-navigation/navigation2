// Copyright (c) 2019 Intel Corporation
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
//
// Modified by: Shivang Patel (shivaang14@gmail.com)

#ifndef NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
#define NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_costmap_2d
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

template<typename CostmapT>
class FootprintCollisionChecker
{
public:
  FootprintCollisionChecker();
  explicit FootprintCollisionChecker(CostmapT costmap);
  double footprintCost(const Footprint footprint);
  double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);
  double lineCost(int x0, int x1, int y0, int y1) const;
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  double pointCost(int x, int y) const;
  void setCostmap(CostmapT costmap);

protected:
  CostmapT costmap_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
