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

#ifndef NAV2_COSTMAP_2D__COSTMAP_2D_UTILS_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_2D_UTILS_HPP_


#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{
struct CostTranslationTable
{
  CostTranslationTable();
  char * costs;
};

nav2_msgs::msg::Costmap
toCostmapMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  const rclcpp::Time & timestamp = rclcpp::Time());

nav_msgs::msg::OccupancyGrid
toOccupancyGridMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  const rclcpp::Time & timestamp = rclcpp::Time());

map_msgs::msg::OccupancyGridUpdate
toOccupancyGridUpdateMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn,
  const rclcpp::Time & timestamp = rclcpp::Time());

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_UTILS_HPP_
