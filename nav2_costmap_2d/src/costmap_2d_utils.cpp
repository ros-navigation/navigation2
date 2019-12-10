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

#include "nav2_costmap_2d/costmap_2d_utils.hpp"

namespace nav2_costmap_2d
{

CostTranslationTable::CostTranslationTable()
{
  costs = new char[256];

  // special values:
  costs[0] = 0;  // NO obstacle
  costs[253] = 99;  // INSCRIBED obstacle
  costs[254] = 100;  // LETHAL obstacle
  costs[255] = -1;  // UNKNOWN

  // regular cost values scale the range 1 to 252 (inclusive) to fit
  // into 1 to 98 (inclusive).
  for (int i = 1; i < 253; i++) {
    costs[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }
}

CostTranslationTable cost_translation_table;

nav2_msgs::msg::Costmap
toCostmapMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  const rclcpp::Time & timestamp)
{
  double resolution = costmap->getResolution();

  double wx, wy;
  costmap->mapToWorld(0, 0, wx, wy);

  unsigned char * data = costmap->getCharMap();

  nav2_msgs::msg::Costmap costmap_msg;
  costmap_msg.header.frame_id = global_frame;
  costmap_msg.header.stamp = timestamp;
  costmap_msg.metadata.layer = "master";
  costmap_msg.metadata.resolution = resolution;
  costmap_msg.metadata.size_x = costmap->getSizeInCellsX();
  costmap_msg.metadata.size_y = costmap->getSizeInCellsY();
  costmap_msg.metadata.origin.position.x = wx - resolution / 2;
  costmap_msg.metadata.origin.position.y = wy - resolution / 2;
  costmap_msg.metadata.origin.position.z = 0.0;
  costmap_msg.metadata.origin.orientation.w = 1.0;
  costmap_msg.data.resize(costmap_msg.metadata.size_x * costmap_msg.metadata.size_y);

  for (unsigned int i = 0; i < costmap_msg.data.size(); i++) {
    costmap_msg.data[i] = data[i];
  }

  return costmap_msg;
}

nav_msgs::msg::OccupancyGrid
toOccupancyGridMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  const rclcpp::Time & timestamp)
{
  double resolution = costmap->getResolution();

  double wx, wy;
  costmap->mapToWorld(0, 0, wx, wy);

  unsigned char * data = costmap->getCharMap();

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = global_frame;
  grid_msg.header.stamp = timestamp;
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = costmap->getSizeInCellsX();
  grid_msg.info.height = costmap->getSizeInCellsY();
  grid_msg.info.origin.position.x = wx - resolution / 2;
  grid_msg.info.origin.position.y = wy - resolution / 2;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);

  for (unsigned int i = 0; i < grid_msg.data.size(); i++) {
    grid_msg.data[i] = cost_translation_table.costs[data[i]];
  }

  return grid_msg;
}

map_msgs::msg::OccupancyGridUpdate
toOccupancyGridUpdateMsg(
  const Costmap2D * costmap,
  const std::string & global_frame,
  unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn,
  const rclcpp::Time & timestamp)
{
  map_msgs::msg::OccupancyGridUpdate grid_update;
  grid_update.header.stamp = timestamp;
  grid_update.header.frame_id = global_frame;
  grid_update.x = x0;
  grid_update.y = y0;
  grid_update.width = xn - x0;
  grid_update.height = yn - y0;
  grid_update.data.resize(grid_update.width * grid_update.height);
  unsigned int i = 0;
  for (unsigned int y = y0; y < yn; y++) {
    for (unsigned int x = x0; x < xn; x++) {
      unsigned char cost = costmap->getCost(x, y);
      grid_update.data[i++] = cost_translation_table.costs[cost];
    }
  }

  return grid_update;
}

}  // namespace nav2_costmap_2d
