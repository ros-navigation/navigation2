/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"

#include "nav2_map_server/map_mode.hpp"

namespace nav2_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: semantic_info_sub_(nullptr), map_filter_sub_(nullptr), map_filter_(nullptr)
{
}

void KeepoutFilter::loadFilter(
  const std::string semantic_info_topic)
{
  // Setting new semantic info for costmap filter subscriber
  semantic_info_sub_ = node_->create_subscription<nav2_msgs::msg::CostmapFilterSemanticInfo>(
    semantic_info_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::semanticInfoCallback, this, std::placeholders::_1));
}

void KeepoutFilter::semanticInfoCallback(
  const nav2_msgs::msg::CostmapFilterSemanticInfo::SharedPtr msg)
{
  // Resetting previous subscriber each time when new semantic information arrives
  if (map_filter_sub_) {
    map_filter_sub_.reset();
  }
  map_filter_ = nullptr;

  // Setting new map filter subscriber
  map_filter_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    msg->map_filter_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::mapFilterCallback, this, std::placeholders::_1));
}

void KeepoutFilter::mapFilterCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_filter_ = msg;

  // Updating costmap_filter_
  unsigned int filter_cell_sz_x = map_filter_->info.width;
  unsigned int filter_cell_sz_y = map_filter_->info.height;
  double filter_map_res = map_filter_->info.resolution;
  double filter_origin_x = map_filter_->info.origin.position.x;
  double filter_origin_y = map_filter_->info.origin.position.y;
  // ToDo - current Costmap2D does not support orientation
  costmap_filter_ = std::make_unique<Costmap2D>(
    filter_cell_sz_x, filter_cell_sz_y, filter_map_res, filter_origin_x, filter_origin_y);

  // Fill the costmap_filter_ with values from map_filter_
  unsigned int mx, my, it;
  int8_t data;
  for (mx = 0; mx < filter_cell_sz_x; mx++) {
    for (my = 0; my < filter_cell_sz_y; my++) {
      it = costmap_filter_->getIndex(mx, my);
      data = map_filter_->data[it];
      if (data == nav2_map_server::OCC_GRID_OCCUPIED) {
        costmap_filter_->setCost(mx, my, LETHAL_OBSTACLE);
      } else {
        costmap_filter_->setCost(mx, my, NO_INFORMATION);
      }
    }
  }
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/)
{
  if (!map_filter_) {
    RCLCPP_WARN(node_->get_logger(), "Map filter was not received");
    return;
  }

  unsigned int i, j;  // costmap_filter_ iterators
  double wx, wy;  // world coordinates
  unsigned int mx, my;  // costmap_ coordinates

  unsigned char * costmap_filter_charmap = costmap_filter_->getCharMap();

  for (i = 0; i < costmap_filter_->getSizeInCellsX(); i++) {
    for (j = 0; j < costmap_filter_->getSizeInCellsY(); j++) {
      if (costmap_filter_charmap[costmap_filter_->getIndex(i, j)] == LETHAL_OBSTACLE) {
        // Converting each point on costmap_filter to world coordinates
        costmap_filter_->mapToWorld(i, j, wx, wy);
        // Getting coordinates of costmap_ (if any) corresponding to given
        // world coordinates
        bool on_costmap = master_grid.worldToMap(wx, wy, mx, my);
        // If these coordinates are belonging to costmap bounds put LETHAL_OBSTACLE
        // on layer
        if (on_costmap &&
          mx >= (unsigned int)min_i && mx < (unsigned int)max_i &&
          my >= (unsigned int)min_j && my < (unsigned int)max_j)
        {
          costmap_[master_grid.getIndex(mx, my)] = LETHAL_OBSTACLE;
        }
      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void KeepoutFilter::unloadFilter()
{
  if (semantic_info_sub_) {
    semantic_info_sub_.reset();
  }
  if (map_filter_sub_) {
    map_filter_sub_.reset();
  }
  map_filter_ = nullptr;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
