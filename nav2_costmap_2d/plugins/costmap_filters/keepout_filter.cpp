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
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"

namespace nav2_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: costmap_filter_info_sub_(nullptr), map_filter_sub_(nullptr), map_filter_(nullptr)
{
}

void KeepoutFilter::initializeFilter(
  const std::string costmap_filter_info_topic)
{
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    node_->get_logger(),
    "Setting a subscriber to %s costmap filter info",
    costmap_filter_info_topic.c_str());
  costmap_filter_info_sub_ = node_->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    costmap_filter_info_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::costmapFilterInfoCallback, this, std::placeholders::_1));

  curr_time_ = node_->now();
  prev_time_ = curr_time_;
}

void KeepoutFilter::costmapFilterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  // Resetting previous subscriber each time when new costmap filter information arrives
  if (map_filter_sub_) {
    RCLCPP_WARN(node_->get_logger(), "New map filter info arrived. Replacing old one.");
    map_filter_sub_.reset();
  }
  map_filter_.reset();

  // Setting new map filter subscriber
  RCLCPP_INFO(
    node_->get_logger(),
    "Setting a subscriber to %s map filter",
    msg->map_filter_topic.c_str());
  map_filter_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    msg->map_filter_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::mapFilterCallback, this, std::placeholders::_1));
}

void KeepoutFilter::mapFilterCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (map_filter_) {
    RCLCPP_WARN(node_->get_logger(), "New map filter arrived. Replacing old one.");
    map_filter_.reset();
  }
  map_filter_ = std::make_unique<nav2_util::OccupancyGrid>(*msg);
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/)
{
  if (!map_filter_) {
    curr_time_ = node_->now();
    // Show warning message every 2 seconds to not litter an output
    if (curr_time_ - prev_time_ >= rclcpp::Duration(std::chrono::milliseconds(2000))) {
      RCLCPP_WARN(node_->get_logger(), "Map filter was not received");
      prev_time_ = curr_time_;
    }
    return;
  }

  unsigned int min_i_u, min_j_u, max_i_u, max_j_u;  // Unsigned versions of window bounds
  double wx, wy;  // world coordinates

  // unsigned<-signed conversions.
  // All map bounds are positive, so this will be safe.
  min_i_u = (unsigned int)min_i;
  min_j_u = (unsigned int)min_j;
  max_i_u = (unsigned int)max_i;
  max_j_u = (unsigned int)max_j;

  // Optimization: iterate only by map-filter window bounds corresponding to
  // (min_i, min_j)..(max_i, max_j) window.
  // ToDo: after costmap rotation will be added, this should be re-worked.
  int mf_tmp_x, mf_tmp_y;  // Temp signed map_filter_ coordinates using in calculations
  unsigned int mf_min_x_u, mf_min_y_u, mf_max_x_u, mf_max_y_u;  // map_filter_ bounds

  // Calculating map_filter_ bounds corresponding to (min_i, min_j) corner
  mapToWorld(min_i_u, min_j_u, wx, wy);
  map_filter_->worldToMapNoBounds(wx, wy, mf_tmp_x, mf_tmp_y);
  mf_tmp_x = std::max(mf_tmp_x, 0);
  mf_tmp_y = std::max(mf_tmp_y, 0);
  mf_min_x_u = (unsigned int)mf_tmp_x;  // unsigned<-signed conversion
  mf_min_y_u = (unsigned int)mf_tmp_y;  // unsigned<-signed conversion
  // Checking that mf_min_x_u/mf_min_y_u are not out of map_filter_ bounds
  if (mf_min_x_u > map_filter_->getSizeInCellsX() || mf_min_y_u > map_filter_->getSizeInCellsY()) {
    return;
  }

  // Calculating map_filter_ bounds corresponding to (max_i, max_j) corner
  mapToWorld(max_i_u, max_j_u, wx, wy);
  map_filter_->worldToMapNoBounds(wx, wy, mf_tmp_x, mf_tmp_y);
  // Checking that max_x/max_y are not out of map_filter_ bounds
  if (mf_tmp_x < 0 || mf_tmp_y < 0) {
    return;
  }
  mf_max_x_u = (unsigned int)mf_tmp_x;  // unsigned<-signed conversion
  mf_max_y_u = (unsigned int)mf_tmp_y;  // unsigned<-signed conversion
  mf_max_x_u = std::min(mf_max_x_u, map_filter_->getSizeInCellsX());
  mf_max_y_u = std::min(mf_max_y_u, map_filter_->getSizeInCellsY());

  unsigned int i, j;  // map_filter_ iterators
  unsigned int mx, my;  // costmap_ coordinates
  unsigned char data;  // map_filter_ element data

  for (i = mf_min_x_u; i < mf_max_x_u; i++) {
    for (j = mf_min_y_u; j < mf_max_y_u; j++) {
      data = (*map_filter_)[map_filter_->getIndex(i, j)];
      if (data != nav2_util::OCC_GRID_UNKNOWN) {
        // Converting each point on map_filter to world coordinates
        map_filter_->mapToWorld(i, j, wx, wy);
        // Getting coordinates of costmap_ (if any) corresponding to given world coordinates
        bool on_costmap = worldToMap(wx, wy, mx, my);
        // If these coordinates are belonging to costmap bounds put data on layer
        if (on_costmap && mx >= min_i_u && mx < max_i_u && my >= min_j_u && my < max_j_u) {
          // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
          // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
          costmap_[getIndex(mx, my)] = std::round(
            data * (LETHAL_OBSTACLE - FREE_SPACE) /
            (nav2_util::OCC_GRID_OCCUPIED - nav2_util::OCC_GRID_FREE));
        }
      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void KeepoutFilter::resetFilter()
{
  costmap_filter_info_sub_.reset();
  map_filter_sub_.reset();
  map_filter_.reset();
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
