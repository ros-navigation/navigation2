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
: filter_info_sub_(nullptr), mask_sub_(nullptr), mask_costmap_(nullptr)
{
}

void KeepoutFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    node_->get_logger(),
    "Subscribing to \"%s\" topic for filter info...",
    filter_info_topic.c_str());
  filter_info_sub_ = node_->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));
}

void KeepoutFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // Resetting previous subscriber each time when new costmap filter information arrives
  if (mask_sub_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "New costmap filter info arrived from %s topic. Updating old filter info.",
      filter_info_topic_.c_str());
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != BASE_DEFAULT or msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "base and multiplier in ConstmapFilterInfo should be set to their default values (%d and %d)",
      BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  mask_topic_ = msg->map_mask_topic;

  // Setting new map mask subscriber
  RCLCPP_INFO(
    node_->get_logger(),
    "Subscribing to \"%s\" topic for map mask...",
    msg->map_mask_topic.c_str());
  mask_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "New map mask arrived from %s topic. Updating old map mask.",
      mask_topic_.c_str());
    mask_costmap_.reset();
  }

  // Making a new mask_costmap_
  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & /*pose*/)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()), 2000,
      "Map mask was not received");
    return;
  }

  double wx, wy;  // world coordinates

  // Optimization: iterate only in overlapped
  // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
  //
  //           mask_costmap_
  //       *----------------------------*
  //       |                            |
  //       |                            |
  //       |      (2)                   |
  // *-----+-------*                    |
  // |     |///////|<- overlapped area  |
  // |     |///////|   to iterate in    |
  // |     *-------+--------------------*
  // |    (1)      |
  // |             |
  // *-------------*
  //  master_grid (min_i, min_j)..(max_i, max_j) window
  //
  // ToDo: after costmap rotation will be added, this should be re-worked.

  // Calculating bounds corresponding to bottom-left overlapping (1) corner
  int mg_min_x, mg_min_y;  // masger_grid indexes of bottom-left (1) corner
  // mask_costmap_ -> master_grid intexes conversion
  const double half_cell_size = 0.5 * mask_costmap_->getResolution();
  wx = mask_costmap_->getOriginX() + half_cell_size;
  wy = mask_costmap_->getOriginY() + half_cell_size;
  master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
  // Calculation of (1) corner bounds
  if (mg_min_x >= max_i || mg_min_y >= max_j) {
    // There is no overlapping. Do nothing.
    return;
  }
  mg_min_x = std::max(min_i, mg_min_x);
  mg_min_y = std::max(min_j, mg_min_y);

  // Calculating bounds corresponding to top-right window (2) corner
  int mg_max_x, mg_max_y;  // masger_grid indexes of top-right (2) corner
  // mask_costmap_ -> master_grid intexes conversion
  wx = mask_costmap_->getOriginX() + mask_costmap_->getSizeInMetersX();
  wy = mask_costmap_->getOriginY() + mask_costmap_->getSizeInMetersY();
  master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
  // Calculation of (2) corner bounds
  if (mg_max_x < min_i || mg_max_y < min_j) {
    // There is no overlapping. Do nothing.
    return;
  }
  mg_max_x = std::min(max_i, mg_max_x);
  mg_max_y = std::min(max_j, mg_max_y);

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;  // master_grid iterators
  unsigned int index;  // corresponding index of master_grid
  unsigned int mx, my;  // mask_costmap_ coordinates
  unsigned char data, old_data;  // master_grid element data

  // Main master_grid updating loop
  // Iterate in overlapped window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_
      master_grid.mapToWorld(i, j, wx, wy);
      if (mask_costmap_->worldToMap(wx, wy, mx, my)) {
        data = mask_costmap_->getCost(mx, my);
        // Update if mask_ data is valid and greater than existing master_grid's one
        if (data == NO_INFORMATION) {
          continue;
        }
        if (data > old_data || old_data == NO_INFORMATION) {
          master_array[index] = data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)
