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

#ifndef NAV2_COSTMAP_2D__KEEPOUT_FILTER_HPP_
#define NAV2_COSTMAP_2D__KEEPOUT_FILTER_HPP_

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "nav2_msgs/msg/costmap_filter_semantic_info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <memory>

namespace nav2_costmap_2d
{

class KeepoutFilter : public CostmapFilter
{
public:
  KeepoutFilter();

  void loadFilter(
    const std::string semantic_info_topic);

  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    double robot_x, double robot_y, double robot_yaw);

  void unloadFilter();

private:
  void semanticInfoCallback(const nav2_msgs::msg::CostmapFilterSemanticInfo::SharedPtr msg);
  void mapFilterCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterSemanticInfo>::SharedPtr semantic_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_filter_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_filter_;

  std::unique_ptr<Costmap2D> costmap_filter_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__KEEPOUT_FILTER_HPP_
