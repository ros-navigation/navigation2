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

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_costmap_2d
{
/**
 * @class SpeedFilter
 * @brief Reads in a speed restriction mask and enables a robot to
 * dynamically adjust speed based on pose in map to slow in dangerous
 * areas. Done via absolute speed setting or percentage of maximum speed
 */
class SpeedFilter : public CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  SpeedFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic);

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter();

  /**
   * @brief If this filter is active
   */
  bool isActive();

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  nav2::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  nav2::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string global_frame_;  // Frame of current layer (master_grid)

  double base_, multiplier_;
  bool percentage_;
  double speed_limit_, speed_limit_prev_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_
