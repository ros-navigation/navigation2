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

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include <exception>

namespace nav2_costmap_2d
{

CostmapFilter::CostmapFilter()
: is_rolling_(true), costmap_filter_info_topic_("/costmap_filter_info")
{
}

CostmapFilter::~CostmapFilter()
{
}

void CostmapFilter::onInitialize()
{
  // Get parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("costmap_filter_info_topic");

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  if (
    node_->get_parameter(name_ + "." + "costmap_filter_info_topic", costmap_filter_info_topic_) ==
    rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    RCLCPP_ERROR(node_->get_logger(), "costmap_filter_info_topic parameter is not set");
    throw std::runtime_error("Parameter is not set");
  }

  // Allocate and set the costmap of current layer
  matchSize();
  // Ask if costmap window is rolling
  is_rolling_ = layered_costmap_->isRolling();
}

void CostmapFilter::activate()
{
  initializeFilter(costmap_filter_info_topic_);
}

void CostmapFilter::deactivate()
{
  resetFilter();
}

void CostmapFilter::reset()
{
  resetMaps();
  resetFilter();
  initializeFilter(costmap_filter_info_topic_);
}

void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  // If window is rolling, it is required to dynamically update costmap origin of
  // current layer
  if (is_rolling_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  latest_robot_x = robot_x;
  latest_robot_y = robot_y;
  latest_robot_yaw = robot_yaw;
}

void CostmapFilter::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // Clear costmap_ for process()
  resetMap(min_i, min_j, max_i, max_j);

  process(
    master_grid, min_i, min_j, max_i, max_j,
    latest_robot_x, latest_robot_y, latest_robot_yaw);
}

}  // namespace nav2_costmap_2d
