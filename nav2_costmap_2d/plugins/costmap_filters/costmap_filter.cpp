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

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include <exception>

namespace nav2_costmap_2d
{

CostmapFilter::CostmapFilter()
: semantic_info_topic_("/semantic_info")
{
}

CostmapFilter::~CostmapFilter()
{
}

void CostmapFilter::onInitialize()
{
  costmap_ = nullptr;
  costmap_size_x = 0;
  costmap_size_y = 0;

  // Get parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("semantic_info_topic", rclcpp::ParameterValue("/semantic_info"));

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(name_ + "." + "semantic_info_topic", semantic_info_topic_);
}

void CostmapFilter::activate()
{
  loadFilter(semantic_info_topic_);
}

void CostmapFilter::deactivate()
{
  unloadFilter();
}

void CostmapFilter::reset()
{
  if (costmap_) {
    delete[] costmap_;
  }
  costmap_ = nullptr;
  costmap_size_x = 0;
  costmap_size_y = 0;

  unloadFilter();
  loadFilter(semantic_info_topic_);
}

void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  latest_robot_x = robot_x;
  latest_robot_y = robot_y;
  latest_robot_yaw = robot_yaw;
}

void CostmapFilter::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // (Re)Allocate costmap_ for using in costmap filters
  reAllocateCostmap(master_grid);

  // Clear costmap_ for process()
  clearCostmap(min_i, min_j, max_i, max_j);

  process(
    master_grid, min_i, min_j, max_i, max_j,
    latest_robot_x, latest_robot_y, latest_robot_yaw);
}

void CostmapFilter::reAllocateCostmap(
  const nav2_costmap_2d::Costmap2D & master_grid)
{
  int new_costmap_size_x = master_grid.getSizeInCellsX();
  int new_costmap_size_y = master_grid.getSizeInCellsY();

  // (Re)allocate costmap if it does not exist of its size was changed
  if (!costmap_ || new_costmap_size_x != costmap_size_x || new_costmap_size_y != costmap_size_y) {
    try {
      delete[] costmap_;
      size_t costmap_size = new_costmap_size_x * new_costmap_size_y * sizeof(unsigned char);
      costmap_ = new unsigned char[costmap_size];
    } catch (std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Can not (re)allocate costmap: %s", ex.what());
      throw;
    }
    costmap_size_x = new_costmap_size_x;
    costmap_size_y = new_costmap_size_y;
  }
}

void CostmapFilter::clearCostmap(
  int min_i, int min_j, int max_i, int max_j)
{
  int len_x = max_i - min_i;
  for (int it = min_j * costmap_size_x + min_i;
    it < max_j * costmap_size_x + min_i;
    it += costmap_size_x)
  {
    memset(costmap_ + it, NO_INFORMATION, len_x * sizeof(unsigned char));
  }
}

void CostmapFilter::matchSize()
{
  // When map size was changed it is required to re-allocate costmap layer
  Costmap2D * master_grid = layered_costmap_->getCostmap();
  reAllocateCostmap(*master_grid);
}

}  // namespace nav2_costmap_2d
