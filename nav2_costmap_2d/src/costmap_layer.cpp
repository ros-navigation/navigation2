/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include <nav2_costmap_2d/costmap_layer.hpp>
#include <stdexcept>
#include <algorithm>

namespace nav2_costmap_2d
{

void CostmapLayer::touch(
  double x, double y, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  *min_x = std::min(x, *min_x);
  *min_y = std::min(y, *min_y);
  *max_x = std::max(x, *max_x);
  *max_y = std::max(y, *max_y);
}

void CostmapLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void CostmapLayer::clearArea(int start_x, int start_y, int end_x, int end_y, bool invert)
{
  current_ = false;
  unsigned char * grid = getCharMap();
  for (int x = 0; x < static_cast<int>(getSizeInCellsX()); x++) {
    bool xrange = x > start_x && x < end_x;

    for (int y = 0; y < static_cast<int>(getSizeInCellsY()); y++) {
      if ((xrange && y > start_y && y < end_y) == invert) {
        continue;
      }
      int index = getIndex(x, y);
      if (grid[index] != NO_INFORMATION) {
        grid[index] = NO_INFORMATION;
      }
    }
  }
}

void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
  extra_min_x_ = std::min(mx0, extra_min_x_);
  extra_max_x_ = std::max(mx1, extra_max_x_);
  extra_min_y_ = std::min(my0, extra_min_y_);
  extra_max_y_ = std::max(my1, extra_max_y_);
  has_extra_bounds_ = true;
}

void CostmapLayer::useExtraBounds(double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_extra_bounds_) {
    return;
  }

  *min_x = std::min(extra_min_x_, *min_x);
  *min_y = std::min(extra_min_y_, *min_y);
  *max_x = std::max(extra_max_x_, *max_x);
  *max_y = std::max(extra_max_y_, *max_y);
  extra_min_x_ = 1e6;
  extra_min_y_ = 1e6;
  extra_max_x_ = -1e6;
  extra_max_y_ = -1e6;
  has_extra_bounds_ = false;
}

void CostmapLayer::updateWithMax(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it]) {
        master_array[it] = costmap_[it];
      }
      it++;
    }
  }
}

void CostmapLayer::updateWithMaxWithoutUnknownOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost != NO_INFORMATION && old_cost < costmap_[it]) {
        master_array[it] = costmap_[it];
      }
      it++;
    }
  }
}

void CostmapLayer::updateWithTrueOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  if (costmap_ == nullptr) {
    throw std::runtime_error("Can't update costmap layer: It has't been initialized yet!");
  }

  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithOverwrite(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] != NO_INFORMATION) {
        master[it] = costmap_[it];
      }
      it++;
    }
  }
}

void CostmapLayer::updateWithAddition(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) {
        master_array[it] = costmap_[it];
      } else {
        int sum = old_cost + costmap_[it];
        if (sum >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          master_array[it] = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        } else {
          master_array[it] = sum;
        }
      }
      it++;
    }
  }
}

CombinationMethod CostmapLayer::combination_method_from_int(const int value)
{
  switch (value) {
    case 0:
      return CombinationMethod::Overwrite;
    case 1:
      return CombinationMethod::Max;
    case 2:
      return CombinationMethod::MaxWithoutUnknownOverwrite;
    default:
      RCLCPP_WARN(
        logger_,
        "Param combination_method: %i. Possible values are  0 (Overwrite) or 1 (Maximum) or "
        "2 (Maximum without overwriting the master's NO_INFORMATION values)."
        "The default value 1 will be used", value);
      return CombinationMethod::Max;
  }
}
}  // namespace nav2_costmap_2d
