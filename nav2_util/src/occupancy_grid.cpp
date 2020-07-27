// Copyright (c) 2020 Samsung Research Russia
// Copyright (c) 2008, 2013, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the <ORGANIZATION> nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Alexey Merzlyakov
//         Eitan Marder-Eppstein
//         David V. Lu!!

#include "nav2_util/occupancy_grid.hpp"

#include <memory>

namespace nav2_util
{

OccupancyGrid::OccupancyGrid()
{
  nav_msgs::msg::OccupancyGrid occ_grid;
  occ_grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>(occ_grid);
}

OccupancyGrid::OccupancyGrid(const nav_msgs::msg::OccupancyGrid & occ_grid)
{
  occ_grid_ = std::make_unique<nav_msgs::msg::OccupancyGrid>(occ_grid);
}

OccupancyGrid::~OccupancyGrid()
{
  occ_grid_.reset();
}

unsigned char OccupancyGrid::operator[](const unsigned int index) const
{
  if (index > occ_grid_->data.size()) {
    throw std::runtime_error("Index is out of bounds map data");
  }
  return occ_grid_->data[index];
}

unsigned int OccupancyGrid::getSizeInCellsX() const
{
  return occ_grid_->info.width;
}

unsigned int OccupancyGrid::getSizeInCellsY() const
{
  return occ_grid_->info.height;
}

void OccupancyGrid::mapToWorld(
  unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = occ_grid_->info.origin.position.x + (mx + 0.5) * occ_grid_->info.resolution;
  wy = occ_grid_->info.origin.position.y + (my + 0.5) * occ_grid_->info.resolution;
}

bool OccupancyGrid::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < occ_grid_->info.origin.position.x || wy < occ_grid_->info.origin.position.y) {
    return false;
  }

  mx = static_cast<int>((wx - occ_grid_->info.origin.position.x) / occ_grid_->info.resolution);
  my = static_cast<int>((wy - occ_grid_->info.origin.position.y) / occ_grid_->info.resolution);

  if (mx < occ_grid_->info.width && my < occ_grid_->info.height) {
    return true;
  }

  return false;
}

void OccupancyGrid::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>((wx - occ_grid_->info.origin.position.x) / occ_grid_->info.resolution);
  my = static_cast<int>((wy - occ_grid_->info.origin.position.y) / occ_grid_->info.resolution);
}

}  // namespace nav2_util
