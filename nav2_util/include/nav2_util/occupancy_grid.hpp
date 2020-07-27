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

#ifndef NAV2_UTIL__OCCUPANCY_GRID_HPP_
#define NAV2_UTIL__OCCUPANCY_GRID_HPP_

#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_util
{

/**
  * @brief OccupancyGrid data constants
  */
static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

/**
  * @brief OccupancyGrid wrapper class having all necessary API to work with
  * (similar to Costmap2D API)
  */
class OccupancyGrid
{
public:
  /**
   * @brief  Default constructor. Creates empty nav_msgs::msg::OccupancyGrid.
   */
  OccupancyGrid();

  /**
   * @brief  Constructor with one argument. Copies nav_msgs::msg::OccupancyGrid
   * from input argument to occ_grid_ pointer.
   * @param  nav_msgs::msg::OccupancyGrid to copy
   */
  explicit OccupancyGrid(const nav_msgs::msg::OccupancyGrid & occ_grid);

  /**
   * @brief  Destructor. Releases occ_grid_ pointer.
   */
  ~OccupancyGrid();

  /**
   * @brief  Giving element of map
   * @param  Index of required element
   * @return data[index]
   * @throw  std::runtime_error if index is out of bounds data
   */
  unsigned char operator[](const unsigned int index) const;

  /**
   * @brief  Accessor for the x size of the costmap in cells
   * @return The x size of the map
   */
  unsigned int getSizeInCellsX() const;

  /**
   * @brief  Accessor for the y size of the costmap in cells
   * @return The y size of the map
   */
  unsigned int getSizeInCellsY() const;

  /**
   * @brief  Given two map coordinates... compute the associated index
   * @param mx The x coordinate
   * @param my The y coordinate
   * @return The associated index
   */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * occ_grid_->info.width + mx;
  }

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  mx The x map coordinate
   * @param  my The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates <b>are not guaranteed to lie within the map.</b>
   */
  void worldToMapNoBounds(double wx, double wy, int & mx, int & my) const;

private:
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> occ_grid_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__OCCUPANCY_GRID_HPP_
