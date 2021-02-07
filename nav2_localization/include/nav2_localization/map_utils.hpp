// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef NAV2_LOCALIZATION__MAP_UTILS_HPP_
#define NAV2_LOCALIZATION__MAP_UTILS_HPP_

#include <cstdint>
#include <utility>
#include "nav_msgs/msg/map_meta_data.hpp"

// TODO(unassigned): should this be moved to nav2_util?
namespace nav2_localization
{
/**
 * @class MapUtils
 * @brief useful methods for occupancy grid maps
 */
class MapUtils
{
public:
  /**
   * @brief Converts world coordinates into grid map coordinates
   * @param x World x coordinate
   * @param y World y coordinate
   * @param map_info Grid map metadata
   * @return The corresponding grid map coordinates as a pair
   */
  static std::pair<int, int> worldToMapCoord(
    const double & x,
    const double & y,
    const nav_msgs::msg::MapMetaData & map_info);

  /**
   * @brief Returns the index of a point in the map
   * @param x X coordinate of the cell (width)
   * @param y Y coordinate of the cell (height)
   * @param map_width Width of the map (in cells)
   * @return Index of ("x", "y") in the map
   */
  static int mapCoordToIndex(const uint32_t &x, const uint32_t &y, const uint32_t &map_width);

  /**
   * @brief Returns the coordinates of an index in the map
   * @param index 
   * @param map_width Width of the map (in cells)
   * @return The coordinates corresponding to the given index
   */
  static std::pair<uint32_t, uint32_t> mapIndexToCoord(const int &index, const uint32_t &map_width);

  /**
   * @brief Returns the euclidean distance between two points
   * @param x1 x corrdinate of point 1
   * @param y1 y corrdinate of point 1
   * @param x2 x corrdinate of point 2
   * @param y2 y corrdinate of point 2
   * @return The euclidean distance between (x1, y1) and (x2, y2)
   */
  static double distanceBetweenTwoPoints(
    const int & x1, const int & y1,
    const int & x2, const int & y2);
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__MAP_UTILS_HPP_
