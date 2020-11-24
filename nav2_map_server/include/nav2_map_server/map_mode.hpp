// Copyright (c) 2019 Rover Robotics
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
// limitations under the License.

#ifndef NAV2_MAP_SERVER__MAP_MODE_HPP_
#define NAV2_MAP_SERVER__MAP_MODE_HPP_

#include <string>
#include <vector>
namespace nav2_map_server
{
/**
 * @enum nav2_map_server::MapMode
 * @brief Describes the relation between image pixel values and map occupancy
 * status (0-100; -1). Lightness refers to the mean of a given pixel's RGB
 * channels on a scale from 0 to 255.
 */
enum class MapMode
{
  /**
   * Together with associated threshold values (occupied and free):
   *   lightness >= occupied threshold - Occupied (100)
   *             ... (anything in between) - Unknown (-1)
   *    lightness <= free threshold - Free (0)
   */
  Trinary,
  /**
   * Together with associated threshold values (occupied and free):
   *   alpha < 1.0 - Unknown (-1)
   *   lightness >= occ_th - Occupied (100)
   *             ... (linearly interpolate to)
   *   lightness <= free_th - Free (0)
   */
  Scale,
  /**
   * Lightness = 0 - Free (0)
   *          ... (linearly interpolate to)
   * Lightness = 100 - Occupied (100)
   * Lightness >= 101 - Unknown
   */
  Raw,
};

/**
 * @brief Convert a MapMode enum to the name of the map mode
 * @param map_mode Mode for the map
 * @return String identifier of the given map mode
 * @throw std::invalid_argument if the given value is not a defined map mode
 */
const char * map_mode_to_string(MapMode map_mode);

/**
 * @brief Convert the name of a map mode to a MapMode enum
 * @param map_mode_name Name of the map mode
 * @throw std::invalid_argument if the name does not name a map mode
 * @return map mode corresponding to the string
 */
MapMode map_mode_from_string(std::string map_mode_name);
}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_MODE_HPP_
