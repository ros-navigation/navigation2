// Copyright (c) 2023 Samsung R&D Institute Russia
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

#ifndef NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_
#define NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_

#include <uuid/uuid.h>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_map_server
{

// ---------- Working with UUID-s ----------

/**
 * @brief Converts UUID from input array to unparsed string
 * @param uuid Input UUID in array format
 * @return Unparsed UUID string
 */
inline std::string unparseUUID(const unsigned char * uuid)
{
  char uuid_str[37];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

// ---------- Working with shapes' overlays ----------

/// @brief Type of overlay between different vector objects and map
enum class OverlayType : uint8_t
{
  OVERLAY_SEQ = 0,  // Vector objects are superimposed in the order in which they have arrived
  OVERLAY_MAX = 1,  // Maximum value from vector objects and map is being chosen
  OVERLAY_MIN = 2   // Minimum value from vector objects and map is being chosen
};

/**
 * @brief Updates map value with shape's one according to the given overlay type
 * @param map_val Map value. To be updated with new value if overlay is involved
 * @param shape_val Vector object value to be overlaid on map
 * @param overlay_type Type of overlay
 * @throw std::exception in case of unknown overlay type
 */
inline void processVal(
  int8_t & map_val, const int8_t shape_val,
  const OverlayType overlay_type)
{
  switch (overlay_type) {
    case OverlayType::OVERLAY_SEQ:
      map_val = shape_val;
      return;
    case OverlayType::OVERLAY_MAX:
      if (shape_val > map_val) {
        map_val = shape_val;
      }
      return;
    case OverlayType::OVERLAY_MIN:
      if ((map_val == nav2_util::OCC_GRID_UNKNOWN || shape_val < map_val) &&
        shape_val != nav2_util::OCC_GRID_UNKNOWN)
      {
        map_val = shape_val;
      }
      return;
    default:
      throw std::runtime_error{"Unknown overlay type"};
  }
}

/**
 * @brief Updates the cell on the map with given shape value according to the given overlay type
 * @param map Output map to be updated with
 * @param offset Offset to the cell to be updated
 * @param shape_val Vector object value to be updated map with
 * @param overlay_type Type of overlay
 */
inline void processCell(
  nav_msgs::msg::OccupancyGrid::SharedPtr map,
  const unsigned int offset,
  const int8_t shape_val,
  const OverlayType overlay_type)
{
  int8_t map_val = map->data[offset];
  processVal(map_val, shape_val, overlay_type);
  map->data[offset] = map_val;
}

/// @brief Functor class used in raytraceLine algorithm
class MapAction
{
public:
  /**
   * @brief MapAction constructor
   * @param map Pointer to output map
   * @param value Value to put on map
   * @param overlay_type Overlay type
   */
  MapAction(
    nav_msgs::msg::OccupancyGrid::SharedPtr map,
    int8_t value, OverlayType overlay_type)
  : map_(map), value_(value), overlay_type_(overlay_type)
  {}

  /**
   * @brief Map' cell updating operator
   * @param offset Offset on the map where the cell to be changed
   */
  inline void operator()(unsigned int offset)
  {
    processCell(map_, offset, value_, overlay_type_);
  }

protected:
  /// @brief Output map pointer
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  /// @brief Value to put on map
  int8_t value_;
  /// @brief Overlay type
  OverlayType overlay_type_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_
