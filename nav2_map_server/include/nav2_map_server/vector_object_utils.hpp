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
#include <string>
#include <stdexcept>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_map_server
{

// ---------- Working with UUID-s ----------

/**
 * @beirf Converts input UUID from input array to unparsed string
 * @param uuid Input UUID in array format
 * @return Unparsed UUID string
 */
inline std::string unparseUUID(const unsigned char * uuid)
{
  char uuid_str[37];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}

// ---------- Working with ROS-parameters ----------

/**
 * @brief Declares and obtains ROS-parameter from given node
 * @param node LifecycleNode pointer where the parameter belongs to
 * @param param_name Parameter name string
 * @param default_val Default value of the parameter (in case if parameter is not set)
 * @return Obtained parameter value
 */
template<typename ValT>
inline rclcpp::Parameter getParameter(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::string & param_name,
  const ValT & default_val)
{
  nav2_util::declare_parameter_if_not_declared(
    node, param_name, rclcpp::ParameterValue(default_val));
  return node->get_parameter(param_name);
}

/**
 * @brief Declares and obtains ROS-parameter from given node
 * @param node LifecycleNode pointer where the parameter belongs to
 * @param param_name Parameter name string
 * @param val_type Type of obtained parameter
 * @return Obtained parameter value
 * @throw std::exception if parameter is not set
 */
template<>
inline rclcpp::Parameter getParameter<rclcpp::ParameterType>(
  nav2_util::LifecycleNode::SharedPtr node,
  const std::string & param_name,
  const rclcpp::ParameterType & val_type)
{
  nav2_util::declare_parameter_if_not_declared(
    node, param_name, val_type);
  return node->get_parameter(param_name);
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
 * @param map_val Map value. To be updated with new value if overlay is required
 * @param shape_val Vector object value to be overlayed on map
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
 * @brief Fill the cell on the map with given shape value according to the given overlay type
 * @param map Output map to be filled with
 * @param offset Offset to the cell to be filled
 * @param shape_val Vector object value to be overlayed on map
 * @param overlay_type Type of overlay
 */
inline void fillMap(
  nav_msgs::msg::OccupancyGrid::SharedPtr map,
  const unsigned int offset,
  const int8_t shape_val,
  const OverlayType overlay_type)
{
  int8_t map_val = map->data[offset];
  processVal(map_val, shape_val, overlay_type);
  map->data[offset] = map_val;
}

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__VECTOR_OBJECT_UTILS_HPP_
