// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_UTIL__COSTMAP_HPP_
#define NAV2_UTIL__COSTMAP_HPP_

#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_util
{

enum class TestCostmap
{
  open_space,
  bounded,
  bottom_left_obstacle,
  top_left_obstacle,
  maze1,
  maze2
};

/**
 * @class nav2_util::Costmap
 * @brief Class for a single layered costmap initialized from an
 * occupancy grid representing the map.
 */
class Costmap
{
public:
  typedef uint8_t CostValue;

  /**
   * @brief A constructor for nav2_util::Costmap
   * @param node Ptr to a node
   * @param trinary_costmap Whether the costmap should be trinary
   * @param track_unknown_space Whether to track unknown space in costmap
   * @param lethal_threshold The lethal space cost threshold to use
   * @param unknown_cost_value Internal costmap cell value for unknown space
   */
  Costmap(
    rclcpp::Node * node, bool trinary_costmap = true, bool track_unknown_space = true,
    int lethal_threshold = 100, int unknown_cost_value = -1);
  Costmap() = delete;
  ~Costmap();

  /**
   * @brief Set the static map of this costmap
   * @param occupancy_grid Occupancy grid to populate this costmap with
   */
  void set_static_map(const nav_msgs::msg::OccupancyGrid & occupancy_grid);

  /**
   * @brief Set the test costmap type of this costmap
   * @param testCostmapType Type of stored costmap to use
   */
  void set_test_costmap(const TestCostmap & testCostmapType);

  /**
   * @brief Get a costmap message from this object
   * @param specifications Parameters of costmap
   * @return Costmap msg of this costmap
   */
  nav2_msgs::msg::Costmap get_costmap(const nav2_msgs::msg::CostmapMetaData & specifications);

  /**
   * @brief Get a metadata message from this object
   * @return Costmap metadata of this costmap
   */
  nav2_msgs::msg::CostmapMetaData get_properties() {return costmap_properties_;}

  /**
   * @brief Get whether some coordinates are free
   * @return bool if free
   */
  bool is_free(const unsigned int x_coordinate, const unsigned int y_coordinate) const;

  /**
   * @brief Get whether some index in the costmap is free
   * @return bool if free
   */
  bool is_free(const unsigned int index) const;

  // Mapping for often used cost values
  static const CostValue no_information;
  static const CostValue lethal_obstacle;
  static const CostValue inscribed_inflated_obstacle;
  static const CostValue medium_cost;
  static const CostValue free_space;

private:
  /**
   * @brief Get data from the test
   * @return data
   */
  std::vector<uint8_t> get_test_data(const TestCostmap configuration);

  /**
   * @brief Get the interpreted value in the costmap
   * @return uint value
   */
  uint8_t interpret_value(const int8_t value) const;

  // Costmap isn't itself a node
  rclcpp::Node * node_;

  // TODO(orduno): For now, only holding costs from static map
  nav2_msgs::msg::CostmapMetaData costmap_properties_;
  std::vector<uint8_t> costs_;

  // Static layer parameters
  bool trinary_costmap_;
  bool track_unknown_space_;
  int lethal_threshold_;
  int unknown_cost_value_;

  // Flags to determine the origin of the costmap
  bool map_provided_;
  bool using_test_map_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__COSTMAP_HPP_
