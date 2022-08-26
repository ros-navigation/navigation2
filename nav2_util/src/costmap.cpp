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

#include <vector>
#include <algorithm>
#include "nav2_util/costmap.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_util/geometry_utils.hpp"

using std::vector;

namespace nav2_util
{
using nav2_util::geometry_utils::orientationAroundZAxis;

const Costmap::CostValue Costmap::no_information = 255;
const Costmap::CostValue Costmap::lethal_obstacle = 254;
const Costmap::CostValue Costmap::inscribed_inflated_obstacle = 253;
const Costmap::CostValue Costmap::medium_cost = 128;
const Costmap::CostValue Costmap::free_space = 0;

// TODO(orduno): Port ROS1 Costmap package
Costmap::Costmap(
  rclcpp::Node * node, bool trinary_costmap, bool track_unknown_space,
  int lethal_threshold, int unknown_cost_value)
: node_(node), trinary_costmap_(trinary_costmap), track_unknown_space_(track_unknown_space),
  lethal_threshold_(lethal_threshold), unknown_cost_value_(unknown_cost_value)
{
  if (lethal_threshold_ < 0. || lethal_threshold_ > 100.) {
    RCLCPP_WARN(
      node_->get_logger(), "Costmap: Lethal threshold set to %d, it should be within"
      " bounds 0-100. This could result in potential collisions!", lethal_threshold_);
    // lethal_threshold_ = std::max(std::min(lethal_threshold_, 100), 0);
  }
}

Costmap::~Costmap()
{
}

void Costmap::set_static_map(const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  RCLCPP_INFO(node_->get_logger(), "Costmap: Setting static costmap");

  costmap_properties_.map_load_time = node_->now();
  costmap_properties_.update_time = node_->now();
  costmap_properties_.layer = "Master";

  // Store the properties of the occupancy grid
  costmap_properties_.resolution = occupancy_grid.info.resolution;
  costmap_properties_.size_x = occupancy_grid.info.width;
  costmap_properties_.size_y = occupancy_grid.info.height;
  costmap_properties_.origin = occupancy_grid.info.origin;

  uint32_t size_x = costmap_properties_.size_x;
  uint32_t size_y = costmap_properties_.size_y;

  costs_.resize(size_x * size_y);

  // TODO(orduno): for now just doing a direct mapping of values from the original static map
  //               i.e. no cell inflation, etc.
  std::vector<int8_t> static_map_cell_values = occupancy_grid.data;

  unsigned int index = 0;
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = static_map_cell_values[index];
      costs_[index] = interpret_value(value);
      ++index;
    }
  }

  map_provided_ = true;
}

void Costmap::set_test_costmap(const TestCostmap & testCostmapType)
{
  costmap_properties_.map_load_time = node_->now();
  costmap_properties_.update_time = node_->now();
  costmap_properties_.layer = "master";
  costmap_properties_.resolution = 1;
  costmap_properties_.size_x = 10;
  costmap_properties_.size_y = 10;
  costmap_properties_.origin.position.x = 0.0;
  costmap_properties_.origin.position.y = 0.0;
  costmap_properties_.origin.position.z = 0.0;

  // Define map rotation
  // Provided as yaw with counterclockwise rotation, with yaw = 0 meaning no rotation
  costmap_properties_.origin.orientation = orientationAroundZAxis(0.0);

  costs_ = get_test_data(testCostmapType);

  using_test_map_ = true;
}

nav2_msgs::msg::Costmap Costmap::get_costmap(
  const nav2_msgs::msg::CostmapMetaData & /*specifications*/)
{
  if (!map_provided_ && !using_test_map_) {
    throw std::runtime_error("Costmap has not been set.");
  }

  // TODO(orduno): build a costmap given the specifications
  //               for now using the specs of the static map

  nav2_msgs::msg::Costmap costmap;

  costmap.header.stamp = node_->now();
  costmap.header.frame_id = "map";
  costmap.metadata = costmap_properties_;
  costmap.data = costs_;

  return costmap;
}

vector<uint8_t> Costmap::get_test_data(const TestCostmap testCostmapType)
{
  // TODO(orduno): alternatively use a mathematical function

  const uint8_t n = no_information;
  const uint8_t x = lethal_obstacle;
  const uint8_t i = inscribed_inflated_obstacle;
  const uint8_t u = medium_cost;
  const uint8_t o = free_space;

  vector<uint8_t> costmapFree =
    // 0  1  2  3  4  5  6  7  8  9
  {o, o, o, o, o, o, o, o, o, o,     // 0
    o, o, o, o, o, o, o, o, o, o,    // 1
    o, o, o, o, o, o, o, o, o, o,    // 2
    o, o, o, o, o, o, o, o, o, o,    // 3
    o, o, o, o, o, o, o, o, o, o,    // 4
    o, o, o, o, o, o, o, o, o, o,    // 5
    o, o, o, o, o, o, o, o, o, o,    // 6
    o, o, o, o, o, o, o, o, o, o,    // 7
    o, o, o, o, o, o, o, o, o, o,    // 8
    o, o, o, o, o, o, o, o, o, o};   // 9

  vector<uint8_t> costmapBounded =
    // 0  1  2  3  4  5  6  7  8  9
  {n, n, n, n, n, n, n, n, n, n,     // 0
    n, o, o, o, o, o, o, o, o, n,    // 1
    n, o, o, o, o, o, o, o, o, n,    // 2
    n, o, o, o, o, o, o, o, o, n,    // 3
    n, o, o, o, o, o, o, o, o, n,    // 4
    n, o, o, o, o, o, o, o, o, n,    // 5
    n, o, o, o, o, o, o, o, o, n,    // 6
    n, o, o, o, o, o, o, o, o, n,    // 7
    n, o, o, o, o, o, o, o, o, n,    // 8
    n, n, n, n, n, n, n, n, n, n};   // 9

  vector<uint8_t> costmapObstacleBL =
    // 0  1  2  3  4  5  6  7  8  9
  {n, n, n, n, n, n, n, n, n, n,     // 0
    n, o, o, o, o, o, o, o, o, n,    // 1
    n, o, o, o, o, o, o, o, o, n,    // 2
    n, o, o, o, o, o, o, o, o, n,    // 3
    n, o, o, o, o, o, o, o, o, n,    // 4
    n, o, x, x, x, o, o, o, o, n,    // 5
    n, o, x, x, x, o, o, o, o, n,    // 6
    n, o, x, x, x, o, o, o, o, n,    // 7
    n, o, o, o, o, o, o, o, o, n,    // 8
    n, n, n, n, n, n, n, n, n, n};   // 9

  vector<uint8_t> costmapObstacleTL =
    // 0  1  2  3  4  5  6  7  8  9
  {n, n, n, n, n, n, n, n, n, n,     // 0
    n, o, o, o, o, o, o, o, o, n,    // 1
    n, o, x, x, x, o, o, o, o, n,    // 2
    n, o, x, x, x, o, o, o, o, n,    // 3
    n, o, x, x, x, o, o, o, o, n,    // 4
    n, o, o, o, o, o, o, o, o, n,    // 5
    n, o, o, o, o, o, o, o, o, n,    // 6
    n, o, o, o, o, o, o, o, o, n,    // 7
    n, o, o, o, o, o, o, o, o, n,    // 8
    n, n, n, n, n, n, n, n, n, n};   // 9

  vector<uint8_t> costmapMaze =
    // 0  1  2  3  4  5  6  7  8  9
  {n, n, n, n, n, n, n, n, n, n,     // 0
    n, o, o, o, o, o, o, o, o, n,    // 1
    n, x, x, o, x, x, x, o, x, n,    // 2
    n, o, o, o, o, x, o, o, o, n,    // 3
    n, o, x, x, o, x, o, x, o, n,    // 4
    n, o, x, x, o, x, o, x, o, n,    // 5
    n, o, o, x, o, x, o, x, o, n,    // 6
    n, x, o, x, o, x, o, x, o, n,    // 7
    n, o, o, o, o, o, o, x, o, n,    // 8
    n, n, n, n, n, n, n, n, n, n};   // 9

  vector<uint8_t> costmapMaze2 =
    // 0  1  2  3  4  5  6  7  8  9
  {n, n, n, n, n, n, n, n, n, n,     // 0
    n, o, o, o, o, o, o, o, o, n,    // 1
    n, x, x, u, x, x, x, o, x, n,    // 2
    n, o, o, o, o, o, o, o, u, n,    // 3
    n, o, x, x, o, x, x, x, u, n,    // 4
    n, o, x, x, o, o, o, x, u, n,    // 5
    n, o, o, x, u, x, o, x, u, n,    // 6
    n, x, o, x, u, x, i, x, u, n,    // 7
    n, o, o, o, o, o, o, o, o, n,    // 8
    n, n, n, n, n, n, n, n, n, n};   // 9

  switch (testCostmapType) {
    case TestCostmap::open_space:
      return costmapFree;
    case TestCostmap::bounded:
      return costmapBounded;
    case TestCostmap::bottom_left_obstacle:
      return costmapObstacleBL;
    case TestCostmap::top_left_obstacle:
      return costmapObstacleTL;
    case TestCostmap::maze1:
      return costmapMaze;
    case TestCostmap::maze2:
      return costmapMaze2;
    default:
      return costmapFree;
  }
}

uint8_t Costmap::interpret_value(const int8_t value) const
{
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return no_information;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return free_space;
  } else if (value >= lethal_threshold_) {
    return lethal_obstacle;
  } else if (trinary_costmap_) {
    return free_space;
  }

  double scale = static_cast<double>(value / lethal_threshold_);
  return static_cast<uint8_t>(scale * lethal_obstacle);
}

bool Costmap::is_free(const unsigned int x_coordinate, const unsigned int y_coordinate) const
{
  unsigned int index = y_coordinate * costmap_properties_.size_x + x_coordinate;

  return is_free(index);
}

bool Costmap::is_free(const unsigned int index) const
{
  if (costs_[index] < Costmap::inscribed_inflated_obstacle) {
    return true;
  }

  return false;
}

}  // namespace nav2_util
