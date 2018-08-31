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

#include "nav2_util/costmap.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::vector;

namespace nav2_util
{

const Costmap::CostValue Costmap::no_information = 255;
const Costmap::CostValue Costmap::lethal_obstacle = 254;
const Costmap::CostValue Costmap::inscribed_inflated_obstacle = 253;
const Costmap::CostValue Costmap::medium_cost = 128;
const Costmap::CostValue Costmap::free_space = 0;

// TODO(orduno): Port ROS1 Costmap package
Costmap::Costmap(rclcpp::Node * node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Costmap::Costmap");
}

nav2_tasks::msg::Costmap
Costmap::getCostmap(const nav2_tasks::msg::CostmapMetaData & /*specifications*/)
{
  // TODO(orduno): build a costmap given the specifications

  RCLCPP_INFO(node_->get_logger(), "Costmap::getCostmap");

  // TODO(orduno): faking out a costmap for now

  nav2_tasks::msg::Costmap costmap;
  costmap.header.stamp = node_->now();
  costmap.header.frame_id = "/map";

  // Fill map metadata
  costmap.metadata.map_load_time = node_->now();

  // Some arbitrary numbers
  costmap.metadata.resolution = 1;  // m/cell
  costmap.metadata.size_x = 10;  // cells
  costmap.metadata.size_y = 10;  // cells

  // The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
  // Origin is lower-left pixel?
  costmap.metadata.origin.position.x = 0.0;  // 2D pose of the lower-left pixel in the map
  costmap.metadata.origin.position.y = 0.0;
  costmap.metadata.origin.position.z = 0.0;  // ignored

  // Define map rotation
  // Provided as yaw with counterclockwise rotation, with yaw = 0 meaning no rotation

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  costmap.metadata.origin.orientation.x = quaternion.x();
  costmap.metadata.origin.orientation.y = quaternion.y();
  costmap.metadata.origin.orientation.z = quaternion.z();
  costmap.metadata.origin.orientation.w = quaternion.w();

  costmap.data.resize(costmap.metadata.size_x, costmap.metadata.size_y);

  // Fill with some fake data for testing
  costmap.data = getTestData(costmap.metadata.size_x, costmap.metadata.size_y);

  return costmap;
}

vector<uint8_t>
Costmap::getTestData(const int /*size_x*/, const int /*size_y*/)
{
  // TODO(orduno): fixed size for now

  // TODO(orduno): besides hardcoded costmaps, use a mathematical function

  const uint8_t n = no_information;
  const uint8_t x = lethal_obstacle;
  const uint8_t i = inscribed_inflated_obstacle;
  const uint8_t u = medium_cost;
  const uint8_t o = free_space;

  // TODO(orduno): make these vector of vectors, select maze by index
  //               within the test program

  vector<uint8_t> costmapFree =
    // 0 1 2 3 4 5 6 7 8 9
  {o, o, o, o, o, o, o, o, o, o,    // 0
    o, o, o, o, o, o, o, o, o, o,   // 1
    o, o, o, o, o, o, o, o, o, o,   // 2
    o, o, o, o, o, o, o, o, o, o,   // 3
    o, o, o, o, o, o, o, o, o, o,   // 4
    o, o, o, o, o, o, o, o, o, o,   // 5
    o, o, o, o, o, o, o, o, o, o,   // 6
    o, o, o, o, o, o, o, o, o, o,   // 7
    o, o, o, o, o, o, o, o, o, o,   // 8
    o, o, o, o, o, o, o, o, o, o};  // 9

  vector<uint8_t> costmapBounded =
    // 0 1 2 3 4 5 6 7 8 9
  {n, n, n, n, n, n, n, n, n, n,    // 0
    n, o, o, o, o, o, o, o, o, n,   // 1
    n, o, o, o, o, o, o, o, o, n,   // 2
    n, o, o, o, o, o, o, o, o, n,   // 3
    n, o, o, o, o, o, o, o, o, n,   // 4
    n, o, o, o, o, o, o, o, o, n,   // 5
    n, o, o, o, o, o, o, o, o, n,   // 6
    n, o, o, o, o, o, o, o, o, n,   // 7
    n, o, o, o, o, o, o, o, o, n,   // 8
    n, n, n, n, n, n, n, n, n, n};  // 9

  vector<uint8_t> costmapObstacleBL =
    // 0 1 2 3 4 5 6 7 8 9
  {n, n, n, n, n, n, n, n, n, n,    // 0
    n, o, o, o, o, o, o, o, o, n,   // 1
    n, o, o, o, o, o, o, o, o, n,   // 2
    n, o, o, o, o, o, o, o, o, n,   // 3
    n, o, o, o, o, o, o, o, o, n,   // 4
    n, o, x, x, x, o, o, o, o, n,   // 5
    n, o, x, x, x, o, o, o, o, n,   // 6
    n, o, x, x, x, o, o, o, o, n,   // 7
    n, o, o, o, o, o, o, o, o, n,   // 8
    n, n, n, n, n, n, n, n, n, n};  // 9

  vector<uint8_t> costmapObstacleTL =
    // 0 1 2 3 4 5 6 7 8 9
  {n, n, n, n, n, n, n, n, n, n,    // 0
    n, o, o, o, o, o, o, o, o, n,   // 1
    n, o, x, x, x, o, o, o, o, n,   // 2
    n, o, x, x, x, o, o, o, o, n,   // 3
    n, o, x, x, x, o, o, o, o, n,   // 4
    n, o, o, o, o, o, o, o, o, n,   // 5
    n, o, o, o, o, o, o, o, o, n,   // 6
    n, o, o, o, o, o, o, o, o, n,   // 7
    n, o, o, o, o, o, o, o, o, n,   // 8
    n, n, n, n, n, n, n, n, n, n};  // 9

  vector<uint8_t> costmapMaze =
    // 0 1 2 3 4 5 6 7 8 9
  {n, n, n, n, n, n, n, n, n, n,    // 0
    n, o, o, o, o, o, o, o, o, n,   // 1
    n, x, x, o, x, x, x, o, x, n,   // 2
    n, o, o, o, o, x, o, o, o, n,   // 3
    n, o, x, x, o, x, o, x, o, n,   // 4
    n, o, x, x, o, x, o, x, o, n,   // 5
    n, o, o, x, o, x, o, x, o, n,   // 6
    n, x, o, x, o, x, o, x, o, n,   // 7
    n, o, o, o, o, o, o, x, o, n,   // 8
    n, n, n, n, n, n, n, n, n, n};  // 9

  vector<uint8_t> costmapMaze2 =
    // 0 1 2 3 4 5 6 7 8 9
  {n, n, n, n, n, n, n, n, n, n,    // 0
    n, o, o, o, o, o, o, o, o, n,   // 1
    n, x, x, u, x, x, x, o, x, n,   // 2
    n, o, o, o, o, o, o, o, u, n,   // 3
    n, o, x, x, o, x, x, x, u, n,   // 4
    n, o, x, x, o, o, o, x, u, n,   // 5
    n, o, o, x, u, x, o, x, u, n,   // 6
    n, x, o, x, u, x, i, x, u, n,   // 7
    n, o, o, o, o, o, o, o, o, n,   // 8
    n, n, n, n, n, n, n, n, n, n};  // 9

  return costmapMaze2;
}

}  // namespace nav2_util
