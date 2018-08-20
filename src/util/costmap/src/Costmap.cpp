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

#include "costmap/Costmap.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::vector;

const Costmap::CostValue Costmap::no_information = 255;
const Costmap::CostValue Costmap::lethal_obstacle = 254;
const Costmap::CostValue Costmap::inscribed_inflated_obstacle = 253;
const Costmap::CostValue Costmap::medium_cost = 128;
const Costmap::CostValue Costmap::free_space = 0;

// TODO(orduno): Port ROS1 Costmap package
Costmap::Costmap(rclcpp::Node * node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Costmap::Costmap");
}

nav2_msgs::msg::Costmap
Costmap::getCostmap(const nav2_msgs::msg::CostmapMetaData & /*specifications*/)
{
  // TODO(orduno): build a costmap given the specifications

  RCLCPP_INFO(node_->get_logger(), "Costmap::getCostmap");

  // TODO(orduno): faking out a costmap for now

  nav2_msgs::msg::Costmap costmap;
  costmap.header.stamp = node_->now();
  costmap.header.frame_id = "/map";

  // Fill map metadata
  costmap.info.map_load_time = node_->now();

  // Some arbitrary numbers
  costmap.info.resolution = 1;  // m/cell
  costmap.info.width = 10;  // cells
  costmap.info.height = 10;  // cells

  // The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
  // Origin is lower-left pixel?
  costmap.info.origin.position.x = 0.0;  // 2D pose of the lower-left pixel in the map
  costmap.info.origin.position.y = 0.0;
  costmap.info.origin.position.z = 0.0;  // ignored

  // Define map rotation
  // Provided as yaw with counterclockwise rotation, with yaw = 0 meaning no rotation

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  costmap.info.origin.orientation.x = quaternion.x();
  costmap.info.origin.orientation.y = quaternion.y();
  costmap.info.origin.orientation.z = quaternion.z();
  costmap.info.origin.orientation.w = quaternion.w();

  costmap.data.resize(costmap.info.width, costmap.info.height);

  // Fill with some fake data for testing
  costmap.data = getTestData(costmap.info.width, costmap.info.height);

  return costmap;
}

vector<uint8_t>
Costmap::getTestData(const int /*width*/, const int /*height*/)
{
  // TODO(orduno): fixed size for now

  // TODO(orduno): besides hardcoded costmaps, use a mathematical function

  const uint8_t n = no_information;
  const uint8_t x = lethal_obstacle;
  const uint8_t i = inscribed_inflated_obstacle;
  const uint8_t u = medium_cost;
  const uint8_t o = free_space;

  vector<uint8_t> costmapFree =
  // 0 1 2 3 4 5 6 7 8 9
    {o,o,o,o,o,o,o,o,o,o,   //0
     o,o,o,o,o,o,o,o,o,o,   //1
     o,o,o,o,o,o,o,o,o,o,   //2
     o,o,o,o,o,o,o,o,o,o,   //3
     o,o,o,o,o,o,o,o,o,o,   //4
     o,o,o,o,o,o,o,o,o,o,   //5
     o,o,o,o,o,o,o,o,o,o,   //6
     o,o,o,o,o,o,o,o,o,o,   //7
     o,o,o,o,o,o,o,o,o,o,   //8
     o,o,o,o,o,o,o,o,o,o,}; //9

  vector<uint8_t> costmapBounded =
  // 0 1 2 3 4 5 6 7 8 9
    {n,n,n,n,n,n,n,n,n,n,   //0
     n,o,o,o,o,o,o,o,o,n,   //1
     n,o,o,o,o,o,o,o,o,n,   //2
     n,o,o,o,o,o,o,o,o,n,   //3
     n,o,o,o,o,o,o,o,o,n,   //4
     n,o,o,o,o,o,o,o,o,n,   //5
     n,o,o,o,o,o,o,o,o,n,   //6
     n,o,o,o,o,o,o,o,o,n,   //7
     n,o,o,o,o,o,o,o,o,n,   //8
     n,n,n,n,n,n,n,n,n,n};  //9

  vector<uint8_t> costmapObstacleBL =
  // 0 1 2 3 4 5 6 7 8 9
    {n,n,n,n,n,n,n,n,n,n,   //0
     n,o,o,o,o,o,o,o,o,n,   //1
     n,o,o,o,o,o,o,o,o,n,   //2
     n,o,o,o,o,o,o,o,o,n,   //3
     n,o,o,o,o,o,o,o,o,n,   //4
     n,o,x,x,x,o,o,o,o,n,   //5
     n,o,x,x,x,o,o,o,o,n,   //6
     n,o,x,x,x,o,o,o,o,n,   //7
     n,o,o,o,o,o,o,o,o,n,   //8
     n,n,n,n,n,n,n,n,n,n};  //9

  vector<uint8_t> costmapObstacleTL =
  // 0 1 2 3 4 5 6 7 8 9
    {n,n,n,n,n,n,n,n,n,n,   //0
     n,o,o,o,o,o,o,o,o,n,   //1
     n,o,x,x,x,o,o,o,o,n,   //2
     n,o,x,x,x,o,o,o,o,n,   //3
     n,o,x,x,x,o,o,o,o,n,   //4
     n,o,o,o,o,o,o,o,o,n,   //5
     n,o,o,o,o,o,o,o,o,n,   //6
     n,o,o,o,o,o,o,o,o,n,   //7
     n,o,o,o,o,o,o,o,o,n,   //8
     n,n,n,n,n,n,n,n,n,n};  //9

  vector<uint8_t> costmapMaze =
  // 0 1 2 3 4 5 6 7 8 9
    {n,n,n,n,n,n,n,n,n,n,   //0
     n,o,o,o,o,o,o,o,o,n,   //1
     n,x,x,o,x,x,x,o,x,n,   //2
     n,o,o,o,o,x,o,o,o,n,   //3
     n,o,x,x,o,x,o,x,o,n,   //4
     n,o,x,x,o,x,o,x,o,n,   //5
     n,o,o,x,o,x,o,x,o,n,   //6
     n,x,o,x,o,x,o,x,o,n,   //7
     n,o,o,o,o,o,o,x,o,n,   //8
     n,n,n,n,n,n,n,n,n,n};  //9

  vector<uint8_t> costmapMaze2 =
  // 0 1 2 3 4 5 6 7 8 9
    {n,n,n,n,n,n,n,n,n,n,   //0
     n,o,o,o,o,o,o,o,o,n,   //1
     n,x,x,u,x,x,x,o,x,n,   //2
     n,o,o,o,o,o,o,o,u,n,   //3
     n,o,x,x,o,x,x,x,u,n,   //4
     n,o,x,x,o,o,o,x,u,n,   //5
     n,o,o,x,u,x,o,x,u,n,   //6
     n,x,o,x,u,x,i,x,u,n,   //7
     n,o,o,o,o,o,o,o,o,n,   //8
     n,n,n,n,n,n,n,n,n,n};  //9

  return costmapMaze2;
}
