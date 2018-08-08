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


#include "world_model/WorldModel.hpp"
#include <tf2/LinearMath/Quaternion.h>

using std::vector;
using std::string;

WorldModel::WorldModel(const string& name) : Node (name)
{
  // Create a (lambda) callback function for when a costmap service is received.
  auto handle_costmap_service = [this]
    (const std::shared_ptr<rmw_request_id_t> request_header,
     const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
     const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response) -> void
     {
      (void)request_header; // suppress unused variable warning
      (void)request;
      RCLCPP_INFO(this->get_logger(), "WorldModel::WorldModel:Incoming costmap request");
      getCostmap(response->map);
     };

  // Create a service that will use the callback function to handle requests.
  costmapServer_ = create_service<nav2_msgs::srv::GetCostmap>(name, handle_costmap_service);
}

void
WorldModel::getCostmap(nav2_msgs::msg::Costmap& costmap)
{
  costmap.header.stamp = this->now();
  costmap.header.frame_id = "/map";

  // Fill map metadata
  costmap.info.map_load_time = this->now();

  // Some arbitrary numbers
  costmap.info.resolution = 1;  // m/cell
  costmap.info.width = 10;  // cells
  costmap.info.height = 10;  // cells

  // The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map.
  // Origin is lower-left pixel?
  costmap.info.origin.position.x = 0.0; // 2D pose of the lower-left pixel in the map
  costmap.info.origin.position.y = 0.0;
  costmap.info.origin.position.z = 0.0; // ignored

  // Define map rotation
  // Provided as yaw with counterclockwise rotation, with yaw = 0 meaning no rotation

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0); // set roll, pitch, yaw
  costmap.info.origin.orientation.x = quaternion.x();
  costmap.info.origin.orientation.y = quaternion.y();
  costmap.info.origin.orientation.z = quaternion.z();
  costmap.info.origin.orientation.w = quaternion.w();

  costmap.data.resize(costmap.info.width, costmap.info.height);
  getCostVector(costmap.info.width, costmap.info.height, costmap.data);
}

void
WorldModel::getCostVector(const int width, const int height, vector<uint8_t>& data)
{
  // TODO(orduno): fixed size for now
  (void)width;
  (void)height;

  // TODO(orduno): instead of hardcoding, define a function

  const uint8_t n = 255; // no information
  const uint8_t x = 254; // lethal obstacle
  const uint8_t i = 253; // inscribed inflated obstacle
  const uint8_t u = 128; // medium cost
  const uint8_t o = 0;   // free space

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

  data = costmapMaze;
}
