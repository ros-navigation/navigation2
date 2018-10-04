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

#ifndef NAV2_MAP_SERVER__MAP_REPS__OCC_GRID_SERVER_HPP_
#define NAV2_MAP_SERVER__MAP_REPS__OCC_GRID_SERVER_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/base_map_server.hpp"

namespace nav2_map_server
{

class OccGridServer : public BaseMapServer
{
public:
  explicit OccGridServer(rclcpp::Node::SharedPtr node)
  : node_(node) {}
  OccGridServer(rclcpp::Node::SharedPtr node, std::string file_name);
  OccGridServer() {}
  ~OccGridServer() {}

  void LoadMapInfoFromFile(const std::string & file_name);
  void LoadMapFromFile(const std::string & map_name);
  void PublishMap();
  void SetMap();
  void ConnectROS();

protected:
  enum MapMode { TRINARY, SCALE, RAW };

  // Info from parsing the YAML file
  double origin_[3];
  int negate_;
  double occ_th_;
  double free_th_;
  double res_;
  MapMode mode_ = TRINARY;
  std::string frame_id_ = "map";
  std::string map_name_;

  // The ROS node to use for ROS-related operations such as creating a service
  rclcpp::Node::SharedPtr node_;

  // A service to provide the ouccpancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;
  nav_msgs::srv::GetMap::Response occ_resp_;

  // A topic on which the occupance grid will be published and the message to publish
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  nav_msgs::msg::OccupancyGrid map_msg_;

  // For now, publish the map periodically so that its sure to be received on the
  // ROS1 side across the ROS1 bridge
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_REPS__OCC_GRID_SERVER_HPP_
