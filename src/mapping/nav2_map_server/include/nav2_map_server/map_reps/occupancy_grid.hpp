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

#ifndef NAV2_MAP_SERVER__MAP_REPS__OCCUPANCY_GRID_HPP_
#define NAV2_MAP_SERVER__MAP_REPS__OCCUPANCY_GRID_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/base_map_loader.hpp"

enum MapMode
{
  TRINARY,
  SCALE,
  RAW
};

class OccGridLoader : public BaseMapLoader
{
public:
  // Interface //

  void loadMapInfoFromFile(std::string fname);

  void loadMapFromFile(std::string mapfname);

  void publishMap();

  void setMap();

  void connectROS(rclcpp::Node::SharedPtr n);

  void createROSInterface(rclcpp::Node::SharedPtr n);

  // Occupancy Grid Specific //

  OccGridLoader() {}

  OccGridLoader(rclcpp::Node::SharedPtr n, std::string filename);

  ~OccGridLoader() {}

  void OccMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
    const std::shared_ptr<nav_msgs::srv::GetMap::Response> res);

protected:
  // Info From YAML file
  double origin[3];
  int negate;
  double occ_th, free_th;
  double res;
  MapMode mode = TRINARY;
  std::string frame_id = "map";

  // Occupancy Grid ROS Message / Service
  nav_msgs::msg::OccupancyGrid map_msg_;
  nav_msgs::srv::GetMap::Response occ_resp_;

  // ROS Interfaces
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;
};

#endif  // NAV2_MAP_SERVER__MAP_REPS__OCCUPANCY_GRID_HPP_
