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

#ifndef NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_
#define NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_map_server/map_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_map_server
{

class OccGridLoader : public MapLoader
{
public:
  explicit OccGridLoader(rclcpp::Node * node, YAML::Node & doc);
  OccGridLoader() = delete;

  // Load the image and generate an OccupancyGrid
  void loadMapFromFile(const std::string & filename) override;

  // Make the OccupancyGrid available via ROS topic and service
  void startServices() override;

protected:
  // The ROS node to use for ROS-related operations such as creating a service
  rclcpp::Node * node_;

  // The YAML file from which to get the conversion parameters
  YAML::Node & doc_;

  // Conversion parameters read from the YAML document
  double resolution_;
  std::vector<double> origin_;
  double free_thresh_;
  double occupied_thresh_;
  enum MapMode { TRINARY, SCALE, RAW };
  MapMode mode_;
  int negate_;

  // A service to provide the ouccpancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

  // A topic on which the occupancy grid will be published
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // The message to publish on the occupancy grid topic
  nav_msgs::msg::OccupancyGrid msg_;

  // The frame ID used in the returned OccupancyGrid message
  static const char * frame_id_;

  // The name for the topic on which the map will be published
  static const char * topic_name_;

  // The name of the service for getting a map
  static const char * service_name_;

  // For now, use a timer to publish the map periodically so that it is sure
  // to be received on the ROS1 side across the ROS1 bridge
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__OCC_GRID_LOADER_HPP_
