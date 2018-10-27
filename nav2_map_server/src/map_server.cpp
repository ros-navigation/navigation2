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

#include "nav2_map_server/map_server.hpp"

#include <string>
#include <memory>
#include <chrono>
#include "nav2_map_server/occ_grid_loader.hpp"

using namespace std::chrono_literals;

namespace nav2_map_server
{

MapServer::MapServer(const std::string & name)
: Node(name)
{
  map_loader_ = createMapLoader();
  // TODO: map_loader_->loadMapFromFile(filename); // a filename param
  map_msg_ = map_loader_->getOccupancyGrid();

  initServices();
}

std::unique_ptr<MapLoader>
MapServer::createMapLoader()
{
  // Get the parameters
  std::string map_type;
  get_parameter_or_set("map_type", map_type, std::string("occupancy"));

  // Create the specified type of map loader
  if (map_type == "occupancy") {
    return std::make_unique<OccGridLoader>(this);
  }

  RCLCPP_ERROR(get_logger(), "Cannot create map loader for map type %s", map_type.c_str());
  throw std::runtime_error("Map type not supported");
}

void MapServer::initServices()
{
  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      RCLCPP_INFO(get_logger(), "Handling map request");
      response->map = map_msg_;
    };

  // Create a service that provides the occupancy grid
  occ_service_ = create_service<nav_msgs::srv::GetMap>("occ_grid", handle_occ_callback);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "occ_grid", custom_qos_profile);

  // Publish the map using the latched topic
  occ_pub_->publish(map_msg_);

  // TODO(mjeronimo): Remove the following once we've got everything on the ROS2 side
  //
  // Periodically publish the map so that the ros1 bridge will be sure the proxy the
  // message to rviz on the ROS1 side
  auto timer_callback = [this]() -> void {occ_pub_->publish(map_msg_);};
  timer_ = create_wall_timer(2s, timer_callback);
}

}  // namespace nav2_map_server
