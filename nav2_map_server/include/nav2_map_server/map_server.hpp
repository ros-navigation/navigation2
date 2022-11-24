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

#ifndef NAV2_MAP_SERVER__MAP_SERVER_HPP_
#define NAV2_MAP_SERVER__MAP_SERVER_HPP_

#include <octomap/octomap.h>

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_msgs/srv/get_grid_map.hpp"

#include <octomap_msgs/msg/octomap.hpp>
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace nav2_map_server
{

/**
 * @class nav2_map_server::MapServer
 * @brief Parses the map yaml file and creates a service and a publisher that
 * provides occupancy grid
 */
class MapServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_map_server::MapServer
   * @param options Additional options to control creation of the node.
   */
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A Destructor for nav2_map_server::MapServer
   */
  ~MapServer();

protected:
  /**
   * @brief Sets up required params and services. Loads map and its parameters from the file
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Start publishing the map using the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Stops publishing the latched topic
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets the member variables
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Load the map YAML, image from map file name and
   * generate output response containing an OccupancyGrid.
   * Update msg_ class variable.
   * @param yaml_file name of input YAML file
   * @param response Output response with loaded OccupancyGrid map
   * @return true or false
   */
  bool loadMapResponseFromYaml(
    const std::string & yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  /**
   * @brief Method correcting msg_ header when it belongs to instantiated object
   */
  void updateMsgHeader();
  /**
   * @brief Method correcting transform
   */
  void updateTransform();

  /**
   * @brief Map getting service callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void getMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

  /**
   * @brief GridMap getting service callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void getGridMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
    std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  /**
   * @brief Map loading service callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  // The name of the service for getting a map
  const std::string occ_map_service_name_{"map"};

  // The name of the service for getting a map
  const std::string grid_map_service_name_{"grid_map"};

  // The name of the service for loading a map
  const std::string load_map_service_name_{"load_map"};

  // A service to provide the occupancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

  // A service to provide the grid_map (GetGridMap) and the message to return
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr grid_map_service_;

  // A service to load the occupancy grid from file at run time (LoadMap)
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;

  // A topic on which the occupancy grid will be published
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // A topic on which the grid_map will be published
  rclcpp_lifecycle::LifecyclePublisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;

  // A topic on which the grid_map will be published
  rclcpp_lifecycle::LifecyclePublisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

  // The frame ID used in the returned OccupancyGrid message
  std::string frame_id_;

  // The message to publish on the occupancy grid topic
  nav_msgs::msg::OccupancyGrid msg_;

  // true if msg_ was initialized
  bool map_available_;
  // The message to publish on the occupancy grid topic
  grid_map_msgs::msg::GridMap msg_grid_map_;

  // The transform frame ID to represent orient
  std::string grid_map_frame_id_;
  geometry_msgs::msg::TransformStamped map_to_grid_map_t_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // The message to publish on the octomap topic
  octomap_msgs::msg::Octomap msg_octomap_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SERVER_HPP_
