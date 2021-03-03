// Copyright (c) 2020 Samsung Research Russia
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

#ifndef NAV2_MAP_SERVER__MAP_SAVER_HPP_
#define NAV2_MAP_SERVER__MAP_SAVER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/save_map.hpp"

#include "map_io.hpp"

namespace nav2_map_server
{

/**
 * @class nav2_map_server::MapSaver
 * @brief A class that provides map saving methods and services
 */
class MapSaver : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_map_server::MapSaver
   */
  MapSaver();

  /**
   * @brief Destructor for the nav2_map_server::MapServer
   */
  ~MapSaver();

  /**
   * @brief Read a message from incoming map topic and save map to a file
   * @param map_topic Incoming map topic name
   * @param save_parameters Map saving parameters.
   * @return true of false
   */
  bool saveMapTopicToFile(
    const std::string & map_topic,
    const SaveParameters & save_parameters);

protected:
  /**
   * @brief Sets up map saving service
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when node switched to active state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when node switched to inactive state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when it is required node clean-up
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
   * @brief Map saving service callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response);

  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;
  // Default values for map thresholds
  double free_thresh_default_;
  double occupied_thresh_default_;
  // param for handling QoS configuration
  bool map_subscribe_transient_local_;

  // The name of the service for saving a map from topic
  const std::string save_map_service_name_{"save_map"};
  // A service to save the map to a file at run time (SaveMap)
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SAVER_HPP_
