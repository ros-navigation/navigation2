// Copyright (c) 2020 Samsung R&D Institute Russia
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

#include "mapio.hpp"

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
   * NOTE: map_topic could be updated during function execution.
   * @param save_parameters Map saving parameters.
   * NOTE: save_parameters could be updated during function execution.
   * @return true of false
   */
  bool saveMapTopicToFile(std::string & map_topic, SaveParameters & save_parameters);

protected:
  // Lifecycle interfaces
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief A callback function that receives map message from subscribed topic
   * @param map Occupancy Grid message data
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;
  // Default values for map thresholds
  int free_thresh_default_;
  int occupied_thresh_default_;

  // The name of the service for saving a map from topic
  const std::string save_map_service_name_{"save_map"};
  // A service to save the map to a file at run time (SaveMap)
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;

  // Map topic listener node
  rclcpp::Node::SharedPtr map_listener_;

  // Pointer to map message received in the subscription callback
  nav_msgs::msg::OccupancyGrid::SharedPtr msg_;
  // Indicator that map message was receiced
  bool got_map_msg_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_SAVER_HPP_
