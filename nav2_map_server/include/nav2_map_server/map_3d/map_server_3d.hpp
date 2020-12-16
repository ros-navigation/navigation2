// Copyright (c) 2020 Shivam Pandey pandeyshivam2017robotics@gmail.com
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

#ifndef NAV2_MAP_SERVER__MAP_3D__MAP_SERVER_3D_HPP_
#define NAV2_MAP_SERVER__MAP_3D__MAP_SERVER_3D_HPP_

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/get_map3_d.hpp"
#include "nav2_msgs/srv/load_map3_d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_map_server/map_server_core.hpp"

namespace nav2_map_server
{
/**
 * @class nav2_map_server::MapServer
 * @brief Parses the map yaml file and creates a service and a publisher that
 * provides occupancy grid
 */
template<>
class MapServer<sensor_msgs::msg::PointCloud2>: public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_map_server::MapServer
   */
  MapServer();

  /**
   * @brief A Destructor for nav2_map_server::MapServer
   */
  ~MapServer() override;

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
   * generate output response containing a PointCloud2.
   * Update pcd_msg_ class variable.
   * @param yaml_file name of input YAML file
   * @param response Output response with loaded PointCloud2 map
   * @return true or false
   */
  bool loadMapResponseFromYaml(
    const std::string & yaml_file,
    std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response);

  /**
   * @brief Map getting service callback for pcd
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void getMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetMap3D::Request> request,
    std::shared_ptr<nav2_msgs::srv::GetMap3D::Response> response);

  /**
   * @brief Map loading service callback for pcd
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::LoadMap3D::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap3D::Response> response);


  // The name of the service for getting a map
  const std::string service_name_{"map"};

  // The name of the service for loading a map
  const std::string load_map_service_name_{"load_map"};

  // PointCloud2 related fields

  // A service to provide the pointcloud2 (GetMap3D) and the message to return
  rclcpp::Service<nav2_msgs::srv::GetMap3D>::SharedPtr pcd_service_;

  // A service to load the PointCloud2 from file at run time (LoadMap3D)
  rclcpp::Service<nav2_msgs::srv::LoadMap3D>::SharedPtr pcd_load_map_service_;

  // A topic on which the PointCloud2 will be published
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  // The message to publish the pointcloud topic
  sensor_msgs::msg::PointCloud2 pcd_msg_;

  // A topic on which the Pose will be published
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr origin_pub_;

  // The message to publish the pose topic
  geometry_msgs::msg::Pose origin_msg_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_3D__MAP_SERVER_3D_HPP_
