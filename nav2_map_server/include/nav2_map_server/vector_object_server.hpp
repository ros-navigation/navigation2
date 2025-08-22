// Copyright (c) 2023 Samsung R&D Institute Russia
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

#ifndef NAV2_MAP_SERVER__VECTOR_OBJECT_SERVER_HPP_
#define NAV2_MAP_SERVER__VECTOR_OBJECT_SERVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "nav2_msgs/srv/add_shapes.hpp"
#include "nav2_msgs/srv/remove_shapes.hpp"
#include "nav2_msgs/srv/get_shapes.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

#include "nav2_map_server/vector_object_utils.hpp"
#include "nav2_map_server/vector_object_shapes.hpp"

namespace nav2_map_server
{

/// @brief Vector Object server node
class VectorObjectServer : public nav2::LifecycleNode
{
public:
  /**
   * @brief Constructor for the VectorObjectServer
   * @param options Additional options to control creation of the node.
   */
  explicit VectorObjectServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief: Initializes TF buffer/listener, obtains ROS-parameters, creates incoming services,
   * and output map publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Activates output map publisher and creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Deactivates map publisher and timer (if any), destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief: Resets all services, publishers, map and TF-s
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @return True if all parameters were obtained or false in the failure case
   */
  bool obtainParams();

  /**
   * @brief Finds the shape with given UUID
   * @param uuid Given UUID to search with
   * @return Iterator to the shape, if found. Otherwise past-the-end iterator.
   */
  std::vector<std::shared_ptr<Shape>>::iterator findShape(const unsigned char * uuid);

  /**
   * @brief Transform all vector shapes from their local frame to output map frame
   * @return Whether all vector objects were transformed successfully
   */
  bool transformVectorObjects();

  /**
   * @brief Obtains map boundaries to place all vector objects inside
   * @param min_x output min X-boundary of required map
   * @param min_y output min Y-boundary of required map
   * @param max_x output max X-boundary of required map
   * @param max_y output max Y-boundary of required map
   * @throw std::exception if can not obtain one of the map boundaries
   */
  void getMapBoundaries(double & min_x, double & min_y, double & max_x, double & max_y) const;

  /**
   * @brief Creates or updates existing map with required sizes and fills it with default value
   * @param min_x min X-boundary of new map
   * @param min_y min Y-boundary of new map
   * @param max_x max X-boundary of new map
   * @param max_y max Y-boundary of new map
   * @throw std::exception if map has negative X or Y size
   */
  void updateMap(
    const double & min_x, const double & min_y, const double & max_x, const double & max_y);

  /**
   * @brief Processes all vector objects on raster output map
   */
  void putVectorObjectsOnMap();

  /**
   * @brief Publishes output map
   */
  void publishMap();

  /**
   * @brief Calculates new map sizes, updates map, processes all vector objects on it
   * and publishes output map one time
   */
  void processMap();

  /**
   * @brief If map to be update dynamically, creates map processing timer,
   * otherwise process map once
   */
  void switchMapUpdate();

  /**
   * @brief Callback for AddShapes service call.
   * Reads all input vector objects from service request,
   * creates them or updates their shape in case of existing objects
   * and switches map processing/publishing routine
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void addShapesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::AddShapes::Request> request,
    std::shared_ptr<nav2_msgs::srv::AddShapes::Response> response);

  /**
   * @brief Callback for GetShapes service call.
   * Gets all shapes and returns them to the service response
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void getShapesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetShapes::Request> request,
    std::shared_ptr<nav2_msgs::srv::GetShapes::Response> response);

  /**
   * @brief Callback for RemoveShapes service call.
   * Try to remove requested vector objects and switches map processing/publishing routine
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void removeShapesCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::RemoveShapes::Request> request,
    std::shared_ptr<nav2_msgs::srv::RemoveShapes::Response> response);

protected:
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief All shapes vector
  std::vector<std::shared_ptr<Shape>> shapes_;

  /// @brief Output map resolution
  double resolution_;
  /// @brief Default value the output map to be filled with
  int8_t default_value_;
  /// @brief @Overlay Type of overlay of vector objects on the map
  OverlayType overlay_type_;

  /// @brief Output map with vector objects on it
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  /// @brief Whether to process and publish map
  double process_map_;

  /// @brief Frame of output map
  std::string global_frame_id_;
  /// @brief Transform tolerance
  double transform_tolerance_;

  /// @brief Frequency to dynamically update/publish the map (if necessary)
  double update_frequency_;
  /// @brief Map update timer
  rclcpp::TimerBase::SharedPtr map_timer_;

  /// @brief AddShapes service
  nav2::ServiceServer<nav2_msgs::srv::AddShapes>::SharedPtr add_shapes_service_;
  /// @brief GetShapes service
  nav2::ServiceServer<nav2_msgs::srv::GetShapes>::SharedPtr get_shapes_service_;
  /// @brief RemoveShapes service
  nav2::ServiceServer<nav2_msgs::srv::RemoveShapes>::SharedPtr remove_shapes_service_;

  /// @beirf Topic name where the output map to be published to
  std::string map_topic_;
  /// @brief Output map publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__VECTOR_OBJECT_SERVER_HPP_
