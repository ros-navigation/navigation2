// Copyright (c) 2026, Aniruddh Yelluri
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

#ifndef NAV2_COSTMAP_2D__GEOFENCE_LAYER_HPP_
#define NAV2_COSTMAP_2D__GEOFENCE_LAYER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/srv/set_fence.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_costmap_2d
{

/**
 * @class GeofenceLayer
 * @brief A costmap layer that defines an operational zone (geofence).
 * Traces the polygon boundary using costmap raytracing and marks a
 * configurable-thickness band of cells outside the boundary as LETHAL_OBSTACLE.
 */
class GeofenceLayer : public CostmapLayer
{
public:
  /**
   * @brief GeofenceLayer constructor
   */
  GeofenceLayer() = default;

  /**
   * @brief GeofenceLayer destructor
   */
  ~GeofenceLayer() = default;

  /**
   * @brief Initialization process of layer on startup.
   */
  void onInitialize() override;

  /**
   * @brief Activate this layer.
   */
  void activate() override;

  /**
   * @brief Deactivate this layer.
   */
  void deactivate() override;

  /**
   * @brief Reset the layer.
   */
  void reset() override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() override {return false;}

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions.
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap.
   * @param master_grid The master costmap grid to update
   * @param min_i X min map coord of the window to update
   * @param min_j Y min map coord of the window to update
   * @param max_i X max map coord of the window to update
   * @param max_j Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

protected:
  /**
   * @brief Callback for the SetFence service
   */
  void setFenceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response);

  /**
   * @brief Process a newly buffered polygon; resize the costmap if configured
   */
  void processFence();

  /**
   * @brief Rasterize the polygon boundary and inflate it outward by border_thickness_
   */
  void rasterizeFence();

  /**
   * @brief Convert a Point vector to a PolygonStamped and store in the buffer
   */
  void bufferPolygon(const std::vector<geometry_msgs::msg::Point> & pts);

  /**
   * @brief Validate incoming parameter updates before applying them
   * @param parameters List of parameters to validate
   * @return rcl_interfaces::msg::SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply validated parameter updates
   * @param parameters List of parameters that have been updated
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_buffer_;
  bool has_updated_data_{false};
  bool has_fence_{false};

  std::string global_frame_;  ///< @brief The global frame for the costmap
  bool resize_to_fence_{true};
  unsigned int border_thickness_{3};
  tf2::Duration transform_tolerance_;

  /// @brief Vertices of the polygon in the global frame
  std::vector<geometry_msgs::msg::Point> polygon_points_;

  nav2::ServiceServer<nav2_msgs::srv::SetFence>::SharedPtr set_fence_service_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__GEOFENCE_LAYER_HPP_
