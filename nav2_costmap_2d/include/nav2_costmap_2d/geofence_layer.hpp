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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/srv/set_fence.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"

namespace nav2_costmap_2d
{

/**
 * @class GeofenceLayer
 * @brief A costmap layer that defines an operational zone (geofence).
 * Cells outside the geofence polygon are marked as LETHAL_OBSTACLE,
 * preventing the planner from routing outside the zone.
 *
 * The geofence can be set via a ROS parameter at startup or updated dynamically
 * at run-time through the SetFence service. Optionally, the costmap can be
 * resized to fit the geofence polygon (useful for static costmaps; disabled
 * for rolling costmaps).
 */
class GeofenceLayer : public CostmapLayer
{
public:
  GeofenceLayer() = default;
  ~GeofenceLayer() = default;

  /**
   * @brief Initialization process of layer on startup.
   * Reads the initial geofence from the fence_polygon parameter (if provided)
   * and advertises the SetFence service.
   */
  void onInitialize() override;

  /**
   * @brief Activate this layer, registering dynamic parameter callbacks.
   */
  void activate() override;

  /**
   * @brief Deactivate this layer, removing dynamic parameter callbacks.
   */
  void deactivate() override;

  /**
   * @brief Reset the layer, clearing all geofence state and buffers.
   */
  void reset() override;

  /**
   * @brief Match the size of the internal costmap to the master costmap.
   */
  void matchSize() override;

  /**
   * @brief Geofence is a hard constraint — it is not clearable.
   * @return false
   */
  bool isClearable() override {return false;}

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions.
   * Handles costmap resizing when a new polygon is received and resize_to_fence is true.
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
   * @brief Update the costs in the master costmap using a pre-rasterized fence mask.
   * Cells outside the geofence are set to LETHAL_OBSTACLE via flat array lookup.
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
   * @brief Callback for the SetFence service.
   * Validates the polygon (>= 3 vertices), optionally transforms it to the
   * global frame via TF, then stores it and flags for re-rasterization.
   */
  void setFenceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response);

  /**
   * @brief Process a newly buffered polygon: optionally resize the costmap,
   * then pre-rasterize the fence mask for fast updateCosts lookups.
   * Must be called from within the costmap update thread (updateBounds context).
   */
  void processFence();

  /**
   * @brief Pre-rasterize the geofence polygon into a flat boolean mask.
   * outside_fence_[j * size_x + i] is true if cell (i,j) is outside the fence.
   * This is called once per polygon change, not every update cycle.
   */
  void rasterizeMask();

  /**
   * @brief Point-in-polygon test using the ray-casting (crossing number) algorithm.
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @return true if (wx, wy) is inside the polygon
   */
  bool isPointInPolygon(double wx, double wy) const;

  /**
   * @brief Validate incoming dynamic parameter updates.
   * Rejects updates with invalid types or values.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply validated dynamic parameter updates.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  // Polygon state (consumed in updateBounds, protected by polygon_buffer_mutex_)
  /// Thread-safe buffer holding the next polygon to process
  std::shared_ptr<const geometry_msgs::msg::PolygonStamped> polygon_buffer_;
  mutable std::mutex polygon_buffer_mutex_;
  /// Set to true by the service callback when a new polygon is available
  std::atomic<bool> polygon_updated_{false};

  // Active geofence (only touched in updateBounds/updateCosts thread)
  /// Polygon vertices (in global_frame_)
  std::vector<double> polygon_x_;
  std::vector<double> polygon_y_;
  /// Pre-rasterized mask: outside_fence_[j * mask_size_x_ + i] == true → LETHAL
  std::vector<bool> outside_fence_;
  unsigned int mask_size_x_{0};
  unsigned int mask_size_y_{0};
  double mask_origin_x_{0.0};
  double mask_origin_y_{0.0};
  double mask_resolution_{0.0};

  /// Whether a valid geofence has ever been received
  bool has_fence_{false};

  // Parameters
  /// The global frame ID of the layered costmap
  std::string global_frame_;
  /// Whether to resize the costmap to fit the geofence (true for static, false for rolling)
  bool resize_to_fence_{true};

  // ROS interfaces
  /// The SetFence service server
  nav2::ServiceServer<nav2_msgs::srv::SetFence>::SharedPtr set_fence_service_;
  /// Dynamic parameter validation callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  /// Dynamic parameter post-set callback handle
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__GEOFENCE_LAYER_HPP_
