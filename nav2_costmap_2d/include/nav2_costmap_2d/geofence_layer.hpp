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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/srv/set_fence.hpp"
#include "nav2_ros_common/service_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"

namespace nav2_costmap_2d
{

/**
 * @class GeofenceLayer
 * @brief Constrains a non-rolling global costmap to a runtime-configurable polygon.
 *
 * Cells outside the active polygon are lethal; cells inside are free.
 * The fence is set or cleared via the service <plugin_name>/set_fence
 * (nav2_msgs/srv/SetFence).  An empty polygon clears the active fence and
 * restores the costmap to its initial dimensions.
 *
 * When resize_to_fence is true this layer must be listed first in plugins,
 * as LayeredCostmap::resizeMap() resets all layer contributions.
 *
 * fence_polygon is a startup-only parameter; use the service for runtime
 * updates.  Rolling costmaps are rejected at initialization.
 */
class GeofenceLayer : public CostmapLayer
{
public:
  /**
   * @brief Geofence Layer constructor
   */
  GeofenceLayer() = default;

  /**
   * @brief Geofence Layer destructor
   */
  ~GeofenceLayer() override = default;

  /** @brief Initialize the layer and advertise the fence service. */
  void onInitialize() override;

  /** @brief Register dynamic parameter callbacks. */
  void activate() override;

  /** @brief Remove dynamic parameter callbacks. */
  void deactivate() override;

  /** @brief Mark the layer dirty so the fence is re-applied next update. */
  void reset() override;

  /**
   * @brief Geofence data is not clearable by recovery actions.
   * @return False
   */
  bool isClearable() override {return false;}

  /** @brief Match layer size to master costmap and mark dirty if fence is active. */
  void matchSize() override;

  /**
   * @brief Expand update bounds when the fence changes.
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x Minimum x update bound
   * @param min_y Minimum y update bound
   * @param max_x Maximum x update bound
   * @param max_y Maximum y update bound
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Merge the rasterized fence into the master costmap.
   * @param master_grid Master costmap grid
   * @param min_i Minimum x cell index
   * @param min_j Minimum y cell index
   * @param max_i Maximum x cell index
   * @param max_j Maximum y cell index
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

protected:
  /** @brief Read layer parameters and load an optional startup fence. */
  void getParameters();

  /**
   * @brief Handle a fence update or clear request.
   * @param request Fence request; empty polygon clears the fence
   * @param response Result indicating success or rejection reason
   */
  void setFenceCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response);

  /**
   * @brief Validate dynamic parameter updates.
   * @param parameters Parameters to validate
   * @return Validation result
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply validated dynamic parameter updates.
   * @param parameters Updated parameters
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  /// @brief Buffered polygon awaiting processing
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_buffer_;
  /// @brief Active polygon vertices in global frame
  std::vector<geometry_msgs::msg::Point> polygon_points_;
  /// @brief Global frame for the costmap
  std::string global_frame_;
  /// @brief Transform lookup tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Flag indicating pending fence update
  bool has_updated_data_{false};
  /// @brief Flag indicating active fence is set
  bool has_fence_{false};
  /// @brief Whether to resize costmap to fence bounds
  bool resize_to_fence_{true};

  /// @brief Service server for setting or clearing fence
  nav2::ServiceServer<nav2_msgs::srv::SetFence>::SharedPtr set_fence_service_;

  /// @brief Post-set dynamic parameter callback handle
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  /// @brief On-set dynamic parameter validation callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;

private:
  /**
   * @brief Store a polygon for processing by the costmap update thread.
   * @param polygon Polygon in the global frame; pass a PolygonStamped with no
   *   points to clear the active fence
   */
  void bufferPolygon(const geometry_msgs::msg::PolygonStamped & polygon);

  /** @brief Apply a buffered fence change and optionally resize the costmap. */
  void processFence();

  /**
   * @brief Fill the polygon interior as free and raytrace its perimeter as lethal.
   * @param min_x Polygon AABB minimum x (world frame)
   * @param min_y Polygon AABB minimum y (world frame)
   * @param max_x Polygon AABB maximum x (world frame)
   * @param max_y Polygon AABB maximum y (world frame)
   */
  void rasterizeFence(double min_x, double min_y, double max_x, double max_y);

  unsigned int initial_size_x_{0};  ///< @brief Initial costmap width in cells
  unsigned int initial_size_y_{0};  ///< @brief Initial costmap height in cells
  double initial_origin_x_{0.0};    ///< @brief Initial costmap origin x in meters
  double initial_origin_y_{0.0};    ///< @brief Initial costmap origin y in meters
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__GEOFENCE_LAYER_HPP_
