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

#ifndef NAV2_COSTMAP_2D__FENCE_LAYER_HPP_
#define NAV2_COSTMAP_2D__FENCE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/srv/set_fence.hpp"
#include "geometry_msgs/msg/point32.hpp"

namespace nav2_costmap_2d
{

/**
 * @class FenceLayer
 * @brief A costmap layer that defines an operational zone (fence).
 * Cells outside the fence polygon are marked as LETHAL_OBSTACLE,
 * preventing the planner from routing outside the zone.
 * The fence can be set via a ROS parameter or updated dynamically
 * at run-time through a SetFence service.
 */
class FenceLayer : public Layer
{
public:
  /**
   * @brief FenceLayer constructor
   */
  FenceLayer();

  /**
   * @brief FenceLayer destructor
   */
  virtual ~FenceLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

  /**
   * @brief Activate this layer
   */
  void activate() override;

  /**
   * @brief Deactivate this layer
   */
  void deactivate() override;

  /**
   * @brief Reset this costmap
   */
  void reset() override;

  /**
   * @brief If clearing operations should be processed on this layer or not.
   * The fence is a hard constraint, not sensor data, so it is not clearable.
   */
  bool isClearable() override {return false;}

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
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
   * @param request_header The request header
   * @param request The SetFence request containing the polygon
   * @param response The SetFence response
   */
  void setFenceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SetFence::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetFence::Response> response);

  /**
   * @brief Test whether a world-coordinate point lies inside the fence polygon.
   * Uses the ray-casting (crossing number) algorithm.
   * @param wx The x world coordinate
   * @param wy The y world coordinate
   * @return True if the point is inside the fence polygon
   */
  bool isInsideFence(double wx, double wy) const;

  /**
   * @brief Parse a flat list of doubles [x1,y1,x2,y2,...] into a polygon
   * @param raw The flat parameter vector
   * @param polygon Output polygon vertices
   * @return True if parsing succeeded (even number of elements, >= 3 vertices)
   */
  bool parsePolygonParameter(
    const std::vector<double> & raw,
    std::vector<geometry_msgs::msg::Point32> & polygon) const;

  /// The fence polygon vertices (in the global costmap frame)
  std::vector<geometry_msgs::msg::Point32> fence_polygon_;

  /// Mutex protecting fence_polygon_
  mutable std::mutex fence_mutex_;

  /// Whether a valid fence has been set
  bool has_fence_{false};

  /// Whether the fence changed and bounds need to be expanded
  bool fence_changed_{false};

  /// The global frame of the costmap
  std::string global_frame_;

  /// The SetFence service server
  nav2::ServiceServer<nav2_msgs::srv::SetFence>::SharedPtr set_fence_service_;

  /// Dynamic parameters handler
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FENCE_LAYER_HPP_
