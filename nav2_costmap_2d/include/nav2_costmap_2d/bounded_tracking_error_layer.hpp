// Copyright (c) 2025 Berkan Tali
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

#ifndef NAV2_COSTMAP_2D__TRACKING_BOUNDS_ERROR_LAYER_HPP_
#define NAV2_COSTMAP_2D__TRACKING_BOUNDS_ERROR_LAYER_HPP_

#include <vector>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace nav2_costmap_2d
{
/**
 * @class nav2_costmap_2d::BoundedTrackingErrorLayer
 * @brief A costmap layer that creates a dynamic corridor (tracking bounds) around the planned path
 *        using tracking feedback, to restrict navigation to a safe region.
 *
 * This layer gets current path, tracking_feedback and dynamically updates the costmap
 * to create a corridor of configurable width around the path. The corridor adapts as the robot moves,
 * using look-ahead and step parameters to determine the segment of the path to use. The layer can be
 * enabled or disabled at runtime.
 */
class BoundedTrackingErrorLayer : public nav2_costmap_2d::Layer
{
public:
  /**
   * @brief Default constructor for BoundedTrackingErrorLayer.
   */
  BoundedTrackingErrorLayer() = default;

  /**
   * @brief Destructor for BoundedTrackingErrorLayer.
   */
  ~BoundedTrackingErrorLayer();

  /**
   * @brief Initializes the layer, setting up subscriptions and parameters.
   */
  virtual void onInitialize();

  /**
   * @brief Determines edges of region layer can change.
   * @param robot_x X position of the robot in world coordinates.
   * @param robot_y Y position of the robot in world coordinates.
   * @param robot_yaw Orientation of the robot in radians.
   * @param min_x Pointer to minimum X bound to update.
   * @param min_y Pointer to minimum Y bound to update.
   * @param max_x Pointer to maximum X bound to update.
   * @param max_y Pointer to maximum Y bound to update.
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);

  /**
   * @brief Creates obstacles to bound the robot.
   * @param master_grid Reference to the master costmap to update.
   * @param min_i Minimum X index of the region to update.
   * @param min_j Minimum Y index of the region to update.
   * @param max_i Maximum X index of the region to update.
   * @param max_j Maximum Y index of the region to update.
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Resets the layer state.
   */
  virtual void reset();

  /**
   * @brief Indicates whether the layer can be cleared.
   * @return Always returns false for this layer.
   */
  virtual bool isClearable() {return false;}

  /**
   * @brief Activates the layer, enabling subscriptions and updates.
   */
  virtual void activate();

  /**
   * @brief Deactivates the layer, disabling subscriptions and updates.
   */
  virtual void deactivate();

  /**
   * @brief Computes wall points for creating corridor boundaries from a path segment.
   * @param segment Path segment to generate wall points from.
   * @return Vector of wall point coordinates [x, y].
   */
  std::vector<std::vector<double>> getWallPoints(const nav_msgs::msg::Path & segment);

  /**
   * @brief Creates segment from current path. Length is decided by look_ahead parameter.
   * @note Caller must hold data_mutex_ before calling this method.
   * @return The path segment as a nav_msgs::msg::Path.
   */
  nav_msgs::msg::Path getPathSegment();

protected:
  /**
   * @brief Callback for path updates.
   * @param msg Incoming path message.
   */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Callback for tracking feedback updates.
   * @param msg Incoming tracking feedback message.
   */
  void trackingCallback(const nav2_msgs::msg::TrackingFeedback::SharedPtr msg);

  /**
   * @brief Callback for dynamic parameter updates.
   * @param parameters Vector of parameters being updated.
   * @return Result indicating success or failure of parameter update.
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  nav2_msgs::msg::TrackingFeedback last_tracking_feedback_;

private:
  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav2::Subscription<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_sub_;
  std::mutex data_mutex_;
  nav_msgs::msg::Path last_path_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  size_t temp_step_;
  int step_;
  double width_;
  double look_ahead_;
  bool enabled_;
  std::string path_topic_;
  std::string tracking_feedback_topic_;
  int wall_thickness_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__TRACKING_BOUNDS_ERROR_LAYER_HPP_
