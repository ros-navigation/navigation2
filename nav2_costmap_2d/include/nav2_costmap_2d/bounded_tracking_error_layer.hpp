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

#ifndef NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_
#define NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_

#include <atomic>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <memory>
#include <mutex>
#include <string>
#include <algorithm>
#include <array>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/tracking_feedback.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "tf2/time.hpp"
#include "nav2_util/line_iterator.hpp"


namespace nav2_costmap_2d
{

/**
 * @brief Structure holding separate wall polygon components for left, right walls
 * with inner and outer boundaries for span buffer approach.
 * Optimized to use std::array for fixed-size 2D points instead of vector<vector>.
 */
struct WallPolygons
{
  std::vector<std::array<double, 2>> left_inner;
  std::vector<std::array<double, 2>> left_outer;
  std::vector<std::array<double, 2>> right_inner;
  std::vector<std::array<double, 2>> right_outer;

  /**
   * @brief Clear all vectors and reserve capacity for reuse
   * @param capacity Expected number of points
   */
  void clearAndReserve(size_t capacity)
  {
    left_inner.clear();
    left_outer.clear();
    right_inner.clear();
    right_outer.clear();
    left_inner.reserve(capacity);
    left_outer.reserve(capacity);
    right_inner.reserve(capacity);
    right_outer.reserve(capacity);
  }


};

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
  ~BoundedTrackingErrorLayer() = default;

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
   * @brief Computes separate wall polygons for left and right corridor boundaries.
   * @param segment Path segment to generate wall polygons from.
   * @param walls Output WallPolygons structure (reused buffer).
   */
  void getWallPolygons(const nav_msgs::msg::Path & segment, WallPolygons & walls);

  /**
   * @brief Creates segment from current path. Length is decided by look_ahead parameter.
   * @param path The full path to extract segment from.
   * @param path_index Current position index on the path.
   * @param segment Output path segment (reused buffer).
   */
  void getPathSegment(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    nav_msgs::msg::Path & segment);

protected:
  /**
   * @brief Resets internal state by clearing path, tracking feedback, and goal.
   * @note Caller must hold data_mutex_ before calling this method.
   */
  void resetState();

  /**
   * @brief Callback for path updates.
   * @param msg Incoming path message.
   */
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief Callback for tracking feedback updates.
   * @param msg Incoming tracking feedback message.
   */
  void trackingCallback(const nav2_msgs::msg::TrackingFeedback::ConstSharedPtr msg);

  /**
   * @brief Callback to validate parameter updates before they are applied.
   * @param parameters Vector of parameters being validated.
   * @return Result indicating success or failure of parameter validation.
   */
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Callback to update internal state after parameters have been successfully set.
   * @param parameters Vector of parameters that were updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  std::atomic<uint32_t> current_path_index_{0};

private:
  /**
   * @brief Fills a corridor quad (convex quadrilateral) using span buffer approach.
   *
   * This function rasterizes a quad defined by four vertices (inner[i], inner[i+1],
   * outer[i+1], outer[i]) by tracing all four edges and building a span buffer that
   * tracks [x_min, x_max] for each y-row. Then fills all pixels between the boundaries.
   *
   * @param costmap Pointer to the costmap character array to modify.
   * @param size_x Width of the costmap in cells.
   * @param size_y Height of the costmap in cells.
   * @param inner_x0 Inner boundary point i, X coordinate.
   * @param inner_y0 Inner boundary point i, Y coordinate.
   * @param inner_x1 Inner boundary point i+1, X coordinate.
   * @param inner_y1 Inner boundary point i+1, Y coordinate.
   * @param outer_x0 Outer boundary point i, X coordinate.
   * @param outer_y0 Outer boundary point i, Y coordinate.
   * @param outer_x1 Outer boundary point i+1, X coordinate.
   * @param outer_y1 Outer boundary point i+1, Y coordinate.
   */
  void fillCorridorQuad(
    unsigned char * costmap,
    unsigned int size_x,
    unsigned int size_y,
    unsigned int inner_x0,
    unsigned int inner_y0,
    unsigned int inner_x1,
    unsigned int inner_y1,
    unsigned int outer_x0,
    unsigned int outer_y0,
    unsigned int outer_x1,
    unsigned int outer_y1);

  /**
   * @brief Draws corridor walls using span buffer approach for complete fill.
   *
   * Iterates through paired inner and outer boundary points, treating each segment
   * pair as a convex quadrilateral and filling it completely using the span buffer
   * technique. This eliminates gaps on diagonals and ensures constant thickness.
   *
   * @param costmap Pointer to the costmap character array to modify.
   * @param size_x Width of the costmap in cells.
   * @param size_y Height of the costmap in cells.
   * @param master_grid Reference to master grid for coordinate conversion.
   * @param inner_points Vector of inner boundary points.
   * @param outer_points Vector of outer boundary points.
   */
  void drawCorridorWalls(
    unsigned char * costmap,
    unsigned int size_x,
    unsigned int size_y,
    const nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<std::array<double, 2>> & inner_points,
    const std::vector<std::array<double, 2>> & outer_points);

  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav2::Subscription<nav2_msgs::msg::TrackingFeedback>::SharedPtr tracking_feedback_sub_;
  std::mutex data_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr last_path_ptr_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;

  size_t step_size_;
  int step_;
  double corridor_width_;
  double look_ahead_;

  std::string path_topic_;
  std::string tracking_feedback_topic_;
  unsigned char corridor_cost_;
  int wall_thickness_;
  tf2::Duration transform_tolerance_;
  double resolution_;

  // Reusable buffers to avoid repeated allocations
  nav_msgs::msg::Path segment_buffer_;
  nav_msgs::msg::Path transformed_segment_buffer_;
  WallPolygons walls_buffer_;
  std::vector<int> span_x_min_buffer_;
  std::vector<int> span_x_max_buffer_;

};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_
