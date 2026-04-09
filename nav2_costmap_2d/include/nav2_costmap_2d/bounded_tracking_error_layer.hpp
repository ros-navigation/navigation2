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

#include <array>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_util/robot_utils.hpp"


namespace nav2_costmap_2d
{

/**
 * @brief Separate wall polygon components for left and right corridor boundaries.
 *
 * Each wall is represented by an inner and outer polyline of 2D world-coordinate
 * points. Consecutive point pairs form convex quads that are rasterized into the
 * costmap by the span-buffer fill algorithm.
 */
struct WallPolygons
{

  std::vector<std::array<double, 2>> left_inner;

  std::vector<std::array<double, 2>> left_outer;

  std::vector<std::array<double, 2>> right_inner;

  std::vector<std::array<double, 2>> right_outer;

  /**
   * @brief Clear all boundary vectors and reserve capacity for reuse.
   * @param capacity Expected number of points per boundary.
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
 * @brief A costmap layer that imposes a penalty corridor around the planned path.
 *
 * On every costmap update cycle this layer finds the closest point on the current
 * path, extracts a look-ahead segment, and rasterizes high-cost walls along both
 * sides of that segment. The walls penalise the local trajectory controller when
 * the robot deviates beyond the configured corridor width without blocking the
 * global planner. All parameters are dynamically reconfigurable at runtime.
 */
class BoundedTrackingErrorLayer : public nav2_costmap_2d::Layer
{

public:
  /**
   * @brief Default constructor
   */
  BoundedTrackingErrorLayer() = default;

  /**
   * @brief Default destructor
   */
  ~BoundedTrackingErrorLayer() = default;

  /**
   * @brief Initializes the layer, setting up subscriptions and parameters.
   */
  void onInitialize() override;

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
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Creates obstacles to bound the robot.
   * @param master_grid Reference to the master costmap to update.
   * @param min_i Minimum X index of the region to update.
   * @param min_j Minimum Y index of the region to update.
   * @param max_i Maximum X index of the region to update.
   * @param max_j Maximum Y index of the region to update.
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Resets the layer state.
   */
  void reset() override;

  /**
   * @brief Indicates whether the layer can be cleared.
   * @return Always returns false for this layer.
   */
  bool isClearable() override {return false;}

  /**
   * @brief Activates the layer, enabling subscriptions and updates.
   */
  void activate() override;

  /**
   * @brief Deactivates the layer, disabling subscriptions and updates.
   */
  void deactivate() override;

  /**
   * @brief Match the size of the master costmap, caching resolution and frame.
   *
   * This layer does not own an internal costmap so there is nothing to resize.
   * Resolution and global frame ID are cached here so updateCosts and
   * getWallPolygons avoid repeated pointer dereferences on every update cycle.
   */
  void matchSize() override;

protected:
  /**
   * @brief Resets internal state by clearing the cached path and path index.
   */
  void resetState();

  /**
   * @brief Reads and validates all ROS parameters, populating member variables.
   */
  void getParameters();

  /**
   * @brief Stores the incoming path. All staleness and index checks are
   *        deferred to updateCosts() to keep the callback minimal.
   * @param msg Incoming path message.
   */
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief Computes separate wall polygons for left and right corridor boundaries.
   * @param segment Path segment to generate wall polygons from.
   * @param walls Output WallPolygons structure (reused buffer).
   */
  void getWallPolygons(const nav_msgs::msg::Path & segment, WallPolygons & walls);

  /**
   * @brief Extracts a path segment of length look_ahead_ starting at path_index.
   * @param path The full path to extract the segment from.
   * @param path_index Current position index on the path.
   * @param segment Output path segment (reused buffer).
   */
  void getPathSegment(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    nav_msgs::msg::Path & segment);

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

  /**
   * @brief Draws corridor walls using span buffer approach for complete fill.
   *
   * Iterates through paired inner and outer boundary points, treating each segment
   * pair as a convex quadrilateral and filling it completely using the span buffer
   * technique. This eliminates gaps on diagonals and ensures constant thickness.
   *
   * @param master_grid Reference to master grid for coordinate conversion.
   * @param inner_points Vector of inner boundary points.
   * @param outer_points Vector of outer boundary points.
   */
  void drawCorridorWalls(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<std::array<double, 2>> & inner_points,
    const std::vector<std::array<double, 2>> & outer_points);

private:
  /**
   * @brief A 2D cell coordinate in the costmap grid.
   */
  struct CellPoint
  {
    unsigned int x;
    unsigned int y;
  };

  /**
   * @brief Fills a corridor quad (convex quadrilateral) using span buffer approach.
   *
   * This function rasterizes a quad defined by four vertices (inner[i], inner[i+1],
   * outer[i+1], outer[i]) by tracing all four edges and building a span buffer that
   * tracks [x_min, x_max] for each y-row. Then fills all pixels between the boundaries.
   *
   * @param master_grid Reference to master grid for size and costmap array access.
   * @param inner0 Inner boundary point i.
   * @param inner1 Inner boundary point i+1.
   * @param outer0 Outer boundary point i.
   * @param outer1 Outer boundary point i+1.
   *
   * See https://en.wikipedia.org/wiki/Scanline_rendering
   */
  void fillCorridorQuad(
    nav2_costmap_2d::Costmap2D & master_grid,
    CellPoint inner0,
    CellPoint inner1,
    CellPoint outer0,
    CellPoint outer1);

  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;

  std::mutex data_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr last_path_ptr_;

  nav_msgs::msg::Path segment_buffer_;
  nav_msgs::msg::Path transformed_segment_buffer_;
  WallPolygons walls_buffer_;
  std::vector<int> span_x_min_buffer_;
  std::vector<int> span_x_max_buffer_;

protected:
  std::atomic<uint32_t> current_path_index_{0};

  std::string path_topic_;
  size_t step_size_;
  double look_ahead_;
  double corridor_width_;
  int wall_thickness_;
  unsigned char corridor_cost_;
  tf2::Duration transform_tolerance_;
  double resolution_{0.0};
  std::string costmap_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_
