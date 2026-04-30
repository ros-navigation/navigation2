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

#include "geometry_msgs/msg/pose_stamped.hpp"
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
 * @class nav2_costmap_2d::BoundedTrackingErrorLayer
 * @brief A costmap layer that imposes a penalty corridor around the planned path.
 *
 * On every costmap update cycle this layer finds the closest point on the current
 * path, extracts a look-ahead segment, and rasterizes high-cost walls along both
 * sides of that segment. The walls penalise the local trajectory controller when
 * the robot deviates beyond the configured corridor width without blocking the
 * global planner. Optionally, all free-space cells outside the corridor within
 * the update bounds can be elevated to corridor cost. All parameters are
 * dynamically reconfigurable at runtime.
 */
class BoundedTrackingErrorLayer : public nav2_costmap_2d::Layer
{
public:
  /**
   * @brief Constructor for BoundedTrackingErrorLayer.
   */
  BoundedTrackingErrorLayer() = default;

  /**
   * @brief Destructor for BoundedTrackingErrorLayer.
   */
  ~BoundedTrackingErrorLayer() = default;

  /**
   * @brief Initializes the layer, declares parameters and creates the path subscription.
   */
  void onInitialize() override;

  /**
   * @brief Activates the layer and registers parameter update callbacks.
   */
  void activate() override;

  /**
   * @brief Deactivates the layer and removes parameter update callbacks.
   */
  void deactivate() override;

  /**
   * @brief Resets the layer, clearing path state and marking as not current.
   */
  void reset() override;

  /**
   * @brief Updates resolution and frame ID from the layered costmap on resize.
   */
  void matchSize() override;

  /**
   * @brief Expands the update bounds to cover the corridor region around the robot.
   * @param robot_x Robot X position in world coordinates.
   * @param robot_y Robot Y position in world coordinates.
   * @param robot_yaw Robot yaw (unused).
   * @param min_x Minimum X bound to update.
   * @param min_y Minimum Y bound to update.
   * @param max_x Maximum X bound to update.
   * @param max_y Maximum Y bound to update.
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Writes corridor wall or fill costs into the master costmap.
   * @param master_grid Costmap to write into.
   * @param min_i Minimum X cell index of the update window.
   * @param min_j Minimum Y cell index of the update window.
   * @param max_i Maximum X cell index of the update window.
   * @param max_j Maximum Y cell index of the update window.
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Returns whether this layer can be cleared by the costmap clearing service.
   * @return Always true; clearing is safe since costs are recomputed every cycle,
   *         and allows recovery behaviors to give the robot room to maneuver out.
   */
  bool isClearable() override {return true;}

  /**
   * @brief Validate parameter updates before they are applied.
   * @param parameters Incoming parameter update list.
   * @return Result indicating success or failure with reason.
   */
  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply validated parameter updates to layer state.
   * @param parameters Validated parameter update list.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

protected:
  /** @brief Wall polygon components for left and right corridor boundaries. */
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
   * @brief Subscription callback that stores the latest global plan.
   * @param msg Incoming path message.
   */
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);

  /**
   * @brief Extract a look-ahead sub-path starting from a given index.
   * @param path Full path in costmap frame.
   * @param path_index Index of the closest pose on the path.
   * @param segment Output sub-path segment.
   */
  void getPathSegment(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    nav_msgs::msg::Path & segment);

  /**
   * @brief Compute wall polygon boundary points from a path segment.
   *
   * step_size_ is the pose-index stride used to estimate the local path direction
   * at each polygon vertex. A stride of 1 gives noisy normals on dense paths;
   * a larger value averages over more poses for smoother corridor boundaries.
   * @param segment Path segment in costmap frame.
   * @param walls Output wall polygons.
   */
  void getWallPolygons(const nav_msgs::msg::Path & segment, WallPolygons & walls);

  /**
   * @brief Apply the fill-outside-corridor mode: trace interior quads, add end-cap
   *        circles, guarantee the robot cell is interior, then flood-fill outside.
   * @param master_grid Costmap to write into.
   * @param robot_pose Current robot pose in costmap frame.
   * @param full_path Full transformed path in costmap frame.
   */
  void applyFillOutsideCorridor(
    nav2_costmap_2d::Costmap2D & master_grid,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & full_path);

  /**
   * @brief Rasterize corridor wall cells between inner and outer boundary polylines.
   * @param master_grid Costmap to write into.
   * @param inner_points Inner boundary polyline in world coordinates.
   * @param outer_points Outer boundary polyline in world coordinates.
   */
  void drawCorridorWalls(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<std::array<double, 2>> & inner_points,
    const std::vector<std::array<double, 2>> & outer_points);

  /**
   * @brief Record all cell indices inside the corridor interior into the index set.
   *
   * Always accumulates into the existing set. Callers are responsible for
   * resetting the relevant region of corridor_interior_mask_ before the first
   * call in each update cycle.
   * @param master_grid Costmap used for coordinate conversion.
   * @param walls Wall polygons defining the corridor interior.
   */
  void saveCorridorInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    const WallPolygons & walls);

  /**
   * @brief Mark all cells within a circle radius as corridor interior. That robot needs
   * for turning around
   *
   * @param master_grid Costmap used for coordinate conversion.
   * @param cx Circle center X in cell coordinates.
   * @param cy Circle center Y in cell coordinates.
   * @param r_sq Squared radius in cells.
   */
  void markCircleAsInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    int cx, int cy, int r_sq);

  /**
   * @brief Elevate all cells outside the corridor index set to corridor cost.
   * @param master_grid Costmap to write into.
   * @param min_i Fill area minimum X cell index.
   * @param min_j Fill area minimum Y cell index.
   * @param max_i Fill area maximum X cell index.
   * @param max_j Fill area maximum Y cell index.
   */
  void fillOutsideCorridor(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Reset path state and index tracker.
   */
  void resetState();

  size_t step_size_;
  double look_ahead_;
  double corridor_width_;
  int wall_thickness_;
  unsigned char corridor_cost_;
  int cost_write_mode_{0};

  double resolution_{0.0};
  std::string costmap_frame_;
  std::string robot_base_frame_;

  std::atomic<uint32_t> current_path_index_{0};
  std::vector<uint8_t> corridor_interior_mask_;

  // Previous fill bounding box in cell indices; -1 = no fill performed yet.
  int prev_fill_min_i_{-1};
  int prev_fill_min_j_{-1};
  int prev_fill_max_i_{-1};
  int prev_fill_max_j_{-1};

private:
  /** @brief A 2D costmap cell coordinate. */
  struct CellPoint
  {
    int x;
    int y;
  };

  /**
   * @brief Declare and load all layer parameters from the node.
   */
  void getParameters();

  /**
   * @brief Convert world coordinates to a CellPoint, clamping to map bounds if outside.
   * @param master_grid Costmap used for coordinate conversion.
   * @param wx World X coordinate.
   * @param wy World Y coordinate.
   * @param out Output cell coordinate.
   * @return True if the point is within map bounds, false if it was clamped.
   */
  bool worldToCell(
    const nav2_costmap_2d::Costmap2D & master_grid,
    double wx, double wy,
    CellPoint & out) const;

  /**
   * @brief Trace all 4 edges of a quad into the span buffers (p0→p1→p2→p3→p0).
   * @param p0 First vertex.
   * @param p1 Second vertex.
   * @param p2 Third vertex.
   * @param p3 Fourth vertex.
   * @param clamped_y_min Minimum Y index of the span buffer.
   * @param height Height of the span buffer.
   */
  void traceQuad(
    CellPoint p0, CellPoint p1, CellPoint p2, CellPoint p3,
    int clamped_y_min, int height);

  /**
   * @brief Fill a convex quadrilateral corridor quad using a span buffer approach.
   *
   * See https://en.wikipedia.org/wiki/Scanline_rendering
   * @param master_grid Costmap to write into.
   * @param inner0 First inner boundary cell.
   * @param inner1 Second inner boundary cell.
   * @param outer0 First outer boundary cell.
   * @param outer1 Second outer boundary cell.
   */
  void fillCorridorQuad(
    nav2_costmap_2d::Costmap2D & master_grid,
    CellPoint inner0,
    CellPoint inner1,
    CellPoint outer0,
    CellPoint outer1);

  /**
   * @brief Reset the corridor interior mask over the union of current and previous fill bounding boxes.
   * @param size_x Costmap width in cells.
   * @param reset_min_i Union bbox minimum X cell index.
   * @param reset_min_j Union bbox minimum Y cell index.
   * @param reset_max_i Union bbox maximum X cell index.
   * @param reset_max_j Union bbox maximum Y cell index.
   */
  void resetCorridorMask(
    unsigned int size_x,
    int reset_min_i, int reset_min_j,
    int reset_max_i, int reset_max_j);

  /**
   * @brief Iterate the full path, split into bbox-clipped sub-segments, and
   *        build the corridor interior mask for each.
   * @param master_grid Costmap used for coordinate conversion.
   * @param full_path Full transformed path in costmap frame.
   * @param fill_min_i Fill bbox minimum X cell index.
   * @param fill_min_j Fill bbox minimum Y cell index.
   * @param fill_max_i Fill bbox maximum X cell index.
   * @param fill_max_j Fill bbox maximum Y cell index.
   * @param extra_poses Extra poses beyond bbox boundary for geometric coverage.
   */
  void buildCorridorMask(
    nav2_costmap_2d::Costmap2D & master_grid,
    const nav_msgs::msg::Path & full_path,
    int fill_min_i, int fill_min_j,
    int fill_max_i, int fill_max_j,
    size_t extra_poses);

  /**
   * @brief Process a sub-segment of the path, filtering to the fill bbox margin
   *        and saving the corridor interior mask for each contiguous chunk.
   * @param master_grid Costmap used for coordinate conversion.
   * @param sub_segment Path sub-segment to process.
   * @param fill_min_i Fill bbox minimum X cell index.
   * @param fill_min_j Fill bbox minimum Y cell index.
   * @param fill_max_i Fill bbox maximum X cell index.
   * @param fill_max_j Fill bbox maximum Y cell index.
   * @param extra_poses Extra poses beyond bbox boundary for geometric coverage.
   */
  void flushSubSegment(
    nav2_costmap_2d::Costmap2D & master_grid,
    nav_msgs::msg::Path & sub_segment,
    int fill_min_i, int fill_min_j,
    int fill_max_i, int fill_max_j,
    size_t extra_poses);

  std::string path_topic_;
  tf2::Duration transform_tolerance_;

  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;

  std::mutex data_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr last_path_ptr_;

  nav_msgs::msg::Path segment_buffer_;
  nav_msgs::msg::Path full_transformed_path_buffer_;
  WallPolygons walls_buffer_;
  std::vector<int> span_x_min_buffer_;
  std::vector<int> span_x_max_buffer_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_
