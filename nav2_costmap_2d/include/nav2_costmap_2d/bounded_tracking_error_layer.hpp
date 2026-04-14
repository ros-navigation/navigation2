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
#include <unordered_set>
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
 * global planner. Optionally, all free-space cells outside the corridor within
 * the update bounds can be elevated to corridor cost. All parameters are
 * dynamically reconfigurable at runtime.
 */
class BoundedTrackingErrorLayer : public nav2_costmap_2d::Layer
{

public:
  BoundedTrackingErrorLayer() = default;
  ~BoundedTrackingErrorLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

  bool isClearable() override {return false;}

  void activate() override;

  void deactivate() override;

  void matchSize() override;

protected:
  struct CellPoint
  {
    unsigned int x;
    unsigned int y;
  };

  void resetState();
  void getParameters();
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void getWallPolygons(const nav_msgs::msg::Path & segment, WallPolygons & walls);
  void getPathSegment(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    nav_msgs::msg::Path & segment);

  rcl_interfaces::msg::SetParametersResult
  validateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters);

  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  void drawCorridorWalls(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<std::array<double, 2>> & inner_points,
    const std::vector<std::array<double, 2>> & outer_points);

  void saveCorridorInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    const WallPolygons & walls,
    bool accumulate = false);

  void markCircleAsInterior(
    nav2_costmap_2d::Costmap2D & master_grid,
    int cx, int cy, int r_sq);

  void fillOutsideCorridor(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  void traceEdge(CellPoint p0, CellPoint p1, int clamped_y_min, int height);

  void getFillArea(
    nav2_costmap_2d::Costmap2D & master_grid,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    int & fill_min_i, int & fill_min_j, int & fill_max_i, int & fill_max_j);



private:
  /**
   * @brief Fills a corridor quad (convex quadrilateral) using span buffer approach.
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
  std::unordered_set<unsigned int> corridor_index_set_;

protected:
  std::atomic<uint32_t> current_path_index_{0};

  std::string path_topic_;
  size_t step_size_;
  double look_ahead_;
  double corridor_width_;
  int wall_thickness_;
  unsigned char corridor_cost_;
  bool fill_outside_corridor_{false};
  tf2::Duration transform_tolerance_;
  double resolution_{0.0};
  std::string costmap_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__BOUNDED_TRACKING_ERROR_LAYER_HPP_
