// Copyright (c) 2026, Marc Blöchlinger
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

#ifndef NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_
#define NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_

#include <algorithm>
#include <vector>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_costmap_2d
{

struct AsymmetricPathSegment
{
  std::pair<double, double> start;
  std::pair<double, double> end;
};

/**
 * @class AsymmetricInflationLayer
 * @brief Costmap layer that inflates obstacles asymmetrically relative to the
 *        global path, biasing the navigable corridor toward one side.
 *
 * Inherits the distance-transform based InflationLayer, which writes the
 * symmetric baseline first. This layer then classifies each lethal obstacle as
 * left (+1), right (-1), or neutral (0) relative to the path and runs a second
 * distance transform seeded from only the disfavored-side boundary cells.
 * Falls back to the inherited symmetric inflation when no path is available,
 * the goal is nearby, or the per-side scaling factors are equal.
 */
class AsymmetricInflationLayer : public nav2_costmap_2d::InflationLayer
{
public:
  AsymmetricInflationLayer();
  ~AsymmetricInflationLayer() override = default;

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

  /**
   * @brief Activate the layer; registers the parameter validation/update callbacks.
   */
  void activate() override;

  /**
   * @brief Deactivate the layer; removes the parameter validation/update callbacks.
   */
  void deactivate() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions.
   *
   * Stores the robot position for goal-proximity checks in updateCosts(), then
   * delegates the actual inflation bound expansion to InflationLayer.
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
   * @brief Update the costs in the master costmap.
   *
   * First writes the inherited symmetric inflation baseline. Runs a second
   * distance-transform pass seeded from disfavored-side boundary cells when a
   * valid path is available and the two per-side decay rates differ.
   * Otherwise leaves the symmetric baseline unchanged.
   * @param master_grid The master costmap grid to update
   * @param min_i X min cell index of the window to update
   * @param min_j Y min cell index of the window to update
   * @param max_i X max cell index of the window to update
   * @param max_j Y max cell index of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

protected:
  /**
   * @brief Process updates on footprint changes to the inflation layer
   */
  void onFootprintChanged() override;

  /**
   * @brief Callback for incoming global path messages
   */
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Extract global path segments that overlap the local costmap window.
   *
   * Transforms the path into the costmap frame via TF2. Returns an empty
   * vector when the robot is within goal_distance_threshold_ of the goal,
   * which disables asymmetry near the goal to prevent docking oscillations.
   * @param master_grid Reference costmap used to filter points to the current window.
   * @return Path segments in costmap-frame world coordinates; empty disables asymmetry.
   */
  std::vector<AsymmetricPathSegment> extractLocalPathSegments(
    nav2_costmap_2d::Costmap2D & master_grid);

  /**
   * @brief Classify an obstacle cell as left (+1), right (-1), or neutral (0)
   *        relative to the closest path segment.
   *
   * For each candidate segment, performs an AABB rejection pass followed by an
   * exact perpendicular-distance and cross-product computation to determine side.
   * Obstacles whose nearest perpendicular distance exceeds the inflation radius
   * are returned as 0 (neutral -> symmetric inflation).
   *
   * @param cx             World x-coordinate of the obstacle cell (metres).
   * @param cy             World y-coordinate of the obstacle cell (metres).
   * @param candidates     Segment indices whose padded AABB may contain (cx, cy).
   * @param local_path_segments Original path segments in costmap-frame world coordinates.
   * @return +1 (left of path), -1 (right of path), or 0 (neutral / beyond inflation radius).
   */
  int8_t computeObstacleSide(
    double cx, double cy,
    const std::vector<size_t> & candidates,
    const std::vector<AsymmetricPathSegment> & local_path_segments);

  /**
   * @brief Build a spatial hash mapping 2D bucket keys to path segment indices.
   *
   * Uses inflation_radius_ as the bucket size so each bucket spans exactly one
   * inflation radius. Each segment is inserted into every bucket whose padded
   * AABB it overlaps, enabling O(1) nearest-segment queries during the disfavored-cell seeding phase.
   * @param local_path_segments Original path segments in costmap-frame world coordinates.
   * @return Hash map from packed (bucket_x, bucket_y) key to segment index list.
   */
  std::unordered_map<uint64_t, std::vector<size_t>>
  buildPathSpatialHash(
    const std::vector<AsymmetricPathSegment> & local_path_segments);

  /**
   * @brief Build a distance map seeded from disfavored-side obstacle boundary cells.
   *
   * Initialises a MatrixXfRM (roi_height × roi_width) to DT_INF, then sets
   * disfavored boundary cells to 0.0f. The caller passes the result to
   * DistanceTransform::distanceTransform2D() then applyInflation().
   * @param master_grid Costmap used for coordinate and cost lookups.
   * @param roi_min_i Left edge of the padded ROI (cells).
   * @param roi_min_j Bottom edge of the padded ROI (cells).
   * @param roi_width Width of the padded ROI (cells).
   * @param roi_height Height of the padded ROI (cells).
   * @param spatial_hash Segment lookup structure from buildPathSpatialHash().
   * @param local_path_segments Original path segments in costmap-frame world coordinates.
   * @return Distance map with 0.0f at disfavored seeds and DT_INF elsewhere.
   */
  MatrixXfRM seedDistanceMap(
    nav2_costmap_2d::Costmap2D & master_grid,
    int roi_min_i, int roi_min_j, int roi_width, int roi_height,
    const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
    const std::vector<AsymmetricPathSegment> & local_path_segments);

  /**
   * @brief Apply disfavored-side costs from a distance map with max(old, new) semantics.
   *
   * Mirrors InflationLayer::applyInflation() but uses cost_lut_disfavored_
   * (built with c_side) instead of the parent's cost_lut_ (built with c_max).
   * Costs are applied only within the originally requested update window.
   * @param master_array Raw costmap data pointer.
   * @param distance_map Result of distanceTransform2D() on the disfavored seed map.
   * @param min_i Left edge of the write window.
   * @param min_j Bottom edge of the write window.
   * @param max_i Right edge of the write window.
   * @param max_j Top edge of the write window.
   * @param roi_min_i Left edge of the padded ROI used to build distance_map.
   * @param roi_min_j Bottom edge of the padded ROI used to build distance_map.
   * @param size_x Full costmap width (cells), for index arithmetic.
   */
  void applyInflation(
    unsigned char * master_array,
    const MatrixXfRM & distance_map,
    int min_i, int min_j, int max_i, int max_j,
    int roi_min_i, int roi_min_j,
    unsigned int size_x);

  /**
   * @brief Pre-compute cost_lut_disfavored_ using c_side (the smaller per-side scaling factor)
   */
  void computeAsymmetricCaches();

  /**
   * @brief Validate parameter updates (pre-set callback). Returns success/failure
   * without mutating any state.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates (post-set callback) after they have been validated.
   * Recomputes caches when geometry parameters change.
   */
  void updateParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // --- Parameters ---
  /// Exponential decay rate for cells on the LEFT side of the path
  double cost_scaling_factor_left_;
  /// Exponential decay rate for cells on the RIGHT side of the path
  double cost_scaling_factor_right_;
  /// Distance to goal where asymmetry disables to prevent docking oscillations
  double goal_distance_threshold_;
  std::string plan_topic_;

  // --- State ---
  double current_robot_x_{0.0};
  double current_robot_y_{0.0};

  // --- Asymmetric Lookup Table ---
  /// Cost LUT for the disfavored side, built with c_side (the smaller per-side scaling factor)
  std::vector<unsigned char> cost_lut_disfavored_;

  // --- Path subscription ---
  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr latest_global_path_;
  std::optional<geometry_msgs::msg::TransformStamped> latest_path_transform_;
  std::mutex path_mutex_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_
