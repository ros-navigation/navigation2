/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Marc Blöchlinger
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marc Blöchlinger
 *         Eitan Marder-Eppstein (Original author)
 *         David V. Lu!! (Original author)
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_
#define NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_

#include <algorithm>
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/legacy_inflation_layer.hpp"  // for nav2_costmap_2d::CellData
#include "nav_msgs/msg/path.hpp"

namespace nav2_costmap_2d
{

/**
 * @class AsymmetricInflationLayer
 * @brief Costmap layer that inflates obstacles asymmetrically relative to the
 *        global path, biasing the navigable corridor toward one side.
 *
 * Inherits the distance-transform based InflationLayer, which writes the
 * symmetric baseline first. This layer then classifies each lethal obstacle as
 * left (+1), right (-1), or neutral (0) relative to the path and uses a BFS
 * with per-side exponential decay rates (`cost_scaling_factor_left` and
 * `cost_scaling_factor_right`) to raise costs on the side with the smaller
 * decay rate (= longer reach). Costs are written with max(old, new) so they can
 * only increase. Falls back to the inherited symmetric inflation when no path
 * is available, the goal is nearby, or the per-side scaling factors are equal.
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
   * First writes the inherited symmetric inflation baseline. Runs the
   * asymmetric BFS overlay when a valid path is available and the two per-side
   * decay rates differ; otherwise leaves the symmetric baseline unchanged.
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
   * @brief Extract the global path points that fall inside the local costmap window.
   *
   * Transforms the path into the costmap frame via TF2. Returns an empty
   * vector when the robot is within goal_distance_threshold_ of the goal,
   * which disables asymmetry near the goal to prevent docking oscillations.
   * @param master_grid Reference costmap used to filter points to the current window.
   * @return Path points in costmap-frame world coordinates; empty disables asymmetry.
   */
  std::vector<std::pair<double, double>> extractLocalPath(
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
   * @param local_path_pts Path points in costmap-frame order; consecutive pairs form segments.
   * @return +1 (left of path), -1 (right of path), or 0 (neutral / beyond inflation radius).
   */
  int8_t computeObstacleSide(
    double cx, double cy,
    const std::vector<size_t> & candidates,
    const std::vector<std::pair<double, double>> & local_path_pts);

  /**
   * @brief Build a spatial hash mapping 2D bucket keys to path segment indices.
   *
   * Uses inflation_radius_ as the bucket size so each bucket spans exactly one
   * inflation radius. Each segment is inserted into every bucket whose padded
   * AABB it overlaps, enabling O(1) nearest-segment queries during the BFS seed phase.
   * @param local_path_pts Path waypoints in costmap-frame world coordinates.
   * @return Hash map from packed (bucket_x, bucket_y) key to segment index list.
   */
  std::unordered_map<uint64_t, std::vector<size_t>>
  buildPathSpatialHash(
    const std::vector<std::pair<double, double>> & local_path_pts);

  /**
   * @brief Seed the asymmetric BFS by classifying boundary obstacle cells.
   *
   * Resets seen_, then iterates lethal/unknown obstacle cells in the provided
   * window. Interior cells (no traversable 4-connected neighbour) are
   * pre-marked seen and skipped. Boundary cells are classified via
   * computeObstacleSide and non-neutral cells are pushed into
   * inflation_cells_[0] to seed the BFS.
   * @param master_grid Costmap used for coordinate lookups.
   * @param min_i X lower bound of the update window.
   * @param min_j Y lower bound of the update window.
   * @param max_i X upper bound of the update window.
   * @param max_j Y upper bound of the update window.
   * @param spatial_hash Segment lookup structure from buildPathSpatialHash().
   * @param local_path_pts Path waypoints in costmap-frame world coordinates.
   */
  void seedAsymmetricBFS(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
    const std::vector<std::pair<double, double>> & local_path_pts);

  /**
   * @brief Run Dial's Algorithm BFS to propagate asymmetric inflation costs.
   *
   * Expands the BFS wave from seeds placed by seedAsymmetricBFS(). Each cell's
   * effective distance is scaled by its path-side factor. Costs are written with
   * max(old, new) so this layer can only raise the symmetric baseline.
   * @param master_grid Costmap to write costs into.
   * @param min_i X lower bound of the update window.
   * @param min_j Y lower bound of the update window.
   * @param max_i X upper bound of the update window.
   * @param max_j Y upper bound of the update window.
   */
  void runAsymmetricBFS(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Compute the asymmetric effective distance (in cells) for BFS priority.
   *
   * Preserves physical distance inside the inscribed radius to keep the collision
   * core symmetric. Beyond the inscribed radius the excess distance is scaled by
   * `c_side / cost_scaling_factor_`, where `c_side` is the side's cost_scaling_factor.
   * Distance for neutral cells doesn't get scaled and returns `eff_dist = physical_dist`.
   * @param physical_dist Euclidean distance from the source obstacle cell, in cells.
   * @param path_side +1 (left of path), -1 (right of path), or 0 (neutral).
   * @return Effective distance in cells.
   */
  inline double getEffectiveDistance(double physical_dist, int8_t path_side) const
  {
    const double inscribed_radius_cells = inscribed_radius_ / resolution_;
    if (physical_dist <= inscribed_radius_cells) {
      return physical_dist;
    }

    const double scale =
      (path_side > 0) ? cost_scaling_factor_left_ / cost_scaling_factor_ :
      (path_side < 0) ? cost_scaling_factor_right_ / cost_scaling_factor_ :
      1.0;  // Neutral cells are unaffected by asymmetry

    return inscribed_radius_cells + (physical_dist - inscribed_radius_cells) * scale;
  }

  /**
   * @brief Lookup pre-computed Euclidean distance between two cells.
   * @param mx X coordinate of the current cell.
   * @param my Y coordinate of the current cell.
   * @param src_x X coordinate of the source obstacle cell.
   * @param src_y Y coordinate of the source obstacle cell.
   * @return Euclidean distance in cells.
   */
  inline double asymmetricDistanceLookup(
    unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  /**
   * @brief Pre-compute distance and cost caches and size the BFS priority queue
   */
  void computeAsymmetricCaches();

  /**
   * @brief Enqueue a cell into the BFS priority queue at the appropriate bin
   */
  inline size_t enqueueAsymmetric(
    unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y, int8_t path_side, size_t current_bin);

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
  unsigned int cached_cell_inflation_radius_{0};
  unsigned int cache_length_;

  // --- BFS data structures ---
  /// Priority queue bins, indexed by floor(effective_distance * kEffDistPrecision)
  std::vector<std::vector<CellData>> inflation_cells_;
  std::vector<bool> seen_;
  std::vector<double> cached_distances_;
  /// Pre-computed exponential costs indexed by effective distance bin
  std::vector<unsigned char> cached_costs_;
  /// Maps lethal obstacles to their left/right/neutral identity
  std::vector<int8_t> obstacle_side_grid_;

  /// Number of priority-queue bins per cell of effective distance.
  /// Higher values give finer BFS priority ordering (bin width = 1/20 cell
  /// ≈ 0.05 cells) at the cost of a larger `inflation_cells_` vector.
  static constexpr double kEffDistPrecision = 20.0;

  // --- Path subscription ---
  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path::SharedPtr latest_global_path_;
  std::mutex path_mutex_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_
