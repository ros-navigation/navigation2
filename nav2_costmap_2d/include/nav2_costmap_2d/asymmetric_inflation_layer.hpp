/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Marc Blöchlinger
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/legacy_inflation_layer.hpp"  // for nav2_costmap_2d::CellData
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_costmap_2d
{

/**
 * @class AsymmetricInflationLayer
 * @brief Costmap layer that inflates obstacles asymmetrically relative to the
 *        global path, biasing the navigable corridor toward one side.
 *
 * Must be chained after nav2_costmap_2d::InflationLayer, which writes the
 * symmetric baseline. This layer classifies each lethal obstacle as left (+1),
 * right (-1), or neutral (0) relative to the path and uses a BFS with
 * asymmetric effective distances to raise costs on the disfavored side.
 * Costs are written with max(old, new) so they can only increase. Falls back
 * to a no-op when no path is available, the goal is nearby, or
 * asymmetry_factor is zero.
 */
class AsymmetricInflationLayer : public nav2_costmap_2d::Layer
{
public:
  AsymmetricInflationLayer();
  ~AsymmetricInflationLayer();

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
   * Stores the robot position for goal-proximity checks in updateCosts(). When
   * need_reinflation_ is set, expands to full-map bounds and clears the flag;
   * otherwise takes the union of the previous and incoming bounds padded by
   * inflation_radius_.
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
   * Assumes master_grid already holds the symmetric baseline from the upstream
   * InflationLayer. Runs asymmetric BFS when a valid path is available and
   * asymmetry_factor is non-zero; otherwise returns without modification.
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

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() override {return false;}

  /**
   * @brief Reset this costmap
   */
  void reset() override
  {
    matchSize();
    setCurrent(false);
  }

  typedef std::recursive_mutex mutex_t;

  /**
   * @brief Get the mutex of the inflation information
   */
  mutex_t * getMutex() {return access_;}

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
   * Obstacles whose nearest perpendicular distance exceeds neutral_threshold_
   * are returned as 0 (neutral -> symmetric inflation).
   *
   * @param cx             World x-coordinate of the obstacle cell (metres).
   * @param cy             World y-coordinate of the obstacle cell (metres).
   * @param candidates     Segment indices whose padded AABB may contain (cx, cy).
   * @param local_path_pts Path points in costmap-frame order; consecutive pairs form segments.
   * @return +1 (left of path), -1 (right of path), or 0 (neutral / beyond influence radius).
   */
  int8_t computeObstacleSide(
    double cx, double cy,
    const std::vector<size_t> & candidates,
    const std::vector<std::pair<double, double>> & local_path_pts);

  /**
   * @brief Compute the asymmetric effective distance (in cells) for BFS priority.
   *
   * Preserves physical distance inside the inscribed radius to keep the collision
   * core symmetric. It scales the excess distance by (1 - asymmetry_factor * path_side).
   * A scale < 1 shrinks the effective distance so that side's BFS wave expands
   * earlier and claims more cells, raising costs further into contested space.
   * A scale > 1 does the opposite.
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

    double excess_dist = physical_dist - inscribed_radius_cells;
    double scale = std::max(0.0, 1.0 - asymmetry_factor_ * path_side);
    return inscribed_radius_cells + excess_dist * scale;
  }

  /**
   * @brief Compute an exponential-decay cost for a distance expressed in cells.
   * @param distance Distance from the source obstacle cell, in cells.
   * @return LETHAL_OBSTACLE at 0, INSCRIBED_INFLATED_OBSTACLE - 1 within the inscribed
   *         radius, exponentially decaying to 0 beyond it.
   */
  inline unsigned char computeCost(double distance) const
  {
    if (distance == 0) {
      return LETHAL_OBSTACLE;
    }
    if (distance * resolution_ <= inscribed_radius_) {
      return INSCRIBED_INFLATED_OBSTACLE;
    }
    double factor = exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
    return static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }

  /**
   * @brief Lookup pre-computed Euclidean distance between two cells.
   * @param mx X coordinate of the current cell.
   * @param my Y coordinate of the current cell.
   * @param src_x X coordinate of the source obstacle cell.
   * @param src_y Y coordinate of the source obstacle cell.
   * @return Euclidean distance in cells.
   */
  inline double distanceLookup(
    unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  /**
   * @brief Convert a world distance (meters) to a cell distance
   */
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  /**
   * @brief Pre-compute distance and cost caches and size the BFS priority queue
   */
  void computeCaches();

  /**
   * @brief Enqueue a cell into the BFS priority queue at the appropriate bin
   */
  inline size_t enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
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
  double inflation_radius_;
  double inscribed_radius_;
  double cost_scaling_factor_;
  double asymmetry_factor_;
  bool inflate_around_unknown_;
  /// Distance to goal where asymmetry disables to prevent docking oscillations
  double goal_distance_threshold_;
  /// Distance from path beyond which obstacles are treated symmetrically
  double neutral_threshold_;
  std::string plan_topic_;

  // --- State ---
  double current_robot_x_{0.0};
  double current_robot_y_{0.0};
  double last_min_x_;
  double last_min_y_;
  double last_max_x_;
  double last_max_y_;
  bool need_reinflation_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_{0};
  double resolution_;
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

  // --- Synchronization ---
  mutex_t * access_;

  // --- Dynamic parameter handlers (split: validate + update) ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ASYMMETRIC_INFLATION_LAYER_HPP_
