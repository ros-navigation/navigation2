// Copyright 2026 Duatic AG
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
 * @brief A costmap layer that boosts obstacle inflation asymmetrically relative
 *        to the global path.
 *
 * This layer must be chained **after** the standard nav2_costmap_2d::InflationLayer.
 * That upstream layer is responsible for writing the symmetric baseline cost
 * field; this layer then runs a single BFS pass that, for obstacles near the
 * path, can raise costs on the "disfavored" side while leaving the "favored"
 * side untouched.
 *
 * Mechanism: each lethal obstacle is classified as left (+1), right (-1) or
 * neutral (0) relative to the global path.  The BFS priority queue is indexed
 * by an asymmetric effective distance so that waves from the disfavored side
 * expand earlier and claim contested cells first.  Because cells are only
 * written with `max(old, new)`, the effect relative to the symmetric baseline
 * is strictly additive: this layer can only increase costs.
 *
 * When no path is available, the goal is nearby, or asymmetry_factor is zero,
 * the layer becomes a no-op and the symmetric baseline from the upstream
 * InflationLayer is left unchanged.
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
   * Always requests full-map reinflation because asymmetric costs are
   * path-relative in world frame; when a rolling-window costmap shifts,
   * shifted cells carry stale asymmetric values.
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap.
   *
   * Assumes master_grid already holds the symmetric baseline written by the
   * upstream InflationLayer.  When a valid path is available and
   * asymmetry_factor is non-zero, runs one BFS with asymmetric effective
   * distances and writes max(old, new) so costs can only increase.
   * Otherwise returns without modifying the costmap.
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
   * @brief Extract the global path into the costmap's coordinate frame.
   *
   * Transforms the stored global path into the costmap frame via TF2,
   * filters to points inside the local costmap window, and returns an
   * empty vector when the robot is within goal_distance_threshold_ of
   * the goal (disabling asymmetry near the goal).
   */
  std::vector<std::pair<double, double>> extractLocalPath(
    nav2_costmap_2d::Costmap2D & master_grid);

  /**
   * @brief Classify an obstacle cell as left (+1), right (-1), or neutral (0)
   * relative to the closest path segment.
   *
   * Uses a spatial hash grid to perform an O(1) broad-phase lookup of nearby
   * path segments, then projects the cell onto each overlapping path segment and 
   * uses the cross product of the segment direction and the cell offset to 
   * determine the side. Obstacles beyond neutral_threshold_ from the path 
   * are classified as neutral.
   */
  int8_t computeObstacleSide(
    int i, int j,
    const std::vector<std::pair<double, double>> & local_path_pts,
    const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
    double bucket_size,
    nav2_costmap_2d::Costmap2D & master_grid);

  /**
   * @brief Compute the asymmetric effective distance (in cells) for BFS priority.
   *
   * Scales the physical distance by (1 - asymmetry_factor * path_side) and caps
   * the result at the inscribed radius (in cells).  The cap means every cell
   * outside the inscribed radius lands in the INSCRIBED bin: the asymmetry then
   * manifests as which obstacle's wave reaches each contested cell first (i.e.
   * a Voronoi-boundary shift toward the disfavored side), not as a tilt in the
   * cost magnitude outside the inscribed plateau.
   */
  inline double getEffectiveDistance(double physical_dist, int8_t path_side) const
  {
    return std::min(
      inscribed_radius_ / resolution_,
      physical_dist * std::max(0.0, 1.0 - asymmetry_factor_ * path_side));
  }

  /**
   * @brief Compute an exponential-decay cost for a distance expressed in cells.
   *
   * Matches the formula used by nav2_costmap_2d::InflationLayer: `distance` is
   * in cells and is converted to meters (via resolution_) before comparison
   * with inscribed_radius_ (meters) and feeding the exponential.
   */
  inline unsigned char computeCost(double distance) const
  {
    if (distance == 0) {
      return LETHAL_OBSTACLE;
    }
    if(distance <= inscribed_radius_) {
      return INSCRIBED_INFLATED_OBSTACLE;
    }
    double factor = exp(-1.0 * cost_scaling_factor_ * (distance - inscribed_radius_) * resolution_);
    return static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }

  /**
   * @brief Lookup pre-computed Euclidean distance between two cells.
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
   * without mutating any state. Mirrors nav2_costmap_2d::InflationLayer.
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
