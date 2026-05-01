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
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <cmath>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::AsymmetricInflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

AsymmetricInflationLayer::AsymmetricInflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  asymmetry_factor_(0),
  inflate_around_unknown_(false),
  goal_distance_threshold_(0),
  neutral_threshold_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max()),
  need_reinflation_(false),
  cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0)
{
  access_ = new mutex_t();
}

AsymmetricInflationLayer::~AsymmetricInflationLayer()
{
  delete access_;
}

void
AsymmetricInflationLayer::onInitialize()
{
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
    inflation_radius_ = node->declare_or_get_parameter(
      name_ + "." + "inflation_radius", 2.0);
    cost_scaling_factor_ = node->declare_or_get_parameter(
      name_ + "." + "cost_scaling_factor", 4.0);
    asymmetry_factor_ = node->declare_or_get_parameter(
      name_ + "." + "asymmetry_factor", 0.75);
    inflate_around_unknown_ = node->declare_or_get_parameter(
      name_ + "." + "inflate_around_unknown", false);
    plan_topic_ = node->declare_or_get_parameter<std::string>(
      name_ + "." + "plan_topic", "plan");
    goal_distance_threshold_ = node->declare_or_get_parameter(
      name_ + "." + "goal_distance_threshold", 1.5);
    neutral_threshold_ = node->declare_or_get_parameter(
      name_ + "." + "neutral_threshold", 2.0);

    // Apply the same bound checks as dynamic reconfigure, so bad YAML values fail
    // loudly at startup instead of silently producing bad costmaps.
    if (std::abs(asymmetry_factor_) >= 1.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: asymmetry_factor magnitude must be < 1.0");
    }
    if (inflation_radius_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: inflation_radius must be >= 0");
    }
    if (cost_scaling_factor_ <= 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: cost_scaling_factor must be > 0");
    }
    if (neutral_threshold_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: neutral_threshold must be >= 0");
    }
    if (goal_distance_threshold_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: goal_distance_threshold must be >= 0");
    }

    plan_topic_ = joinWithParentNamespace(plan_topic_);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      plan_topic_,
      std::bind(
        &AsymmetricInflationLayer::globalPathCallback,
        this, std::placeholders::_1),
      rclcpp::QoS(1).durability_volatile());
  }

  setCurrent(true);
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

void
AsymmetricInflationLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AsymmetricInflationLayer::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &AsymmetricInflationLayer::updateParametersCallback,
      this, std::placeholders::_1));
}

void
AsymmetricInflationLayer::deactivate()
{
  auto node = node_.lock();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
}

void
AsymmetricInflationLayer::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_global_path_ = msg;
  // Path change invalidates all asymmetric costs in the costmap.
  // Force a full-map reinflation on the next update cycle.
  need_reinflation_ = true;
  setCurrent(false);
}

void
AsymmetricInflationLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();

  unsigned int size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
  seen_ = std::vector<bool>(size, false);
  obstacle_side_grid_ = std::vector<int8_t>(size, 0);
}

void
AsymmetricInflationLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // Track robot pose for the goal-proximity fallback check
  current_robot_x_ = robot_x;
  current_robot_y_ = robot_y;

  if (need_reinflation_) {
    // Reset last_* to "no expansion" values so the next cycle won't
    // merge with these full-map bounds (avoids double full-map update after reset)
    last_min_x_ = last_min_y_ = std::numeric_limits<double>::max();
    last_max_x_ = last_max_y_ = std::numeric_limits<double>::lowest();

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void
AsymmetricInflationLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;

  if (inflation_radius_ < inscribed_radius_) {
    RCLCPP_ERROR(
      logger_,
      "The configured inflation radius (%.3f) is smaller than "
      "the computed inscribed radius (%.3f) of your footprint, "
      "it is highly recommended to set inflation radius to be at "
      "least as big as the inscribed radius to avoid collisions",
      inflation_radius_, inscribed_radius_);
  }

  RCLCPP_DEBUG(
    logger_, "AsymmetricInflationLayer::onFootprintChanged(): num footprint points: %zu,"
    " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
    layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

std::vector<std::pair<double, double>>
AsymmetricInflationLayer::extractLocalPath(
  nav2_costmap_2d::Costmap2D & master_grid)
{
  std::vector<std::pair<double, double>> local_path_pts;
  nav_msgs::msg::Path current_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!latest_global_path_ || latest_global_path_->poses.empty()) {
      return local_path_pts;
    }
    current_path = *latest_global_path_;
  }

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  std::string path_frame = current_path.header.frame_id;
  geometry_msgs::msg::TransformStamped transform;
  bool need_transform = (global_frame != path_frame && !path_frame.empty());

  // Transform the global path into the costmap frame (e.g., map -> odom)
  if (need_transform) {
    try {
      transform = tf_->lookupTransform(global_frame, path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "AsymmetricInflationLayer: TF lookup failed (%s -> %s): %s. "
        "Falling back to symmetric inflation.",
        path_frame.c_str(), global_frame.c_str(), ex.what());
      return local_path_pts;
    }
  }

  // Disable asymmetry near the goal to prevent target oscillations
  geometry_msgs::msg::PoseStamped goal_pose = current_path.poses.back();
  if (need_transform) {
    tf2::doTransform(goal_pose, goal_pose, transform);
  }

  double dist_to_goal = std::hypot(
    goal_pose.pose.position.x - current_robot_x_,
    goal_pose.pose.position.y - current_robot_y_);

  if (dist_to_goal <= goal_distance_threshold_) {
    // Empty vector causes algorithm to use standard symmetry
    return local_path_pts;
  }

  for (const auto & pose : current_path.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose = pose;
    if (need_transform) {
      tf2::doTransform(pose, transformed_pose, transform);
    }

    double px = transformed_pose.pose.position.x;
    double py = transformed_pose.pose.position.y;
    unsigned int mx, my;

    // Only process points inside our current local costmap window
    if (master_grid.worldToMap(px, py, mx, my)) {
      local_path_pts.push_back({px, py});
    }
  }
  return local_path_pts;
}

int8_t
AsymmetricInflationLayer::computeObstacleSide(
  double cx, double cy,
  const std::vector<size_t> & candidates,
  const std::vector<std::pair<double, double>> & local_path_pts)
{
  const double neutral_threshold_sq = neutral_threshold_ * neutral_threshold_;

  double min_dist_sq = std::numeric_limits<double>::max();
  double best_cross = 0.0;

  // Evaluate candidate segments provided by the spatial hash.
  for (size_t p : candidates) {
    // Define segment endpoints A (start) and B (end).
    double ax = local_path_pts[p].first;
    double ay = local_path_pts[p].second;
    double bx = local_path_pts[p + 1].first;
    double by = local_path_pts[p + 1].second;

    // --- Boundary Check ---
    // Quickly skip segments if the point is further than the neutral threshold from the segment's
    // bounding box. This prevents unnecessary calculations for distant segments.
    double min_x = std::min(ax, bx) - neutral_threshold_;
    double max_x = std::max(ax, bx) + neutral_threshold_;
    if (cx < min_x || cx > max_x) {continue;}

    double min_y = std::min(ay, by) - neutral_threshold_;
    double max_y = std::max(ay, by) + neutral_threshold_;
    if (cy < min_y || cy > max_y) {continue;}

    // --- Distance and Orientation Calculation ---
    // Vector AB represents the path segment, Vector AC represents the path to the cell.
    double abx = bx - ax;
    double aby = by - ay;
    double len_sq = abx * abx + aby * aby;

    double acx = cx - ax;
    double acy = cy - ay;

    double dist_sq;
    double cross;

    // Handle the case of a zero-length segment to avoid division by zero.
    if (len_sq < 1e-10) {
      dist_sq = acx * acx + acy * acy;
      cross = 0.0;
    } else {
      /*
       * Calculate 't', the scalar projection of C onto the line AB.
       * We clamp 't' to the [0, 1] range so that points projecting outside the
       * segment endpoints are snapped to the nearest vertex.
       */
      double t = std::clamp((acx * abx + acy * aby) / len_sq, 0.0, 1.0);

      // Compute the vector from the projected point on the segment to the cell C.
      double proj_dx = acx - t * abx;
      double proj_dy = acy - t * aby;
      dist_sq = proj_dx * proj_dx + proj_dy * proj_dy;

      /*
       * Determine the side of the segment using the 2D cross product (AB x AC).
       * Positive values indicate the point is on the Left side of the path.
       * Negative values indicate the point is on the Right side.
       */
      cross = abx * acy - aby * acx;
    }

    // Update the best match if this segment is the closest one found so far.
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_cross = cross;
    }
  }

  // Check if the cell is outside the influence radius.
  if (min_dist_sq > neutral_threshold_sq) {
    return 0;
  }

  // Return the orientation based on the cross product of the closest segment.
  if (best_cross > 0.0) {return 1;}    // Left
  if (best_cross < 0.0) {return -1;}   // Right

  return 0;  // Neutral/On the line
}

void
AsymmetricInflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || (cell_inflation_radius_ == 0)) {
    return;
  }

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : inflation_cells_) {
    RCLCPP_FATAL_EXPRESSION(
      logger_,
      !dist.empty(), "The inflation list must be empty at the beginning of inflation");
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    RCLCPP_WARN(
      logger_, "AsymmetricInflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
    obstacle_side_grid_ = std::vector<int8_t>(size_x * size_y, 0);
  }

  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;

  min_i = std::max(0, min_i - static_cast<int>(cell_inflation_radius_));
  min_j = std::max(0, min_j - static_cast<int>(cell_inflation_radius_));
  max_i = std::min(static_cast<int>(size_x), max_i + static_cast<int>(cell_inflation_radius_));
  max_j = std::min(static_cast<int>(size_y), max_j + static_cast<int>(cell_inflation_radius_));

  std::vector<std::pair<double, double>> local_path_pts = extractLocalPath(master_grid);
  bool use_asymmetry = (local_path_pts.size() >= 2) && (asymmetry_factor_ != 0.0);

  // This method runs in three phases:
  //
  //   Phase 1 – Spatial-hash construction  O(P)  P = path segments in window
  //   Phase 2 – BFS seed                   O(W)  W = update-window cells
  //   Phase 3 – Dial's Algorithm BFS       O(B)  B = cells within inflation radius
  //
  // neutral_threshold_ is the maximum perpendicular distance (metres) from the
  // path centreline within which an obstacle is classified as left (+1) or
  // right (-1). Obstacles farther away are "neutral" and receive symmetric
  // inflation.  path_bucket_size equals neutral_threshold_ so each hash bucket
  // spans exactly one influence radius: the padded AABB insertion ensures every
  // segment that can affect a cell in bucket B is stored in B, making per-cell
  // lookups in Phase 2 O(1) instead of O(P).

  // ── Phase 1: Spatial-hash construction ──────────────────────────────────
  // Pre-partition path segments into spatial buckets so that per-cell nearest-
  // segment queries in Phase 2 are O(1) instead of O(path_segments).
  const double path_bucket_size = std::max(neutral_threshold_, 1.0);
  std::unordered_map<uint64_t, std::vector<size_t>> spatial_hash;

  if (!use_asymmetry) {
    setCurrent(true);
    return;
  }

  for (size_t p = 0; p < local_path_pts.size() - 1; ++p) {
    double ax = local_path_pts[p].first;
    double ay = local_path_pts[p].second;
    double bx = local_path_pts[p + 1].first;
    double by = local_path_pts[p + 1].second;

    // Pad the segment's bounding box by the threshold
    double min_x = std::min(ax, bx) - neutral_threshold_;
    double max_x = std::max(ax, bx) + neutral_threshold_;
    double min_y = std::min(ay, by) - neutral_threshold_;
    double max_y = std::max(ay, by) + neutral_threshold_;

    // Find which buckets this padded segment touches
    int64_t min_bx = static_cast<int64_t>(std::floor(min_x / path_bucket_size));
    int64_t max_bx = static_cast<int64_t>(std::floor(max_x / path_bucket_size));
    int64_t min_by = static_cast<int64_t>(std::floor(min_y / path_bucket_size));
    int64_t max_by = static_cast<int64_t>(std::floor(max_y / path_bucket_size));

    for (int64_t b_x = min_bx; b_x <= max_bx; ++b_x) {
      for (int64_t b_y = min_by; b_y <= max_by; ++b_y) {
        // Bitwise magic to safely map 2D signed coordinates into a 1D unsigned 64-bit key
        uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
          (static_cast<uint32_t>(b_y));

        spatial_hash[key].push_back(p);
      }
    }
  }

  // ── Phase 2: BFS seed – classify boundary obstacle cells ─────────────────
  // Only the outer perimeter of each lethal obstacle mass is seeded.
  // Interior cells (no traversable 4-connected neighbour) are pre-marked seen
  // so the BFS wave skips them; they can never be the nearest obstacle to any
  // free cell. Neutral boundary cells (exact distance > neutral_threshold_) are also skipped:
  // they cannot raise costs above the symmetric baseline already written by the InflationLayer.

  // Reset seen_ so the BFS can track which cells we visited without needing to clear master_array.
  std::fill(begin(seen_), end(seen_), false);

  // Helper lambda to check if a neighbor is "traversable" (i.e., open space)
  auto is_traversable = [&](int nx, int ny) {
      unsigned char n_cost = master_array[master_grid.getIndex(nx, ny)];
      if (inflate_around_unknown_) {
        return  n_cost != LETHAL_OBSTACLE && n_cost != NO_INFORMATION;
      } else {
        return  n_cost != LETHAL_OBSTACLE;
      }
    };

  // Seed all lethal cells
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(master_grid.getIndex(i, j));
      unsigned char cost = master_array[index];

      // Early exit 1: Skip cells that aren't lethal/unknown obstacles
      if (cost != LETHAL_OBSTACLE && !(inflate_around_unknown_ && cost == NO_INFORMATION)) {
        continue;
      }

      // Check if the cell touches the absolute edges of the costmap
      bool is_on_map_edge = (i == 0 || i == static_cast<int>(size_x) - 1 ||
        j == 0 || j == static_cast<int>(size_y) - 1);

      // An obstacle cell is a boundary if it's on the map edge OR touches free space.
      bool is_obstacle_boundary = is_on_map_edge ||
        is_traversable(i - 1, j) ||
        is_traversable(i + 1, j) ||
        is_traversable(i, j - 1) ||
        is_traversable(i, j + 1);

      // Early exit 2: Skip interior obstacle cells
      if (!is_obstacle_boundary) {
        // Mark interior cells as 'seen' so the BFS wave doesn't
        // waste time propagating backwards into the solid obstacle mass.
        seen_[index] = true;
        continue;
      }

      // O(1) hash lookup to find candidate segments near this cell.
      double cx, cy;
      master_grid.mapToWorld(i, j, cx, cy);
      int64_t b_x = static_cast<int64_t>(std::floor(cx / path_bucket_size));
      int64_t b_y = static_cast<int64_t>(std::floor(cy / path_bucket_size));

      // Reinterpret each signed int64 bucket coordinate as uint32 (preserving the bit
      // pattern for negative values via two's complement) then pack both into a single
      // uint64. Collision-free for any coordinate fitting in int32 — true for all
      // real-world costmap sizes.
      uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
        static_cast<uint32_t>(b_y);

      auto it = spatial_hash.find(key);

      // Early exit 3: Skip cells with no nearby path segments
      if (it == spatial_hash.end()) {continue;}

      // Forward the candidate segments to determine which side of the path this cell is on
      int8_t side = computeObstacleSide(cx, cy, it->second, local_path_pts);
      obstacle_side_grid_[index] = side;

      // Skip neutral cells since they won't receive asymmetric inflation.
      if (side != 0) {
        inflation_cells_[0].emplace_back(i, j, i, j);
      }
    }
  }

  // ── Phase 3: Dial's Algorithm BFS ────────────────────────────────────────
  // inflation_cells_ is a bucket-priority queue indexed by
  //   floor(effective_distance * kEffDistPrecision).
  // Expanding the lowest non-empty bin first guarantees monotonic wave order.
  // Cells are written with max(old, new) so this layer can only raise costs
  // above the symmetric baseline left by the upstream InflationLayer.
  for (size_t current_bin = 0; current_bin < inflation_cells_.size(); ++current_bin) {
    while (!inflation_cells_[current_bin].empty()) {
      const CellData cell = inflation_cells_[current_bin].back();
      inflation_cells_[current_bin].pop_back();

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;
      unsigned int index = master_grid.getIndex(mx, my);

      // A cell can be enqueued by multiple obstacle sources.
      // The bucket queue guarantees the first dequeue is the minimum effective
      // distance, so any later arrival for this cell is a duplicate.
      if (seen_[index]) {
        continue;
      }
      seen_[index] = true;

      // src_x/src_y is the original obstacle seed, carried intact through the wave.
      // Every cell expanding from the same seed shares its left/right classification.
      int src_index = master_grid.getIndex(sx, sy);
      int8_t path_side = obstacle_side_grid_[src_index];

      // physical_dist is the true Euclidean distance from the source obstacle.
      // eff_dist applies the asymmetric scale factor, determining the priority-queue bin:
      // the disfavoured side gets a smaller eff_dist so its wave expands into contested
      // space before the favoured side, shifting the Voronoi boundary.
      double physical_dist = distanceLookup(mx, my, sx, sy);
      double eff_dist = getEffectiveDistance(physical_dist, path_side);
      size_t bin = static_cast<size_t>(eff_dist * kEffDistPrecision);

      unsigned char eff_cost = (bin < cached_costs_.size()) ? cached_costs_[bin] : 0;
      unsigned char old_cost = master_array[index];

      // Write costs only within the originally requested window (base_min/max)
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        if (eff_cost > old_cost) {
          master_array[index] = eff_cost;
        }
      }

      // Propagate to 4-connected neighbours. enqueue() returns the lowest non-empty
      // bin after insertion; asymmetric scaling can place a neighbour in a bin below
      // current_bin, so current_bin must be allowed to decrease here.
      if (mx > 0) {
        current_bin = enqueue(index - 1, mx - 1, my, sx, sy, path_side, current_bin);
      }
      if (my > 0) {
        current_bin = enqueue(index - size_x, mx, my - 1, sx, sy, path_side, current_bin);
      }
      if (mx < size_x - 1) {
        current_bin = enqueue(index + 1, mx + 1, my, sx, sy, path_side, current_bin);
      }
      if (my < size_y - 1) {
        current_bin = enqueue(index + size_x, mx, my + 1, sx, sy, path_side, current_bin);
      }
    }
  }

  setCurrent(true);
}

size_t
AsymmetricInflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y, int8_t path_side, size_t current_bin)
{
  if (seen_[index]) {
    return current_bin;
  }

  double physical_dist = distanceLookup(mx, my, src_x, src_y);

  // Stop propagating the wave at the configured inflation radius
  if (physical_dist > cell_inflation_radius_) {
    return current_bin;
  }

  // Map the effective (asymmetric) distance to a priority queue bin.
  // This allows disfavored-side obstacles to claim contested cells first,
  // shifting the Voronoi boundary toward the disfavored side.
  double eff_dist = getEffectiveDistance(physical_dist, path_side);
  size_t target_bin = static_cast<size_t>(eff_dist * kEffDistPrecision);

  if (target_bin < inflation_cells_.size()) {
    inflation_cells_[target_bin].emplace_back(mx, my, src_x, src_y);
    return std::min(target_bin, current_bin);
  } else {
    RCLCPP_WARN(
      logger_,
      "Effective distance %.3f exceeds maximum cache range. "
      "This should not happen and indicates a bug.",
      eff_dist);
    return current_bin;
  }
}

void
AsymmetricInflationLayer::computeCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // Pre-compute the distance cache
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_distances_.resize(cache_length_ * cache_length_);
    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }
    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  // Determine the maximum possible effective distance to size the priority queue.
  // Both sides are considered because asymmetry_factor_ can be positive or negative.
  double max_eff_dist = std::max(
    getEffectiveDistance(cell_inflation_radius_, -1),
    getEffectiveDistance(cell_inflation_radius_, 1));
  size_t max_bin = static_cast<size_t>(std::ceil(max_eff_dist * kEffDistPrecision)) + 2;

  cached_costs_.resize(max_bin);
  for (size_t i = 0; i < max_bin; ++i) {
    double eff_dist = static_cast<double>(i) / kEffDistPrecision;
    cached_costs_[i] = computeCost(eff_dist);
  }

  inflation_cells_.clear();
  inflation_cells_.resize(max_bin);
}

rcl_interfaces::msg::SetParametersResult
AsymmetricInflationLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type != ParameterType::PARAMETER_DOUBLE) {
      continue;
    }

    if (param_name == name_ + ".asymmetry_factor") {
      double val = parameter.as_double();
      if (std::abs(val) >= 1.0) {
        RCLCPP_WARN(
          logger_,
          "asymmetry_factor magnitude %.2f >= 1.0 is out of range. "
          "Rejecting parameter update.", val);
        result.successful = false;
        result.reason = "asymmetry_factor magnitude must be < 1.0";
        return result;
      } else if (std::abs(val) > 0.9) {
        RCLCPP_WARN(
          logger_,
          "asymmetry_factor magnitude %.2f is high. "
          "Values over 0.9 can lead to artefacts.", val);
      }
    } else if (param_name == name_ + ".inflation_radius") {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "inflation_radius must be >= 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "inflation_radius must be >= 0";
        return result;
      }
    } else if (param_name == name_ + ".cost_scaling_factor") {
      if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
          logger_, "cost_scaling_factor must be > 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "cost_scaling_factor must be > 0";
        return result;
      }
    } else if (param_name == name_ + ".neutral_threshold") {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "neutral_threshold must be >= 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "neutral_threshold must be >= 0";
        return result;
      }
    } else if (param_name == name_ + ".goal_distance_threshold") {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "goal_distance_threshold must be >= 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "goal_distance_threshold must be >= 0";
        return result;
      }
    }
  }

  return result;
}

void
AsymmetricInflationLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  bool need_cache_recompute = false;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".inflation_radius" &&
        inflation_radius_ != parameter.as_double())
      {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
        setCurrent(false);
      } else if (param_name == name_ + ".cost_scaling_factor" &&  // NOLINT
        cost_scaling_factor_ != parameter.as_double())
      {
        cost_scaling_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
        setCurrent(false);
      } else if (param_name == name_ + ".asymmetry_factor" &&  // NOLINT
        asymmetry_factor_ != parameter.as_double())
      {
        asymmetry_factor_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
        setCurrent(false);
      } else if (param_name == name_ + ".goal_distance_threshold" &&  // NOLINT
        goal_distance_threshold_ != parameter.as_double())
      {
        goal_distance_threshold_ = parameter.as_double();
        setCurrent(false);
      } else if (param_name == name_ + ".neutral_threshold" &&  // NOLINT
        neutral_threshold_ != parameter.as_double())
      {
        neutral_threshold_ = parameter.as_double();
        setCurrent(false);
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + ".enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        setCurrent(false);
      } else if (param_name == name_ + ".inflate_around_unknown" &&  // NOLINT
        inflate_around_unknown_ != parameter.as_bool())
      {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
        setCurrent(false);
      }
    }
  }

  if (need_cache_recompute) {
    matchSize();
  }
}

}  // namespace nav2_costmap_2d
