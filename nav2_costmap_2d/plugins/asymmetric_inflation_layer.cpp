// Copyright (c) 2026
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
      name_ + "." + "inflation_radius", 0.55);
    cost_scaling_factor_ = node->declare_or_get_parameter(
      name_ + "." + "cost_scaling_factor", 10.0);
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
  int i, int j,
  const std::vector<std::pair<double, double>> & local_path_pts,
  const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
  double bucket_size,
  nav2_costmap_2d::Costmap2D & master_grid)
{
  double cx, cy;
  master_grid.mapToWorld(i, j, cx, cy);

  // 1. BROAD PHASE (Hash Lookup)
  int64_t b_x = static_cast<int64_t>(std::floor(cx / bucket_size));
  int64_t b_y = static_cast<int64_t>(std::floor(cy / bucket_size));
  uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
    (static_cast<uint32_t>(b_y));

  auto it = spatial_hash.find(key);
  if (it == spatial_hash.end()) {
    // Instant rejection: Obstacle is far away from all path segments
    return 0;
  }

  // 2. NARROW PHASE & EXACT PHASE
  const auto & segments_to_check = it->second;
  const double neutral_threshold_sq = neutral_threshold_ * neutral_threshold_;

  double min_dist_sq = std::numeric_limits<double>::max();
  double best_cross = 0.0;

  for (size_t p : segments_to_check) {
    double ax = local_path_pts[p].first;
    double ay = local_path_pts[p].second;
    double bx = local_path_pts[p + 1].first;
    double by = local_path_pts[p + 1].second;

    // Segment AABB check (Narrow Phase)
    double min_x = std::min(ax, bx) - neutral_threshold_;
    double max_x = std::max(ax, bx) + neutral_threshold_;
    if (cx < min_x || cx > max_x) {continue;}

    double min_y = std::min(ay, by) - neutral_threshold_;
    double max_y = std::max(ay, by) + neutral_threshold_;
    if (cy < min_y || cy > max_y) {continue;}

    // Projection Math (Exact Phase)
    double abx = bx - ax, aby = by - ay;
    double len_sq = abx * abx + aby * aby;

    double acx = cx - ax;
    double acy = cy - ay;

    double dist_sq;
    double cross;

    if (len_sq < 1e-10) {
      // Degenerate zero-length segment
      dist_sq = acx * acx + acy * acy;
      cross = 0.0;
    } else {
      // t is the scalar projection of AC onto AB, clamped to [0, 1]
      double t = std::clamp((acx * abx + acy * aby) / len_sq, 0.0, 1.0);

      // Calculate perpendicular distance squared
      double proj_dx = acx - t * abx;
      double proj_dy = acy - t * aby;
      dist_sq = proj_dx * proj_dx + proj_dy * proj_dy;
      cross = abx * acy - aby * acx;
    }

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_cross = cross;
    }
  }

  if (min_dist_sq > neutral_threshold_sq) {
    return 0;
  }

  if (best_cross > 0.0) {return 1;}
  if (best_cross < 0.0) {return -1;}
  return 0;
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

  // Sanity check: bins must be empty at the start of each cycle.
  // On failure, log a single summary (avoid per-cell log flood).
  size_t leftover = 0;
  for (const auto & bin : inflation_cells_) {
    leftover += bin.size();
  }
  RCLCPP_FATAL_EXPRESSION(
    logger_, leftover != 0,
    "AsymmetricInflationLayer: BFS bins not empty at start of updateCosts() "
    "(%zu leftover cells)", leftover);

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

  // Spatial Hash Grid Construction
  double bucket_size = std::max(neutral_threshold_, 1.0);
  std::unordered_map<uint64_t, std::vector<size_t>> spatial_hash;

  if (use_asymmetry) {
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
      int64_t min_bx = static_cast<int64_t>(std::floor(min_x / bucket_size));
      int64_t max_bx = static_cast<int64_t>(std::floor(max_x / bucket_size));
      int64_t min_by = static_cast<int64_t>(std::floor(min_y / bucket_size));
      int64_t max_by = static_cast<int64_t>(std::floor(max_y / bucket_size));

      for (int64_t b_x = min_bx; b_x <= max_bx; ++b_x) {
        for (int64_t b_y = min_by; b_y <= max_by; ++b_y) {
          // Bitwise magic to safely map 2D signed coordinates into a 1D unsigned 64-bit key
          uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
            (static_cast<uint32_t>(b_y));

          spatial_hash[key].push_back(p);
        }
      }
    }
  }

  // The standard Nav2 inflation_layer has already populated master_array
  // with the symmetric baseline. We only need to calculate asymmetric costs
  // and increase the cost where our effective distance is harsher.
  if (use_asymmetry) {
    std::fill(begin(seen_), end(seen_), false);

    auto & obs_bin_asym = inflation_cells_[0];

    // Helper lambda to check if a neighbor is "traversable" (i.e., open space)
    auto is_traversable = [&](int nx, int ny) {
        unsigned char n_cost = master_array[master_grid.getIndex(nx, ny)];
        if (inflate_around_unknown_) {
          return  n_cost != LETHAL_OBSTACLE && n_cost != NO_INFORMATION;
        } else {
          return  n_cost != LETHAL_OBSTACLE;
        }
      };

    // Seed all lethal obstacles (Boundary cells only)
    for (int j = min_j; j < max_j; j++) {
      for (int i = min_i; i < max_i; i++) {
        int index = static_cast<int>(master_grid.getIndex(i, j));
        unsigned char cost = master_array[index];

        if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
          bool is_boundary = false;

          // Map edge cells are treated as boundaries
          if (i == 0 || i == static_cast<int>(size_x) - 1 ||
            j == 0 || j == static_cast<int>(size_y) - 1)
          {
            is_boundary = true;
          } else {
            // Check 4-connected neighbors. If any neighbor is traversable space,
            // this cell is on the outer perimeter of the obstacle.
            if (is_traversable(i - 1, j) || is_traversable(i + 1, j) ||
              is_traversable(i, j - 1) || is_traversable(i, j + 1))
            {
              is_boundary = true;
            }
          }

          if (is_boundary) {
            obs_bin_asym.emplace_back(i, j, i, j);
            obstacle_side_grid_[index] = computeObstacleSide(i, j, local_path_pts, spatial_hash,
                bucket_size, master_grid);
          } else {
            // Mark interior cells as 'seen' so the BFS wave doesn't
            // waste time propagating backwards into the solid obstacle mass.
            seen_[index] = true;
          }
        }
      }
    }

    // Asymmetric BFS expansion: Implements Dial's Algorithm for fast expansion by scaling
    // effective distances into discrete bucket indices (`inflation_cells_`).
    for (size_t current_bin = 0; current_bin < inflation_cells_.size(); ++current_bin) {
      while (!inflation_cells_[current_bin].empty()) {
        const CellData cell = inflation_cells_[current_bin].back();
        inflation_cells_[current_bin].pop_back();

        unsigned int mx = cell.x_;
        unsigned int my = cell.y_;
        unsigned int sx = cell.src_x_;
        unsigned int sy = cell.src_y_;
        unsigned int index = master_grid.getIndex(mx, my);

        if (seen_[index]) {
          continue;
        }
        seen_[index] = true;

        int src_index = master_grid.getIndex(sx, sy);
        int8_t path_side = obstacle_side_grid_[src_index];

        double physical_dist = distanceLookup(mx, my, sx, sy);
        double eff_dist = getEffectiveDistance(physical_dist, path_side);
        size_t bin = static_cast<size_t>(eff_dist * kEffDistPrecision);

        unsigned char eff_cost = (bin < cached_costs_.size()) ? cached_costs_[bin] : 0;
        unsigned char old_cost = master_array[index];

        // Only increase — never decrease below the symmetric baseline
        if (static_cast<int>(mx) >= base_min_i &&
          static_cast<int>(my) >= base_min_j &&
          static_cast<int>(mx) < base_max_i &&
          static_cast<int>(my) < base_max_j)
        {
          if (eff_cost > old_cost) {
            master_array[index] = eff_cost;
          }
        }

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
      if (std::abs(val) >= 0.8) {
        RCLCPP_WARN(
          logger_,
          "asymmetry_factor magnitude %.2f >= 0.8 is out of range. "
          "Rejecting parameter update.", val);
        result.successful = false;
        result.reason = "asymmetry_factor magnitude must be < 0.8";
        return result;
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
