// Copyright (c) 2026 Origin
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

#include "nav2_pose_classifiers/constraint_classifier.hpp"

namespace nav2_pose_classifiers
{

// Scale factor to convert metres → Clipper integer coordinates.
// 1e6 = micrometre precision. Clipper1 uses int64 so max ≈ ±4.6e18,
// giving a coordinate range of ±4.6e12 metres — more than enough.
static constexpr double kClipperScale = 1e6;

// ---------------------------------------------------------------------------
// configure
// ---------------------------------------------------------------------------

void ConstraintClassifier::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = name;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("ConstraintClassifier: parent node expired during configure");
  }
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".class_type",
    rclcpp::ParameterValue(nav2_msgs::msg::PathClasses::CONSTRAINT_SPACE));  // CONSTRAINT_SPACE = 1

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".inflation_resolution",
    rclcpp::ParameterValue(0.20));  // metres per iteration (It Should Be ALways >=0 and <= max_wall_thickness in the environment)

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_constraint_clearance",
    rclcpp::ParameterValue(1.0));  // metres from footprint edge

  class_type_ = static_cast<uint16_t>(
    node->get_parameter(name_ + ".class_type").as_int());
  inflation_resolution_ = node->get_parameter(name_ + ".inflation_resolution").as_double();
  if (inflation_resolution_ <= 0.0) {
    throw std::runtime_error(
            "ConstraintClassifier: 'inflation_resolution' parameter must be positive.");
  }
  max_constraint_clearance_ = node->get_parameter(name_ + ".max_constraint_clearance").as_double();

  collision_checker_.setCostmap(costmap_ros_->getCostmap());

  raw_fp_ = costmap_ros_->getRobotFootprint();
  opposites_ = buildOppositePairs(raw_fp_);

  RCLCPP_INFO(
    logger_,
    "ConstraintClassifier [%s] configured: inflation_resolution=%.3f, "
    "max_constraint_clearance=%.3f",
    name_.c_str(), inflation_resolution_, max_constraint_clearance_);
}

void ConstraintClassifier::cleanup() {}
void ConstraintClassifier::activate() {}
void ConstraintClassifier::deactivate() {}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

nav2_costmap_2d::Footprint ConstraintClassifier::inflateFootprint(
  const nav2_costmap_2d::Footprint & fp, double delta) const
{
  // Convert footprint to Clipper integer path (micrometre scale)
  ClipperLib::Path clipper_path;
  clipper_path.reserve(fp.size());
  for (const auto & pt : fp) {
    clipper_path.emplace_back(
      static_cast<ClipperLib::cInt>(pt.x * kClipperScale),
      static_cast<ClipperLib::cInt>(pt.y * kClipperScale));
  }

  // Run mitered offset
  ClipperLib::ClipperOffset co(2.0);  // miterLimit = 2.0
  co.AddPath(clipper_path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);

  ClipperLib::Paths solution;
  co.Execute(solution, delta * kClipperScale);

  if (solution.empty()) {
    return fp;  // fallback: return original
  }

  // Convert back to nav2 Footprint
  nav2_costmap_2d::Footprint result;
  result.reserve(solution[0].size());
  for (const auto & pt : solution[0]) {
    geometry_msgs::msg::Point p;
    p.x = static_cast<double>(pt.X) / kClipperScale;
    p.y = static_cast<double>(pt.Y) / kClipperScale;
    p.z = 0.0;
    result.push_back(p);
  }
  return result;
}

nav2_costmap_2d::Footprint ConstraintClassifier::orientFootprint(
  const nav2_costmap_2d::Footprint & fp,
  double x, double y, double cos_th, double sin_th) const
{
  // Same rotation as footprintCostAtPose (footprint_collision_checker.cpp:132-139)
  nav2_costmap_2d::Footprint result;
  result.reserve(fp.size());
  for (const auto & pt : fp) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (pt.x * cos_th - pt.y * sin_th);
    new_pt.y = y + (pt.x * sin_th + pt.y * cos_th);
    new_pt.z = 0.0;
    result.push_back(new_pt);
  }
  return result;
}

std::vector<size_t> ConstraintClassifier::buildOppositePairs(
  const nav2_costmap_2d::Footprint & fp) const
{
  const size_t n = fp.size(); // number of vertices in the footprint
  std::vector<size_t> opposite(n, n);  // n = "no opposite" (default value)

  // Centroid
  double cx = 0.0, cy = 0.0;
  for (const auto & pt : fp) {
    cx += pt.x;
    cy += pt.y;
  }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);

  for (size_t i = 0; i < n; ++i) {
    const size_t i_next = (i + 1) % n;

    // Midpoint of edge i
    const double mx = (fp[i].x + fp[i_next].x) * 0.5;
    const double my = (fp[i].y + fp[i_next].y) * 0.5;

    // Ray direction: midpoint → centroid → beyond
    const double dx = cx - mx;
    const double dy = cy - my;

    // Find first edge intersection for t > 1 (past centroid)
    double best_t = 1e18;
    size_t best_edge = n;

    for (size_t j = 0; j < n; ++j) {
      if (j == i) {
        continue;
      }
      const size_t j_next = (j + 1) % n;

      // Ray: P = (mx, my) + t * (dx, dy)
      // Segment: Q = fp[j] + s * (fp[j_next] - fp[j]),  s ∈ [0, 1]
      const double ex = fp[j_next].x - fp[j].x;
      const double ey = fp[j_next].y - fp[j].y;

      const double denom = dx * ey - dy * ex;
      if (std::abs(denom) < 1e-12) {
        continue;  // parallel
      }

      const double t = ((fp[j].x - mx) * ey - (fp[j].y - my) * ex) / denom;
      const double s = ((fp[j].x - mx) * dy - (fp[j].y - my) * dx) / denom;

      // t > 1 = past centroid (other side), s ∈ [0, 1] = on the segment
      if (t > 1.0 && s >= 0.0 && s <= 1.0 && t < best_t) {
        best_t = t;
        best_edge = j;
      }
    }

    opposite[i] = best_edge;
  }

  return opposite;
}

// ---------------------------------------------------------------------------
// matches  —  core classification
// ---------------------------------------------------------------------------

bool ConstraintClassifier::matches(const geometry_msgs::msg::PoseStamped & pose)
{
  // ── 1. Pose ───────────────────────────────────────────────────────────────
  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double theta = tf2::getYaw(pose.pose.orientation);
  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);

  // ── 2. Footprint + opposite pairs (recompute only on change) ──────────
  const nav2_costmap_2d::Footprint current_fp = costmap_ros_->getRobotFootprint();
  if (current_fp.size() < 3) {
    return false;
  }
  // Recompute opposite pairs only if the footprint has changed
  if (current_fp != raw_fp_) {
    raw_fp_ = current_fp;
    opposites_ = buildOppositePairs(raw_fp_);
  }
  const size_t n = raw_fp_.size();

  // ── 3. Costmap + mutex ────────────────────────────────────────────────────
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // ── 4. Track which edges have hit LETHAL ──────────────────────────────────
  std::vector<bool> lethal_edges(n, false);

  // ── 5. Iterative inflation loop ───────────────────────────────────────────
  const int max_steps = static_cast<int>(
    std::ceil(max_constraint_clearance_ / inflation_resolution_));

  for (int step = 1; step <= max_steps; ++step) {
    const double delta = step * inflation_resolution_;

    // 5a. Inflate footprint in robot frame using Clipper mitered offset
    const auto inflated_fp = inflateFootprint(raw_fp_, delta);

    // 5b. Orient to world frame
    const auto world_fp = orientFootprint(inflated_fp, x, y, cos_th, sin_th);

    // 5c. worldToMap all vertices
    const size_t m = world_fp.size();
    std::vector<std::pair<unsigned int, unsigned int>> cells(m);
    bool all_in_map = true;
    for (size_t i = 0; i < m; ++i) {
      if (!costmap->worldToMap(
          world_fp[i].x, world_fp[i].y,
          cells[i].first, cells[i].second))
      {
        all_in_map = false;
        break;
      }
    }
    if (!all_in_map) {
      continue;  // some vertices outside map, skip this step
    }

    // 5d. Per-edge lineCost — check for LETHAL
    // Clipper with jtMiter + etClosedPolygon preserves vertex count for
    // convex polygons, so edge i of inflated == edge i of original.
    if (m != n) {
      continue;  // unexpected vertex count change, skip step
    }

    for (size_t i = 0; i < n; ++i) {
      const size_t j = (i + 1) % n;
      const double cost = collision_checker_.lineCost(
        cells[i].first, cells[j].first,
        cells[i].second, cells[j].second);

      if (cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        lethal_edges[i] = true;

        // Check if the opposite edge (via centroid ray) already hit LETHAL
        if (opposites_[i] < n && lethal_edges[opposites_[i]]) {
          return true;  // Both sides walled → CONSTRAINT
        }
      }
    }
  }

  return false;
}

// ---------------------------------------------------------------------------
// classType
// ---------------------------------------------------------------------------

uint16_t ConstraintClassifier::classType()
{
  return class_type_;
}

}  // namespace nav2_pose_classifiers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_pose_classifiers::ConstraintClassifier,
  nav2_pose_classifiers::ClassifierBase)
