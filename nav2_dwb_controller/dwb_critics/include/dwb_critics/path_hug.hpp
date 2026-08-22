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

#ifndef DWB_CRITICS__PATH_HUG_HPP_
#define DWB_CRITICS__PATH_HUG_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace dwb_critics
{
/**
 * @class PathHugCritic
 * @brief Penalizes trajectories that stray outside a corridor around the global path.
 *
 * Each scored pose is mapped through a continuous distance-based cost curve with
 * a zero-cost grace zone, an optional soft-repulsion band, and a steep outer region
 * lifted by critical_cost_. An optional heading term penalizes misalignment with
 * a forward look-ahead point on the path.
 */
class PathHugCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose & goal,
    const nav_msgs::msg::Path & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

  double getScale() const override;

  static constexpr double kHeadingEps = 1e-6;

  static double distanceSqToSegment(
    const geometry_msgs::msg::Point & p,
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b)
  {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;

    if (len_sq < kHeadingEps * kHeadingEps) {
      const double px = p.x - a.x;
      const double py = p.y - a.y;
      return px * px + py * py;
    }

    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    const double ex = p.x - (a.x + t * dx);
    const double ey = p.y - (a.y + t * dy);
    return ex * ex + ey * ey;
  }

protected:
  struct SegmentSearchResult
  {
    size_t closest_index;
    double distance;
  };

  SegmentSearchResult findClosestSegment(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & pose,
    size_t hint_index,
    double search_window) const;

  nav_msgs::msg::Path decimatePlan(const nav_msgs::msg::Path & path) const;

  double threshold_to_consider_;
  double search_window_;
  double min_path_point_spacing_;
  double max_allowed_distance_;
  double grace_distance_;
  double heading_weight_;
  double tracking_lookahead_dist_;
  double min_traj_point_spacing_;
  double critical_cost_;
  bool use_soft_repulsion_;
  bool zero_scale_;
  int power_;

  size_t hint_;
  std::vector<double> cumulative_distances_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PATH_HUG_HPP_
