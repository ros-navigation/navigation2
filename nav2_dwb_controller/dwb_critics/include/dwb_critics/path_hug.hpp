// Copyright (c) 2025, Berkan Tali
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

#include <string>
#include "dwb_core/trajectory_critic.hpp"
#include "nav_msgs/msg/path.hpp"

namespace dwb_critics
{

/**
 * @class PathHugCritic
 * @brief Critic that penalizes trajectories based on their distance from the global path
 *
 * This critic encourages the robot to stay close to the planned path while allowing
 * deviations for obstacle avoidance. Uses a bidirectional search window to handle
 * both forward and backward trajectory motion.
 */
class PathHugCritic : public dwb_core::TrajectoryCritic
{
public:
  /**
   * @brief Result structure for segment search operations
   */
  struct SegmentSearchResult
  {
    size_t closest_index;  ///< Index of the closest path segment
    double distance;       ///< Distance to the closest segment
  };

  void onInit() override;

  bool prepare(
    const geometry_msgs::msg::Pose & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose & goal,
    const nav_msgs::msg::Path & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

  double getScale() const override;

protected:
  /**
   * @brief Find closest path segment using bidirectional search window
   *
   * Searches both forward and backward from the hint index to handle trajectories
   * that may move in any direction. The search window prevents matching to
   * geometrically close but topologically distant path segments.
   *
   * @param path Global path to search
   * @param pose Current pose to find closest segment for
   * @param hint_index Starting index for search (typically previous closest segment)
   * @param search_window Size of bidirectional search window in path indices
   * @return SegmentSearchResult containing closest segment index and distance
   */
  SegmentSearchResult findClosestSegmentWithLookback(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & pose,
    size_t hint_index,
    double search_window) const;

  nav_msgs::msg::Path global_path_;
  double scale_;
  double search_window_;
};

}  // namespace dwb_critics

#endif  // DWB_CRITICS__PATH_HUG_HPP_
