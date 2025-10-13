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

#include "dwb_critics/path_dist.hpp"

namespace dwb_critics
{
/**
 * @class PathHugCritic
 * @brief DWB critic that penalizes trajectories based on their distance from the global path.
 *        Encourages the robot to "hug" or stay close to the path.
 */
class PathHugCritic : public PathDistCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose & goal,
    const nav_msgs::msg::Path & global_plan) override;
  double getScale() const override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

protected:
  bool zero_scale_{false};
  nav_msgs::msg::Path global_path_;
  size_t start_index_;
  geometry_msgs::msg::Pose current_pose_;
  double average_score_;
  double search_window_;
};

}  // namespace dwb_critics

#endif  // DWB_CRITICS__PATH_HUG_HPP_
