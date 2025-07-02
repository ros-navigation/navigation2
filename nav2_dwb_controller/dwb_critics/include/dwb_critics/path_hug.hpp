/******************************************************************************
 *  Copyright (c) 2025, Berkan Tali
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#ifndef DWB_CRITICS__PATH_HUG_CRITIC_HPP_
#define DWB_CRITICS__PATH_HUG_CRITIC_HPP_

#include <string>

#include "dwb_core/trajectory_critic.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_utils/pose.hpp"
#include "nav_msgs/msg/path.hpp"

namespace dwb_critics
{

/**
 * @class PathHugCritic
 * @brief  Penalises trajectories that drift away from (or cut corners on)
 *         the global path, encouraging the robot to “hug” the path centre-line.
 *
 * Configuration parameters (declare in `onInit()`):
 *   • penalty        — score returned when path is violated outright
 *   • strafe_x       — threshold on |Δx| before penalising (metres)
 *   • strafe_theta   — threshold on |yaw| before penalising (radians)
 *   • theta_scale    — linear weight applied to |yaw| when within thresholds
 *
 * prepare() aborts scoring if the global_plan is empty, avoiding NaN spam.
 */
class PathHugCritic : public dwb_core::TrajectoryCritic
{
public:
  PathHugCritic() = default;

  /*-------------- TrajectoryCritic overrides --------------------------------*/
  void onInit() override;

  bool prepare(const geometry_msgs::msg::Pose2D & pose,
               const nav_2d_msgs::msg::Twist2D & vel,
               const geometry_msgs::msg::Pose2D & goal,
               const nav_2d_msgs::msg::Path2D & global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  double penalty_      {1.0};
  double strafe_x_     {0.1};
  double strafe_theta_ {0.2};
  double theta_scale_  {10.0};

  nav_msgs::msg::Path global_plan_path_;
};

}  // namespace dwb_critics

#endif  // DWB_CRITICS__PATH_HUG_CRITIC_HPP_
