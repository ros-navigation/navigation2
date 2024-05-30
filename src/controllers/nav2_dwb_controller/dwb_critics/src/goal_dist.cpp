/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/goal_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
bool GoalDistCritic::prepare(
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  reset();

  unsigned int local_goal_x, local_goal_y;
  if (!getLastPoseOnCostmap(global_plan, local_goal_x, local_goal_y)) {
    return false;
  }

  // Enqueue just the last pose
  int index = costmap_->getIndex(local_goal_x, local_goal_y);
  cell_values_[index] = 0.0;
  queue_->enqueueCell(local_goal_x, local_goal_y);

  propogateManhattanDistances();

  return true;
}

bool GoalDistCritic::getLastPoseOnCostmap(
  const nav_2d_msgs::msg::Path2D & global_plan,
  unsigned int & x, unsigned int & y)
{
  nav_2d_msgs::msg::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(
    global_plan,
    costmap_->getResolution());
  bool started_path = false;

  // skip global path points until we reach the border of the local map
  for (unsigned int i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      // Still on the costmap. Continue.
      x = map_x;
      y = map_y;
      started_path = true;
    } else if (started_path) {
      // Off the costmap after being on the costmap. Return the last saved indices.
      return true;
    }
    // else, we have not yet found a point on the costmap, so we just continue
  }

  if (started_path) {
    return true;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "GoalDistCritic"), "None of the points of the global plan were in the local costmap.");
    return false;
  }
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::GoalDistCritic, dwb_core::TrajectoryCritic)
