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

#include "dwb_critics/path_dist.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/path_ops.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace dwb_critics
{
bool PathDistCritic::prepare(
  const geometry_msgs::msg::Pose &, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose &,
  const nav_msgs::msg::Path & global_plan)
{
  reset();
  bool started_path = false;

  nav_msgs::msg::Path adjusted_global_plan =
    nav_2d_utils::adjustPlanResolution(global_plan, costmap_->getResolution());

  if (adjusted_global_plan.poses.size() != global_plan.poses.size()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(
        "PathDistCritic"), "Adjusted global plan resolution, added %zu points",
      adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  unsigned int i;
  // put global path points into local map until we reach the border of the local map
  for (i = 0; i < adjusted_global_plan.poses.size(); ++i) {
    double g_x = adjusted_global_plan.poses[i].pose.position.x;
    double g_y = adjusted_global_plan.poses[i].pose.position.y;
    unsigned int map_x, map_y;
    if (costmap_->worldToMap(
        g_x, g_y, map_x,
        map_y) && costmap_->getCost(map_x, map_y) != nav2_costmap_2d::NO_INFORMATION)
    {
      int index = costmap_->getIndex(map_x, map_y);
      cell_values_[index] = 0.0;
      queue_->enqueueCell(map_x, map_y);
      started_path = true;
    } else if (started_path) {
      break;
    }
  }
  if (!started_path) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PathDistCritic"),
      "None of the %d first of %zu (%zu) points of the global plan were in "
      "the local costmap and free",
      i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  propagateManhattanDistances();

  return true;
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDistCritic, dwb_core::TrajectoryCritic)
