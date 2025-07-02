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

#include "dwb_critics/path_hug_critic.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathHugCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

// ---------------------------------------------------------------------------
//  Helper: Trajectory2D → Path
// ---------------------------------------------------------------------------
static nav_msgs::msg::Path
trajectory2DToPath(const dwb_msgs::msg::Trajectory2D & traj,
                   const std::string & frame_id)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp    = rclcpp::Clock().now();
  path.poses.reserve(traj.poses.size());

  for (const auto & p2 : traj.poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p2.x;
    ps.pose.position.y = p2.y;
    ps.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p2.theta);
    ps.pose.orientation = tf2::toMsg(q);

    path.poses.emplace_back(std::move(ps));
  }
  return path;
}

// ---------------------------------------------------------------------------
//  onInit()
// ---------------------------------------------------------------------------
void PathHugCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"PathHugCritic: failed to lock node"};
  }

  declare_parameter_if_not_declared(
    node, full_name_ + ".penalty",      rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, full_name_ + ".strafe_x",     rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, full_name_ + ".strafe_theta", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, full_name_ + ".theta_scale",  rclcpp::ParameterValue(10.0));

  node->get_parameter(full_name_ + ".penalty",      penalty_);
  node->get_parameter(full_name_ + ".strafe_x",     strafe_x_);
  node->get_parameter(full_name_ + ".strafe_theta", strafe_theta_);
  node->get_parameter(full_name_ + ".theta_scale",  theta_scale_);
}

// ---------------------------------------------------------------------------
//  prepare()
// ---------------------------------------------------------------------------
bool PathHugCritic::prepare(const geometry_msgs::msg::Pose2D & /*pose*/,
                            const nav_2d_msgs::msg::Twist2D & /*vel*/,
                            const geometry_msgs::msg::Pose2D & /*goal*/,
                            const nav_2d_msgs::msg::Path2D & global_plan)
{
  if (global_plan.poses.empty()) {
    return false;
  }
  global_plan_path_ =
    nav_2d_utils::path2DToPath(global_plan);  // same frame + stamp retained
  return true;
}

// ---------------------------------------------------------------------------
//  scoreTrajectory()
// ---------------------------------------------------------------------------
double PathHugCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) {
    return penalty_;  // invalid candidate
  }

  // Convert the candidate once, then compute mean lateral deviation
  const nav_msgs::msg::Path traj_path =
    trajectory2DToPath(traj, global_plan_path_.header.frame_id);

  double accum = 0.0;
  size_t idx   = 0;  // latch for iterative local search

  for (const auto & pose_stamped : traj_path.poses) {
    accum += nav2_util::distanceFromPath(
      pose_stamped, global_plan_path_, &idx);
  }
  return accum / traj_path.poses.size();  // lower is better
}

}  // namespace dwb_critics
