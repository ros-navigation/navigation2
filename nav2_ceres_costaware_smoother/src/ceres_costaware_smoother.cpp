// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <memory>
#include <utility>

#include "nav2_ceres_costaware_smoother/ceres_costaware_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_ceres_costaware_smoother
{

template<typename T>
inline int sign(T a) {
  return a == 0 ? 0 : a > 0 ? 1 : -1;
}

void CeresCostawareSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> & costmap_sub,
  const std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> &)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  
  _smoother = std::make_unique<nav2_ceres_costaware_smoother::Smoother>();
  _optimizer_params.get(node.get(), name);
  _smoother_params.get(node.get(), name);
  _smoother_params.max_time = _optimizer_params.max_time;
  _smoother->initialize(_optimizer_params);

}

void CeresCostawareSmoother::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up smoother: %s of type"
    " nav2_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

void CeresCostawareSmoother::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating smoother: %s of type "
    "nav2_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

void CeresCostawareSmoother::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating smoother: %s of type "
    "nav2_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

bool CeresCostawareSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() <= 2)
    return true;

  auto costmap = costmap_sub_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // populate smoother input with (x, y, forward/reverse dir)
  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());
  Eigen::Vector2d start_dir;
  Eigen::Vector2d end_dir;
  for (int i = 0; i < (int)path.poses.size(); i++) {
    auto &pose = path.poses[i].pose;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double angle = q.getAngle()*sign(q.getZ());
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    if (i == (int)path.poses.size()-1) {
      path_world.emplace_back(pose.position.x, pose.position.y, path_world.back()[2]);
      end_dir = orientation;
    }
    else {
      auto &pos_next = path.poses[i+1].pose.position;
      Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);
      bool reversing = _smoother_params.reversing_enabled && orientation.dot(mvmt) < 0;
      path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
      if (i == 0)
        start_dir = orientation;
    }
  }

  _smoother_params.max_time = max_time.seconds();

  // Smooth plan
  if (!_smoother->smooth(path_world, start_dir, end_dir, &*costmap, _smoother_params)) {
    RCLCPP_WARN(
      logger_,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      plugin_name_.c_str());
    throw new nav2_core::PlannerException("Failed to smooth plan, Ceres could not find a usable solution.");
  }

  // populate final path
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  path.poses.clear();
  path.poses.reserve(path_world.size());
  for (auto &pw : path_world) {
    pose.pose.position.x = pw[0];
    pose.pose.position.y = pw[1];
    pose.pose.orientation.z = sin(pw[2]/2);
    pose.pose.orientation.w = cos(pw[2]/2);

    path.poses.push_back(pose);
  }

  return true;
}

}  // namespace nav2_ceres_costaware_smoother

// Register this smoother as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_ceres_costaware_smoother::CeresCostawareSmoother,
  nav2_core::Smoother)
