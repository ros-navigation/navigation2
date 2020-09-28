// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <memory>
#include <string>
#include "lazy_theta_star_p_planner/lazy_theta_star_p_planner.hpp"
#include "lazy_theta_star_p_planner/lazy_theta_star_p.hpp"

namespace lazyThetaStarP_Planner
{
void LazyThetaStarP_Planner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos)
{
  planner_ = std::make_unique<lazyThetaStarP::LazyThetaStarP>();
  node_ = parent;
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmapRos->getCostmap();
  globalFrame_ = costmapRos->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".how_many_corners", rclcpp::ParameterValue(
      8));

  node_->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

  if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
    planner_->how_many_corners_ = 8;
    RCLCPP_WARN(node_->get_logger(),
      "Your value for - .how_many_coreners  was overriden, and is now set to 8");
  }


  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_dist", rclcpp::ParameterValue(
      0.1));

  node_->get_parameter(name_ + ".interpolation_dist", interpolation_dist_);

///   could be added in the future verions if required, this
///   includes the parameters for lethal_cost, parameters for distance function and the costmap traversal costs
//  nav2_util::declare_parameter_if_not_declared(
//    node_, name_ + ".lethal_cost", rclcpp::ParameterValue(
//      252));
//
//  node_->get_parameter(name_ + ".lethal_cost", lethal_cost_);
//
//    nav2_util::declare_parameter_if_not_declared(
//    node_, name_ + ".euc_tolerance", rclcpp::ParameterValue(1.0));
//
//    node_->get_parameter(name_ + ".euc_tolerance", planner_->euc_tolerance_);
//
//    nav2_util::declare_parameter_if_not_declared(
//    node_, name_ + ".costmap_tolerance", rclcpp::ParameterValue(1.0));
//
//    node_->get_parameter(name_ + ".costmap_tolerance", planner_->costmap_tolerance_);
}

void LazyThetaStarP_Planner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type LazyThetaStarP_Planner",
    name_.c_str());
}

void LazyThetaStarP_Planner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type LazyThetaStarP_Planner",
    name_.c_str());
}

void LazyThetaStarP_Planner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type LazyThetaStarP_Planner",
    name_.c_str());
}

nav_msgs::msg::Path LazyThetaStarP_Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  setStartAndGoal(start, goal);
  RCLCPP_INFO(
    node_->get_logger(), "Got the src and dst... (%i, %i) && (%i, %i)",
    planner_->src.x, planner_->src.y, planner_->dst.x, planner_->dst.y);
  getPlan(global_path);
  return global_path;
}

void LazyThetaStarP_Planner::getPlan(nav_msgs::msg::Path & global_path)
{
  planner_->node_ = node_;

  /// NOTE : a few lines of code is to be removed in the final version

  std::vector<coordsW> path;
  auto start = std::chrono::steady_clock::now();
  if (!(planner_->isSafe(planner_->src.x,
    planner_->src.y)) || !(planner_->isSafe(planner_->dst.x, planner_->dst.y)))
  {
//    RCLCPP_INFO(node_->get_logger(), "EITHER OF THE START OR GOAL POINTS ARE AN OBSTACLE! ");
//    coordsW world{};
//    planner_->costmap_->mapToWorld(planner_->src.x, planner_->src.y, world.x, world.y);
//    path.push_back({world.x, world.y});
//    planner_->costmap_->mapToWorld(planner_->dst.x, planner_->dst.y, world.x, world.y);
//    path.push_back({world.x, world.y});
//    global_path = linearInterpolation(path, interpolation_dist_);
	global_path.poses.clear();
  } else if (planner_->generatePath(path)) {
    RCLCPP_INFO(node_->get_logger(), "RECEIVED THE PATH");
    global_path = linearInterpolation(path, interpolation_dist_);
  } else {
    global_path.poses.clear();
  }

  auto stop = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "the time taken in microseconds is \t\t : " << dur.count() << '\n';
  path.clear();
  std::cout << global_path.poses.size() << '\n';
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = globalFrame_;
  RCLCPP_INFO(node_->get_logger(), "SENT OUT THE PATH");

  RCLCPP_INFO(node_->get_logger(), "the cost at src : %i",
    planner_->costmap_->getCost(planner_->src.x, planner_->src.y));
  RCLCPP_INFO(node_->get_logger(), "the cost at dst : %i",
    planner_->costmap_->getCost(planner_->dst.x, planner_->dst.y));
}

nav_msgs::msg::Path LazyThetaStarP_Planner::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;

  for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
    geometry_msgs::msg::PoseStamped p;
    coordsW pt1 = {raw_path[j].x, raw_path[j].y};
    p.pose.position.x = pt1.x;
    p.pose.position.y = pt1.y;
    pa.poses.push_back(p);

    coordsW pt2 = {raw_path[j + 1].x, raw_path[j + 1].y};
    geometry_msgs::msg::PoseStamped p1;
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }
  return pa;
}
void LazyThetaStarP_Planner::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int src_[2], dst_[2];
  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, src_[0], src_[1]);
  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, dst_[0], dst_[1]);

  planner_->src = {static_cast<int>(src_[0]), static_cast<int>(src_[1])};
  planner_->dst = {static_cast<int>(dst_[0]), static_cast<int>(dst_[1])};
}

}  // namespace lazyThetaStarP_Planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lazyThetaStarP_Planner::LazyThetaStarP_Planner, nav2_core::GlobalPlanner)
