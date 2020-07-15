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

#ifndef LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_H_
#define LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_H_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "lazy_theta_star2.h"

namespace lazyThetaStarPlanner
{

class LazyThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_;

  std::string globalFrame_, name_;
  int how_many_corners_;
  double interpolation_dist_;
  std::unique_ptr<lazyThetaStar::LazyThetaStar> planner_;


  unsigned int src_[2], dst_[2];

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapRos) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  nav_msgs::msg::Path linearInterpolation(
    std::vector<coords<world_pts>> & raw_path,
    double dist_bw_points);

};

}   // namespace planner

#endif //LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_H_
