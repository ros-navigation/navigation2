// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAV2_ERROR_CODE_TEST__GLOBAL_PLANNERS__GOAL_OUTSIDE_MAP_PLANNER_HPP_
#define NAV2_ERROR_CODE_TEST__GLOBAL_PLANNERS__GOAL_OUTSIDE_MAP_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/planner_exceptions.hpp"

namespace nav2_error_code_test
{

class GoalOutsideMapBounds : public nav2_core::GlobalPlanner
{
public:
  GoalOutsideMapBounds() = default;
  ~GoalOutsideMapBounds() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override {}

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    throw nav2_core::GoalOutsideMapBounds("Goal Outside Map");
  }
};

}  // namespace nav2_error_code_test

#endif  // NAV2_ERROR_CODE_TEST__GLOBAL_PLANNERS__GOAL_OUTSIDE_MAP_PLANNER_HPP_
