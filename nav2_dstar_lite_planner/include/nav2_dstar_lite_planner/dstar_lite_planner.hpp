// Copyright (c) 2024 Nav2 Contributors
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

#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <cmath>
#include <functional>
#include <string>
#include <limits>
#include <chrono>
#include <algorithm>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_dstar_lite_planner/dstar_lite.hpp"
#include "nav2_dstar_lite_planner/parameter_handler.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_dstar_lite_planner
{

class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarLitePlanner")};
  std::string global_frame_, name_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;

  std::unique_ptr<DStarLite> planner_;

  Parameters * params_;
  std::unique_ptr<ParameterHandler> param_handler_;

  nav_msgs::msg::Path prev_path_;
  double prev_path_cost_{std::numeric_limits<double>::max()};

  void getPlan(
    nav_msgs::msg::Path & global_path,
    std::function<bool()> cancel_checker);

  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<WorldCoord> & raw_path,
    double resolution);

  double computePathCost(const nav_msgs::msg::Path & path) const;

  bool shouldSwitchPath(const nav_msgs::msg::Path & new_path) const;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
