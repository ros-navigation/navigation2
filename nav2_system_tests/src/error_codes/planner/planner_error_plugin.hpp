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

#ifndef ERROR_CODES__PLANNER__PLANNER_ERROR_PLUGIN_HPP_
#define ERROR_CODES__PLANNER__PLANNER_ERROR_PLUGIN_HPP_

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

namespace nav2_system_tests
{

class UnknownErrorPlanner : public nav2_core::GlobalPlanner
{
public:
  UnknownErrorPlanner() = default;
  ~UnknownErrorPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::PlannerException("Unknown Error");
  }
};

class StartOccupiedErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::StartOccupied("Start Occupied");
  }
};

class GoalOccupiedErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::GoalOccupied("Goal occupied");
  }
};

class StartOutsideMapErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::StartOutsideMapBounds("Start OutsideMapBounds");
  }
};

class GoalOutsideMapErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::GoalOutsideMapBounds("Goal outside map bounds");
  }
};

class NoValidPathErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    return nav_msgs::msg::Path();
  }
};


class TimedOutErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::PlannerTimedOut("Planner Timed Out");
  }
};

class TFErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::PlannerTFError("TF Error");
  }
};

class NoViapointsGivenErrorPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()>) override
  {
    throw nav2_core::NoViapointsGiven("No Via points given");
  }
};

class CancelledPlanner : public UnknownErrorPlanner
{
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    std::function<bool()> cancel_checker) override
  {
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
      std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5))
    {
      if (cancel_checker()) {
        throw nav2_core::PlannerCancelled("Planner Cancelled");
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    throw nav2_core::PlannerException("Cancel is not called in time.");
  }
};

}  // namespace nav2_system_tests


#endif  // ERROR_CODES__PLANNER__PLANNER_ERROR_PLUGIN_HPP_
