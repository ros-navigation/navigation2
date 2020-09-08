// Copyright (c) 2020 Samsung Research Russia
// Copyright (c) 2018 Intel Corporation
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
// limitations under the License. Reserved.

#ifndef COSTMAP_FILTERS__FILTERS_TESTER_HPP_
#define COSTMAP_FILTERS__FILTERS_TESTER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_system_tests
{

// Zone tolerance parameter which is qeual to one cell size
static constexpr double ZONE_TOLERANCE = 0.1;

class FiltersTester : public nav2_util::LifecycleNode
{
public:
  FiltersTester();
  ~FiltersTester();

  // Test that planner can produce a path from start to end point
  bool testPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end);

  // FiltersTester spinner
  void spinTester();

  // Returns true if FiltersTester was activated
  inline bool isActive() const
  {
    return is_active_;
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<nav2_navfn_planner::NavfnPlanner> planner_;

  // The tester must provide the robot pose through a transform
  std::unique_ptr<geometry_msgs::msg::TransformStamped> base_transform_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  void startRobotTransform();
  void updateRobotPosition(const geometry_msgs::msg::Point & position);
  void publishRobotTransform();

  // Returns true if (x, y) belongs to (x1, y1, x2, y2) bar, otherwise returns false
  inline bool isInBar(
    const double & x, const double & y,
    const double & x1, const double & y1, const double & x2, const double & y2)
  {
    if (x > x1 + ZONE_TOLERANCE && y > y1 + ZONE_TOLERANCE &&
      x < x2 - ZONE_TOLERANCE && y < y2 - ZONE_TOLERANCE)
    {
      return true;
    }
    return false;
  }

  // Returns true if position is inside keepout zone
  bool isInKeepout(const geometry_msgs::msg::Point & position);

  // Check that planner could make a plan from start to end point
  bool checkPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & end,
    nav_msgs::msg::Path & path) const;

  // Print produced plan. Debug
  void printPath(const nav_msgs::msg::Path & path) const;

  bool is_active_;
};

}  // namespace nav2_system_tests

#endif  // COSTMAP_FILTERS__FILTERS_TESTER_HPP_
