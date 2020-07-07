// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_navfn_planner
{

class NavfnPlanner : public nav2_core::GlobalPlanner
{
public:
  NavfnPlanner();
  ~NavfnPlanner();

  // plugin configure
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;


  // plugin create path
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // Compute a plan given start and goal poses, provided in global world frame.
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav_msgs::msg::Path & plan);

  // Compute the navigation function given a seed point in the world to start from
  bool computePotential(const geometry_msgs::msg::Point & world_point);

  // Compute a plan to a goal from a potential - must call computePotential first
  bool getPlanFromPotential(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  // Remove artifacts at the end of the path - originated from planning on a discretized world
  void smoothApproachToGoal(
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan);

  // Compute the potential, or navigation cost, at a given point in the world
  // - must call computePotential first
  double getPointPotential(const geometry_msgs::msg::Point & world_point);

  // Check for a valid potential value at a given point in the world
  // - must call computePotential first
  // - currently unused
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

  // Compute the squared distance between two points
  inline double squared_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2)
  {
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    return dx * dx + dy * dy;
  }

  // Transform a point from world to map frame
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  // Transform a point from map to world frame
  void mapToWorld(double mx, double my, double & wx, double & wy);

  // Set the corresponding cell cost to be free space
  void clearRobotCell(unsigned int mx, unsigned int my);

  // Determine if a new planner object should be made
  bool isPlannerOutOfDate();

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<NavFn> planner_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_;

  // Whether to use the astar planner or default dijkstras
  bool use_astar_;
};

}  // namespace nav2_navfn_planner

#endif  // NAV2_NAVFN_PLANNER__NAVFN_PLANNER_HPP_
