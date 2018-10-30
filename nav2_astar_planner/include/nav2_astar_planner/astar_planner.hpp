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
// limitations under the License.

#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <exception>

#include "nav2_util/costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_dijkstra_planner/navfn.hpp"

namespace nav2_astar_planner
{

class AStarPlanner : public nav2_tasks::ComputePathToPoseTaskServer
{
public:
  AStarPlanner();
  ~AStarPlanner();

  nav2_tasks::TaskStatus execute(
    const nav2_tasks::ComputePathToPoseCommand::SharedPtr command) override;

private:

  makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav2_msgs::msg::Path & plan);

  computePotential(const geometry_msgs::msg::Point & world_point);

  getPlanFromPotential(
    const geometry_msgs::msg::Pose & goal,
    nav2_msgs::msg::Path & plan);

  getPointPotential(const geometry_msgs::msg::Point & world_point);

  validPointPotential(const geometry_msgs::msg::Point & world_point);

  validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

  worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  mapToWorld(double mx, double my, double & wx, double & wy);

  clearRobotCell(unsigned int mx, unsigned int my);

  getCostmap(
    nav2_msgs::msg::Costmap & costmap, const std::string /*layer*/,
    const std::chrono::milliseconds /*waitTime*/);

  printCostmap(const nav2_msgs::msg::Costmap & costmap);

  publishEndpoints(const nav2_tasks::ComputePathToPoseCommand::SharedPtr & endpoints);

  publishPlan(const nav2_msgs::msg::Path & path);

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<NavFn> planner_;

  // Service client for getting the costmap
  nav2_tasks::CostmapServiceClient costmap_client_;

  // The global frame of the costmap
  std::string global_frame_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // Amount the planner can relax the space constraint
  double default_tolerance_;
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
