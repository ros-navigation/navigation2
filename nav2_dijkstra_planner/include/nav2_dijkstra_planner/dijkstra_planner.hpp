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

#ifndef NAV2_DIJKSTRA_PLANNER__DIJKSTRA_PLANNER_HPP_
#define NAV2_DIJKSTRA_PLANNER__DIJKSTRA_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_tasks/costmap_service_client.hpp"
#include "nav2_dijkstra_planner/navfn.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_dijkstra_planner
{

class DijkstraPlanner : public nav2_tasks::ComputePathToPoseTaskServer
{
public:
  DijkstraPlanner();
  ~DijkstraPlanner();

  nav2_tasks::TaskStatus execute(
    const nav2_tasks::ComputePathToPoseCommand::SharedPtr command) override;

private:
  // Compute a plan given start and goal poses, provided in global world frame.
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav2_msgs::msg::Path & plan);

  // Compute the navigation function given a seed point in the world to start from
  bool computePotential(const geometry_msgs::msg::Point & world_point);

  // Compute a plan to a goal from a potential - must call computePotential first
  bool getPlanFromPotential(
    const geometry_msgs::msg::Pose & goal,
    nav2_msgs::msg::Path & plan);

  // Compute the potential, or navigation cost, at a given point in the world
  // - must call computePotential first
  double getPointPotential(const geometry_msgs::msg::Point & world_point);

  // Check for a valid potential value at a given point in the world
  // - must call computePotential first
  bool validPointPotential(const geometry_msgs::msg::Point & world_point);
  bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

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

  // Request costmap from world model
  void getCostmap(
    nav2_msgs::msg::Costmap & costmap, const std::string layer = "master",
    const std::chrono::milliseconds waitTime = std::chrono::milliseconds(100));

  // Print costmap to terminal
  void printCostmap(const nav2_msgs::msg::Costmap & costmap);

  // Publish a path for visualization purposes
  void publishPlan(const nav2_msgs::msg::Path & path);
  void publishEndpoints(const nav2_tasks::ComputePathToPoseCommand::SharedPtr & endpoints);

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<NavFn> planner_;

  // Service client for getting the costmap
  nav2_tasks::CostmapServiceClient costmap_client_;

  // Publishers for the path and endpoints
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plan_marker_publisher_;

  // The costmap to use
  nav2_msgs::msg::Costmap costmap_;

  // The global frame of the costmap
  std::string global_frame_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;

  // Amount the planner can relax the space constraint
  double default_tolerance_;
};

}  // namespace nav2_dijkstra_planner

#endif  // NAV2_DIJKSTRA_PLANNER__DIJKSTRA_PLANNER_HPP_
