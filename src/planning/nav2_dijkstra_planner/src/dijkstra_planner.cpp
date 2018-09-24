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


// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <limits>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <exception>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_dijkstra_planner/dijkstra_planner.hpp"
#include "nav2_dijkstra_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_libs_msgs/msg/costmap.hpp"
#include "nav2_world_model_msgs/srv/get_costmap.hpp"

using namespace std::chrono_literals;
using nav2_tasks::TaskStatus;

namespace nav2_dijkstra_planner
{

DijkstraPlanner::DijkstraPlanner()
: nav2_tasks::ComputePathToPoseTaskServer("ComputePathToPoseNode"),
  global_frame_("/map"), allow_unknown_(true), default_tolerance_(1.0)
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::DijkstraPlanner");

  // TODO(orduno): Enable parameter server

  costmap_client_ = this->create_client<nav2_world_model_msgs::srv::GetCostmap>("CostmapService");
  waitForCostmapServer();

  // TODO(orduno): Service for getting the costmap sometimes fails,
  //               check how the parent task can handle this. Might need to pull out
  //               the getCostmap() method out of the constructor and have an initialize() one.
  try {
    getCostmap(costmap_);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
      "DijkstraPlanner::makePlan: failed to obtain costmap from server");
    throw std::runtime_error("makePlan: failed to obtain costmap from server");
  }

  printCostmap(costmap_);

  planner_ = std::make_shared<NavFn>(costmap_.metadata.size_x, costmap_.metadata.size_y);

  // Plan publisher for visualization purposes
  printf("DijkstraPlanner::DijkstraPlanner: publish a plan (Path)\n");
  plan_publisher_ = this->create_publisher<nav2_planning_msgs::msg::Path>("plan", 1);
}

DijkstraPlanner::~DijkstraPlanner()
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::~DijkstraPlanner");
}

TaskStatus
DijkstraPlanner::execute(const nav2_tasks::ComputePathToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::execute");

  // MJ: This is not initialized:
  nav2_tasks::ComputePathToPoseResult result;
  try {
    // TODO(orduno): Get an updated costmap
    // getCostmap(costmap_);
    makePlan(command->start, command->goal, command->tolerance, result);
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "DijkstraPlanner::execute: plan calculation failed");
    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    return TaskStatus::FAILED;
  }

  if (cancelRequested()) {
    RCLCPP_INFO(get_logger(), "DijkstraPlanner::execute: task has been canceled");
    setCanceled();
    return TaskStatus::CANCELED;
  }
  // TODO(orduno): should check for cancel within the makePlan() method?

  RCLCPP_INFO(get_logger(),
    "DijkstraPlanner::execute: calculated path of size %u", result.poses.size());

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::execute: task completed");

  // Pass the output to the Task Client
  setResult(result);

  // Publish the plan for visualization purposes
  printf("DijkstraPlanner::execute: publish a plan (Path)\n");
  plan_publisher_->publish(result);

  // TODO(orduno): Enable potential visualization

  return TaskStatus::SUCCEEDED;
}

bool
DijkstraPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav2_planning_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // TODO(orduno): add checks for start and goal reference frame -- should be in gobal frame

  double wx = start.position.x;
  double wy = start.position.y;

  printf("wx, wy: %f,%f\n", wx, wy);
  printf("start x, y: %f,%f\n", start.position.x, start.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      this->get_logger(),
      "DijkstraPlanner::makePlan: The robot's start position is off the global costmap."
      " Planning will always fail, are you sure the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);

  planner_->setCostmap(&costmap_.data[0], true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (worldToMap(wx, wy, mx, my)) {
    if (tolerance <= 0.0) {
      std::cout << "tolerance: " << tolerance << std::endl;
      RCLCPP_WARN(
        this->get_logger(),
        "DijkstraPlanner::makePlan: The goal sent to the planner is off the global costmap."
        " Planning will always fail to this goal.");
      return false;
    }
    mx = 0;
    my = 0;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  planner_->calcNavFnDijkstra(true);

  double resolution = costmap_.metadata.resolution;
  geometry_msgs::msg::Pose p, best_pose;
  p = goal;

  bool found_legal = false;
  double best_sdist = std::numeric_limits<double>::max();

  p.position.y = goal.position.y - tolerance;

  while (p.position.y <= goal.position.y + tolerance) {
    p.position.x = goal.position.x - tolerance;
    while (p.position.x <= goal.position.x + tolerance) {
      double potential = getPointPotential(p.position);
      double sdist = squared_distance(p, goal);
      if (potential < POT_HIGH && sdist < best_sdist) {
        best_sdist = sdist;
        best_pose = p;
        found_legal = true;
      }
      p.position.x += resolution;
    }
    p.position.y += resolution;
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      geometry_msgs::msg::Pose goal_copy = best_pose;
      plan.poses.push_back(goal_copy);
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "DijkstraPlanner::makePlan: Failed to get a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

bool
DijkstraPlanner::computePotential(const geometry_msgs::msg::Point & world_point)
{
  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.metadata.size_x, costmap_.metadata.size_y);

  std::vector<unsigned char> costmapData = std::vector<unsigned char>(
    costmap_.data.begin(), costmap_.data.end());

  planner_->setCostmap(&costmapData[0], true, allow_unknown_);

  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return false;
  }

  int map_start[2];
  map_start[0] = 0;
  map_start[1] = 0;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_start);
  planner_->setGoal(map_goal);

  return planner_->calcNavFnDijkstra();
}

bool
DijkstraPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav2_planning_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      this->get_logger(),
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  planner_->calcPath(costmap_.metadata.size_x * 4);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  plan.header.stamp = this->now();
  plan.header.frame_id = global_frame_;

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::Pose pose;
    pose.position.x = world_x;
    pose.position.y = world_y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
DijkstraPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool
DijkstraPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
{
  return validPointPotential(world_point, default_tolerance_);
}

bool
DijkstraPlanner::validPointPotential(
  const geometry_msgs::msg::Point & world_point, double tolerance)
{
  double resolution = costmap_.metadata.resolution;

  geometry_msgs::msg::Point p;
  p = world_point;

  p.y = world_point.y - tolerance;

  while (p.y <= world_point.y + tolerance) {
    p.x = world_point.x - tolerance;
    while (p.x <= world_point.x + tolerance) {
      double potential = getPointPotential(p);
      if (potential < POT_HIGH) {
        return true;
      }
      p.x += resolution;
    }
    p.y += resolution;
  }

  return false;
}

bool
DijkstraPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_.metadata.origin.position.x || wy < costmap_.metadata.origin.position.y) {
    return false;
  }

  mx = static_cast<int>((wx - costmap_.metadata.origin.position.x) / costmap_.metadata.resolution);
  my = static_cast<int>((wy - costmap_.metadata.origin.position.y) / costmap_.metadata.resolution);

  if (mx < costmap_.metadata.size_x && my < costmap_.metadata.size_y) {
    return true;
  }

  return false;
}

void
DijkstraPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_.metadata.origin.position.x + mx * costmap_.metadata.resolution;
  wy = costmap_.metadata.origin.position.y + my * costmap_.metadata.resolution;
}

void
DijkstraPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  unsigned int index = my * costmap_.metadata.size_x + mx;
  costmap_.data[index] = nav2_util::Costmap::free_space;
}

void
DijkstraPlanner::getCostmap(
  nav2_libs_msgs::msg::Costmap & costmap, const std::string /*layer*/,
  const std::chrono::milliseconds waitTime)
{
  RCLCPP_INFO(this->get_logger(), "DijkstraPlanner::getCostmap: requesting a new costmap");

  // TODO(orduno): explicitly provide specifications for costmap using the costmap on the request,
  //               including master (aggreate) layer
  auto costmapServiceResult = costmap_client_->async_send_request(
    std::make_shared<nav2_world_model_msgs::srv::GetCostmap::Request>());

  if (rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), costmapServiceResult, waitTime) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "DijkstraPlanner::getCostmap: costmap service call failed");
    throw std::runtime_error("getCostmap: service call failed");
  }
  costmap = costmapServiceResult.get()->map;
}

void
DijkstraPlanner::waitForCostmapServer(const std::chrono::seconds waitTime)
{
  while (!costmap_client_->wait_for_service(waitTime)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "DijkstraPlanner::waitForCostmapServer:"
        " costmap client interrupted while waiting for the service to appear.");
      throw std::runtime_error(
              "waitForCostmapServer: interrupted while waiting for costmap server to appear");
    }
    RCLCPP_INFO(this->get_logger(),
      "DijkstraPlanner::waitForCostmapServer: waiting for the costmap service to appear...")
  }
}

void
DijkstraPlanner::printCostmap(const nav2_libs_msgs::msg::Costmap & costmap)
{
  std::cout << "Costmap" << std::endl;
  std::cout << "  size:       " <<
    costmap.metadata.size_x << "," << costmap.metadata.size_x << std::endl;
  std::cout << "  origin:     " <<
    costmap.metadata.origin.position.x << "," << costmap.metadata.origin.position.y << std::endl;
  std::cout << "  resolution: " << costmap.metadata.resolution << std::endl;
  std::cout << "  data:       " <<
    "(" << costmap.data.size() << " cells)" << std::endl << "    ";

  const char separator = ' ';
  const int valueWidth = 4;

  unsigned int index = 0;
  for (unsigned int h = 0; h < costmap.metadata.size_y; ++h) {
    for (unsigned int w = 0; w < costmap.metadata.size_x; ++w) {
      std::cout << std::left << std::setw(valueWidth) << std::setfill(separator) <<
        static_cast<unsigned int>(costmap.data[index]);
      index++;
    }
    std::cout << std::endl << "    ";
  }
  std::cout << std::endl;
}

}  // namespace nav2_dijkstra_planner
