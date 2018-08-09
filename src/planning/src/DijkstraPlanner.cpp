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
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <limits>
#include <iostream>
#include <algorithm>
#include "planning/DijkstraPlanner.hpp"
#include "planning/Navfn.hpp"
#include "world_model/CostValues.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

using namespace std::chrono_literals;


DijkstraPlanner::DijkstraPlanner(const std::string & name)
: ComputePathToPoseTaskServer(name), global_frame_("/map"), allow_unknown_(true), default_tolerance_(1.0)
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::DijkstraPlanner");

  //TODO(orduno): Enable parameter server

  costmap_client_ = this->create_client<nav2_msgs::srv::GetCostmap>("CostmapService");
  waitForCostmapServer();

  // TODO(orduno): Service for getting the costmap sometimes fails, check how the parent task can handle this.
  //               might need to pull out the getCostmap() method out of the constructor and have an initialize() one.
  try {
    getCostmap(costmap_);
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "DijkstraPlanner::makePlan: failed to obtain costmap from server");
    throw;
  }

  printCostmap(costmap_);

  planner_ = std::make_shared<NavFn>(costmap_.info.width, costmap_.info.height);

  // Plan publisher for visualization purposes
  plan_publisher_ = this->create_publisher<nav2_msgs::msg::Path>("plan", 1);
}

DijkstraPlanner::~DijkstraPlanner()
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::~DijkstraPlanner");
}

TaskStatus
DijkstraPlanner::executeAsync(const ComputePathToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::executeAsync");

  ComputePathToPoseResult result;
  try {
    // TODO(caorduno): call getCostmap() to update
    //                 if costmap update throws error, might use the old one under some conditions.
    makePlan(command->start, command->goal, command->tolerance, result.poses);
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "DijkstraPlanner::executeAsync: plan calculation failed");
    // TODO(orduno): provide information about fail error to parent task, for example: couldn't get costmap update
    return TaskStatus::FAILED;
  }

  if (cancelRequested()){
    RCLCPP_INFO(get_logger(), "DijkstraPlanner::executeAsync: task has been canceled");
    setCanceled();
    return TaskStatus::CANCELED;
  }
  // TODO(orduno): should check for cancel within the makePlan() method?

  RCLCPP_INFO(get_logger(), "DijkstraPlanner::executeAsync: calculated path of size %u", result.poses.size());

  // We've successfully completed the task, so return the result
  RCLCPP_INFO(get_logger(), "DijkstraPlanner::executeAsync: task completed");

  result.header.stamp = this->now();
  result.header.frame_id = global_frame_;
  setResult(result);

  return TaskStatus::SUCCEEDED;
}

bool
DijkstraPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
    double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  // clear the plan, just in case
  plan.clear();

  // TODO(orduno): add checks for start and goal reference frame -- should be in gobal frame

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int mx, my;
  if(!worldToMap(wx, wy, mx, my)){
    RCLCPP_WARN(this->get_logger(), "DijkstraPlanner::makePlan: The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.info.width, costmap_.info.height);

  // planner_->setCostmap(&std::vector<unsigned char>(costmap_.data.begin(), costmap_.data.end())[0], true, allow_unknown_);
  planner_->setCostmap(&costmap_.data[0], true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;

  if(worldToMap(wx, wy, mx, my)){
    if(tolerance <= 0.0){
      std::cout << "tolerance: " << tolerance << std::endl;
      RCLCPP_WARN(this->get_logger(), "DijkstraPlanner::makePlan: The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }
    mx = 0;
    my = 0;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Provide details on why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  planner_->calcNavFnDijkstra(true);

  double resolution = costmap_.info.resolution;
  geometry_msgs::msg::PoseStamped p, best_pose;
  p = goal;

  bool found_legal = false;
  double best_sdist = std::numeric_limits<double>::max();

  p.pose.position.y = goal.pose.position.y - tolerance;

  while(p.pose.position.y <= goal.pose.position.y + tolerance){
    p.pose.position.x = goal.pose.position.x - tolerance;
    while(p.pose.position.x <= goal.pose.position.x + tolerance){
      double potential = getPointPotential(p.pose.position);
      double sdist = squared_distance(p, goal);
      if(potential < POT_HIGH && sdist < best_sdist){
        best_sdist = sdist;
        best_pose = p;
        found_legal = true;
      }
      p.pose.position.x += resolution;
    }
    p.pose.position.y += resolution;
  }

  if(found_legal){
    //extract the plan
    if(getPlanFromPotential(best_pose, plan)){
      //make sure the goal we push on has the same timestamp as the rest of the plan
      geometry_msgs::msg::PoseStamped goal_copy = best_pose;
      goal_copy.header.stamp = this->now();
      plan.push_back(goal_copy);
    }
    else{
      RCLCPP_ERROR(this->get_logger(), "DijkstraPlanner::makePlan: Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
    }
  }

  // TODO(orduno): Enable potential visualization

  // Publish the plan for visualization purposes
  publishPlan(plan);

  return !plan.empty();
}

bool
DijkstraPlanner::computePotential(const geometry_msgs::msg::Point& world_point)
{
  //make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_.info.width, costmap_.info.height);

  std::vector<unsigned char> costmapData = std::vector<unsigned char>(costmap_.data.begin(), costmap_.data.end());

  planner_->setCostmap(&costmapData[0], true, allow_unknown_);

  unsigned int mx, my;
  if(!worldToMap(world_point.x, world_point.y, mx, my))
    return false;

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
DijkstraPlanner::getPlanFromPotential(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  //clear the plan, just in case
  plan.clear();

  // Goal should be in global frame
  double wx = goal.pose.position.x;
  double wy = goal.pose.position.y;

  //the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if(!worldToMap(wx, wy, mx, my)){
    RCLCPP_WARN(this->get_logger(), "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  planner_->calcPath(costmap_.info.width * 4);

  //extract the plan
  float *x = planner_->getPathX();
  float *y = planner_->getPathY();
  int len = planner_->getPathLen();
  rclcpp::Time plan_time = this->now();

  for(int i = len - 1; i >= 0; --i){
    //convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  //publish the plan for visualization purposes
  publishPlan(plan);
  return !plan.empty();
}

double
DijkstraPlanner::getPointPotential(const geometry_msgs::msg::Point& world_point)
{
  unsigned int mx, my;
  if(!worldToMap(world_point.x, world_point.y, mx, my))
    return std::numeric_limits<double>::max();

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool
DijkstraPlanner::validPointPotential(const geometry_msgs::msg::Point& world_point)
{
  return validPointPotential(world_point, default_tolerance_);
}

bool
DijkstraPlanner::validPointPotential(const geometry_msgs::msg::Point& world_point, double tolerance)
{
  double resolution = costmap_.info.resolution;

  geometry_msgs::msg::Point p;
  p = world_point;

  p.y = world_point.y - tolerance;

  while(p.y <= world_point.y + tolerance){
    p.x = world_point.x - tolerance;
    while(p.x <= world_point.x + tolerance){
      double potential = getPointPotential(p);
      if(potential < POT_HIGH){
        return true;
      }
      p.x += resolution;
    }
    p.y += resolution;
  }

  return false;
}

bool
DijkstraPlanner::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
  if (wx < costmap_.info.origin.position.x || wy < costmap_.info.origin.position.y)
  {
    return false;
  }

  mx = (int)((wx - costmap_.info.origin.position.x) / costmap_.info.resolution);
  my = (int)((wy - costmap_.info.origin.position.y) / costmap_.info.resolution);

  if (mx < costmap_.info.width && my < costmap_.info.height)
    return true;

  return false;
}

void
DijkstraPlanner::mapToWorld(double mx, double my, double& wx, double& wy)
{
  wx = costmap_.info.origin.position.x + mx * costmap_.info.resolution;
  wy = costmap_.info.origin.position.y + my * costmap_.info.resolution;
}

void
DijkstraPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno), check usage of this function, might instead be a request to world_model / map server
  unsigned int index = my * costmap_.info.width + mx;
  costmap_.data[index] = costmap::FREE_SPACE;
}

void
DijkstraPlanner::getCostmap(nav2_msgs::msg::Costmap& costmap, const std::chrono::milliseconds waitTime)
{
  auto costmapServiceResult = costmap_client_->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), costmapServiceResult, waitTime) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "DijkstraPlanner::getCostmap: costmap service call failed");
    throw "costmap service call failed";
  }
  costmap = costmapServiceResult.get()->map;
}

bool
DijkstraPlanner::waitForCostmapServer(const std::chrono::seconds waitTime)
{
  while (!costmap_client_->wait_for_service(waitTime))
  {
    if(!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "DijkstraPlanner::waitForCostmapServer: costmap client interrupted while waiting for the service to appear.");
      throw "interrupted while waiting for costmap server to appear";
    }
    RCLCPP_INFO(this->get_logger(), "DijkstraPlanner:waitForCostmapServer: waiting for the costmap service to appear...")
  }
  // TODO(orduno): fix return
  return true;
}

void
DijkstraPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path)
{
  //create a message for the plan
  nav2_msgs::msg::Path gui_path;
  gui_path.poses.resize(path.size());

  if(!path.empty())
  {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
  }

  // Extract the plan in world coordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
    gui_path.poses[i] = path[i];
  }

  plan_publisher_->publish(gui_path);
}

void
DijkstraPlanner::printCostmap(const nav2_msgs::msg::Costmap& costmap)
{
  std::cout << "Costmap" << std::endl;
  std::cout << "  size:       " << costmap.info.width << "," << costmap.info.height << std::endl;
  std::cout << "  origin:     " << costmap.info.origin.position.x << "," << costmap.info.origin.position.y << std::endl;
  std::cout << "  resolution: " << costmap.info.resolution << std::endl;
  std::cout << "  data:       " << "(" << costmap.data.size() << " cells)" << std::endl << "    ";

  int index = 0;
  for (int h = 0; h < costmap.info.height; ++h)
  {
    for (int w = 0; w < costmap.info.height; ++w)
    {
      std::cout << static_cast<unsigned int>(costmap.data[index]) << " ";
      index++;
    }
    std::cout << std::endl << "    ";
  }
  std::cout << std::endl;
}
