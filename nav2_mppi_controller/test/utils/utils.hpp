// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <string_view>
#include <rclcpp/executors.hpp>

#include "tf2_ros/transform_broadcaster.h"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "models.hpp"
#include "factory.hpp"

using namespace std::chrono_literals;  // NOLINT

template<typename TNode>
void waitSome(const std::chrono::nanoseconds & duration, TNode & node)
{
  rclcpp::Time start_time = node->now();
  while (rclcpp::ok() && node->now() - start_time <= rclcpp::Duration(duration)) {
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(3ms);
  }
}

void sendTf(
  std::string_view source, std::string_view dest,
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, size_t n)
{
  while (--n != 0u) {
    auto t = geometry_msgs::msg::TransformStamped();
    t.header.frame_id = source;
    t.child_frame_id = dest;

    t.header.stamp = node->now() + rclcpp::Duration(3ms);
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster->sendTransform(t);

    // Allow tf_buffer_ to be filled by listener
    waitSome(10ms, node);
  }
}

/**
 * Print costmap to stdout.
 * @param costmap map to be printed.
 */
void printMap(const nav2_costmap_2d::Costmap2D & costmap)
{
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      printf("%4d", static_cast<int>(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

/**
 * Print costmap with trajectory and goal point to stdout.
 * @param costmap map to be printed.
 * @param trajectory trajectory container (xt::tensor) to be printed.
 * @param goal_point goal point to be printed.
 */
template<typename TTrajectory>
void printMapWithTrajectoryAndGoal(
  nav2_costmap_2d::Costmap2D & costmap, const TTrajectory & trajectory,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const unsigned int trajectory_cost = 1;
  const unsigned int goal_cost = 2;

  std::cout << "Costmap: \n trajectory = " << trajectory_cost << "\n goal = " << goal_cost
            << "\n obsctacle = 255 \n";

  // create new costmap
  nav2_costmap_2d::Costmap2D costmap2d(
    costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), costmap.getResolution(),
    costmap.getOriginX(), costmap.getOriginY(), costmap.getDefaultValue());

  // copy obstacles from original costmap
  costmap2d = costmap;

  // add trajectory on map
  unsigned int point_mx = 0;
  unsigned int point_my = 0;
  for (size_t i = 0; i < trajectory.shape()[0]; ++i) {
    costmap2d.worldToMap(trajectory(i, 0), trajectory(i, 1), point_mx, point_my);
    costmap2d.setCost(point_mx, point_my, trajectory_cost);
  }

  unsigned int goal_j{0};
  unsigned int goal_i{0};
  costmap2d.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_j, goal_i);
  std::cout << "Goal Position: " << goal_j << " " << goal_i << "\n";
  costmap2d.setCost(goal_j, goal_i, goal_cost);
  printMap(costmap2d);
}

/**
 * Add a square obstacle to the costmap.
 * @param costmap map to be modified.
 * @param upper_left_corner_x obstacle upper left corner X coord (on the
 * costmap).
 * @param upper_left_corner_y obstacle upper left corner Y coord (on the
 * costmap).
 * @param size obstacle side size.
 * @param cost obstacle value on costmap.
 */
void addObstacle(
  nav2_costmap_2d::Costmap2D * costmap, unsigned int upper_left_corner_x,
  unsigned int upper_left_corner_y, unsigned int size, unsigned char cost)
{
  for (unsigned int i = upper_left_corner_x; i < upper_left_corner_x + size; i++) {
    for (unsigned int j = upper_left_corner_y; j < upper_left_corner_y + size; j++) {
      costmap->setCost(i, j, cost);
    }
  }
}

void printInfo(
  TestOptimizerSettings os, TestPathSettings ps,
  const std::vector<std::string> & critics)
{
  std::stringstream ss;
  for (auto str : critics) {
    ss << str << " ";
  }

  std::cout <<  //
    "\n\n--------------------OPTIMIZER OPTIONS-----------------------------\n" <<
    "Critics: " << ss.str() << "\n" \
    "Motion model: " << os.motion_model << "\n"
    "Consider footprint: " << os.consider_footprint << "\n" <<
    "Iterations: " << os.iteration_count << "\n" <<
    "Batch size: " << os.batch_size << "\n" <<
    "Time steps: " << os.time_steps << "\n" <<
    "Path points: " << ps.poses_count << "\n" <<
    "\n-------------------------------------------------------------------\n\n";
}

void addObstacle(nav2_costmap_2d::Costmap2D * costmap, TestObstaclesSettings s)
{
  addObstacle(costmap, s.center_cells_x, s.center_cells_y, s.obstacle_size, s.obstacle_cost);
}

/**
 * Check the trajectory for collisions with obstacles on the map.
 * @param trajectory trajectory container (xt::tensor) to be checked.
 * @param costmap costmap with obstacles
 * @return true - if the trajectory crosses an obstacle on the map, false - if
 * not
 */
template<typename TTrajectory>
bool inCollision(const TTrajectory & trajectory, const nav2_costmap_2d::Costmap2D & costmap)
{
  unsigned int point_mx = 0;
  unsigned int point_my = 0;

  for (size_t i = 0; i < trajectory.shape(0); ++i) {
    costmap.worldToMap(trajectory(i, 0), trajectory(i, 1), point_mx, point_my);
    auto cost_ = costmap.getCost(point_mx, point_my);
    if (cost_ > nav2_costmap_2d::FREE_SPACE || cost_ == nav2_costmap_2d::NO_INFORMATION) {
      return true;
    }
  }
  return false;
}

unsigned char getCost(const nav2_costmap_2d::Costmap2D & costmap, double x, double y)
{
  unsigned int point_mx = 0;
  unsigned int point_my = 0;

  costmap.worldToMap(x, y, point_mx, point_my);
  return costmap.getCost(point_mx, point_my);
}

template<typename TTrajectory>
bool isGoalReached(
  const TTrajectory & trajectory, const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & goal)
{
  unsigned int trajectory_j = 0;
  unsigned int trajectory_i = 0;

  unsigned int goal_j = 0;
  unsigned int goal_i = 0;
  costmap.worldToMap(goal.pose.position.x, goal.pose.position.y, goal_j, goal_i);

  auto match = [](unsigned int i, unsigned int j, unsigned int i_dst, unsigned int j_dst) {
      if (i == i_dst && j == j_dst) {
        return true;
      }
      return false;
    };

  auto match_near = [&](unsigned int i, unsigned int j) {
      if (match(i, j, goal_i, goal_j) ||
        match(i, j, goal_i + 1, goal_j) ||
        match(i, j, goal_i - 1, goal_j) ||
        match(i, j, goal_i, goal_j + 1) ||
        match(i, j, goal_i, goal_j - 1) ||
        match(i, j, goal_i + 1, goal_j + 1) ||
        match(i, j, goal_i + 1, goal_j - 1) ||
        match(i, j, goal_i - 1, goal_j + 1) ||
        match(i, j, goal_i - 1, goal_j - 1))
      {
        return true;
      }
      return false;
    };
  // clang-format on

  for (size_t i = 0; i < trajectory.shape(0); ++i) {
    costmap.worldToMap(trajectory(i, 0), trajectory(i, 1), trajectory_j, trajectory_i);
    if (match_near(trajectory_i, trajectory_j)) {
      return true;
    }
  }

  return false;
}
