// Copyright 2022 FastSense, Samsung Research
#pragma once

#include <algorithm>
#include <iostream>
#include <rclcpp/executors.hpp>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "config.hpp"
#include "factory.hpp"

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
void printMapWithTrajectoryAndGoal(
  nav2_costmap_2d::Costmap2D & costmap, const auto & trajectory,
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

void print_info(TestOptimizerSettings os, TestPathSettings ps)
{
  std::cout <<
    "Parameters of MPPI Planner:" <<
    "Points in path " << ps.poses_count << "\n" <<
    "Iteration_count " << os.iteration_count << "\n" <<
    "Time_steps " << os.time_steps << "\n" <<
    "Motion model " << os.motion_model << "\n"
    "Is footprint considering " << os.consider_footprint << "\n" << std::endl;
}

void print_info(TestOptimizerSettings os, unsigned int poses_count)
{
  std::cout <<
    "Points in path " << poses_count << "\niteration_count " << os.iteration_count
            << "\ntime_steps " << os.time_steps
            << "\nMotion model : " << os.motion_model << std::endl;
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
bool inCollision(const auto & trajectory, const nav2_costmap_2d::Costmap2D & costmap)
{
  unsigned int point_mx = 0;
  unsigned int point_my = 0;

  for (size_t i = 0; i < trajectory.shape()[0]; ++i) {
    costmap.worldToMap(trajectory(i, 0), trajectory(i, 1), point_mx, point_my);
    auto cost_ = costmap.getCost(point_mx, point_my);
    if (cost_ > nav2_costmap_2d::FREE_SPACE || cost_ == nav2_costmap_2d::NO_INFORMATION) {
      return true;
    }
  }
  return false;
}

bool isGoalReached(
  const auto & trajectory, const nav2_costmap_2d::Costmap2D & costmap,
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

  for (size_t i = 0; i < trajectory.shape()[0]; ++i) {
    costmap.worldToMap(trajectory(i, 0), trajectory(i, 1), trajectory_j, trajectory_i);
    if (match_near(trajectory_i, trajectory_j)) {
      return true;
    }
  }

  return false;
}
