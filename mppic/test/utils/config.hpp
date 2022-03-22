// Copyright 2022 FastSense, Samsung Research
#pragma once
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

struct TestOptimizerSettings
{
  const int iteration_count;
  const int time_steps;
  const double lookahead_distance;
  std::string motion_model;
  bool consider_footprint;
};

struct TestPose
{
  double x;
  double y;
};

struct TestCostmapSettings
{
  const unsigned int cells_x = 40;
  const unsigned int cells_y = 40;
  const double origin_x = 0.0;
  const double origin_y = 0.0;
  const double resolution = 0.1;
  const unsigned char cost_map_default_value = 0;

  TestPose getCenterPose()
  {
    return {
      static_cast<double>(cells_x) * resolution / 2.0,
      static_cast<double>(cells_y) * resolution / 2.0};
  }
};
struct TestObstaclesSettings
{
  unsigned int center_cells_x;
  unsigned int center_cells_y;
  unsigned int obstacle_size;
  unsigned char obstacle_cost;
};

struct TestPathSettings
{
  TestPose start_pose;
  unsigned int poses_count;
  double step_x;
  double step_y;
};

/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
void setUpOptimizerParams(
  int iter, int time_steps, double lookahead_dist, std::string motion_model,
  bool /*consider_footprint*/, std::vector<rclcpp::Parameter> & params_,
  std::string node_name = std::string("dummy"))
{
  double dummy_freq = 10.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".iteration_count", iter));
  params_.emplace_back(rclcpp::Parameter(node_name + ".time_steps", time_steps));
  params_.emplace_back(rclcpp::Parameter(node_name + ".lookahead_dist", lookahead_dist));
  params_.emplace_back(rclcpp::Parameter(node_name + ".motion_model", motion_model));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));

  std::string critic_scorer_name = node_name;
  params_.emplace_back(
    rclcpp::Parameter(
      critic_scorer_name + ".critics_names",
      std::vector<std::string>{
    "GoalCritic", "GoalAngleCritic", "ReferenceTrajectoryCritic", "ObstaclesCritic"}));
}

void setUpControllerParams(
  bool visualize, std::vector<rclcpp::Parameter> & params_,
  std::string node_name = std::string("dummy"))
{
  double dummy_freq = 10.0;
  params_.emplace_back(rclcpp::Parameter(node_name + ".visualize", visualize));
  params_.emplace_back(rclcpp::Parameter("controller_frequency", dummy_freq));
}
