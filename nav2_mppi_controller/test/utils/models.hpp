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
#include <vector>
#include <utility>
#include <string>
#include <rclcpp/rclcpp.hpp>

struct TestOptimizerSettings
{
  int batch_size;
  int time_steps;
  int iteration_count;
  double lookahead_distance;
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
  const double footprint_size = 0.15;

  std::pair<unsigned int, unsigned int> getCenterIJ()
  {
    return {
      cells_x / 2,
      cells_y / 2};
  }

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
