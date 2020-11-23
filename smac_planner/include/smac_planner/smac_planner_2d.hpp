// Copyright (c) 2020, Samsung Research America
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

#ifndef SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
#define SMAC_PLANNER__SMAC_PLANNER_2D_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_2d_msgs/Path2D.h"
#include "nav_core2/costmap.h"
#include "nav_core2/global_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "smac_planner/a_star.hpp"
#include "smac_planner/costmap_downsampler.hpp"
#include "smac_planner/smoother.hpp"
#include "tf2/utils.h"

namespace smac_planner
{
class SmacPlanner2D : public nav_core2::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlanner2D();

  /**
   * @brief destructor
   */
  ~SmacPlanner2D();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros CostmapPtr
   */
  void initialize(
    const ros::NodeHandle & parent, const std::string & name, TFListenerPtr tf,
    nav_core2::Costmap::Ptr costmap) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_2d_msgs::Path2D makePlan(
    const nav_2d_msgs::Pose2DStamped & start, const nav_2d_msgs::Pose2DStamped & goal) override;

  /**
   * @brief Create an Eigen Vector2D of world poses from continuous map coords
   * @param mx float of map X coordinate
   * @param my float of map Y coordinate
   * @param costmap Costmap pointer
   * @return Eigen::Vector2d eigen vector of the generated path
   */
  Eigen::Vector2d getWorldCoords(
    const float & mx, const float & my, const costmap_2d::Costmap2D * costmap);

  /**
   * @brief Remove hooking at end of paths
   * @param path Path to remove hooking from
   */
  void removeHook(std::vector<Eigen::Vector2d> & path);

private:
  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  std::unique_ptr<Smoother> _smoother;
  costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _tolerance;
  int _downsampling_factor;
  bool _downsample_costmap;
  ros::Publisher _raw_plan_publisher;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;
  double _max_planning_time;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
