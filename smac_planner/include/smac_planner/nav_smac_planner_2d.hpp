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

#ifndef SMAC_PLANNER__NAV_SMAC_PLANNER_2D_HPP_
#define SMAC_PLANNER__NAV_SMAC_PLANNER_2D_HPP_

#include <memory>
#include <string>
#include <vector>

#include "costmap_2d/footprint_collision_checker.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_2d_msgs/Path2D.h"
#include "nav_core2/costmap.h"
#include "nav_core/base_global_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "smac_planner/a_star.hpp"
#include "smac_planner/collision_checker.hpp"
#include "smac_planner/costmap_downsampler.hpp"
#include "smac_planner/smoother.hpp"
#include "tf2/utils.h"

namespace smac_planner
{
class SmacPlanner2D : public nav_core::BaseGlobalPlanner
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
   * @brief  Initialization function for the SmacPlanner
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup();

  /**
   * @brief Activate lifecycle node
   */
  void activate();

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate();

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(
    const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal,
    std::vector<geometry_msgs::PoseStamped> & planVector) override;

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
  std::unique_ptr<AStarAlgorithm<
      Node2D<GridCollisionChecker<
        costmap_2d::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
        costmap_2d::Footprint>>,
      GridCollisionChecker<
        costmap_2d::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
        costmap_2d::Footprint>,
      costmap_2d::Costmap2D, costmap_2d::Footprint>>
  _a_star;
  std::unique_ptr<Smoother<costmap_2d::Costmap2D>> _smoother;
  costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<CostmapDownsampler<costmap_2d::Costmap2D>> _costmap_downsampler;
  std::string _global_frame, _name;
  float _tolerance;
  int _downsampling_factor;
  bool _downsample_costmap;
  std::unique_ptr<ros::Publisher> _raw_plan_publisher;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;
  double _max_planning_time;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NAV_SMAC_PLANNER_2D_HPP_
