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
#include <vector>
#include <string>

#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"
#include "smac_planner/costmap_downsampler.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace smac_planner
{

class SmacPlanner2D : public nav2_core::GlobalPlanner
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
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

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
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /**
   * @brief Create an Eigen Vector2D of world poses from continuous map coords
   * @param mx float of map X coordinate
   * @param my float of map Y coordinate
   * @param costmap Costmap pointer
   * @return Eigen::Vector2d eigen vector of the generated path
   */
  Eigen::Vector2d getWorldCoords(
    const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap);

  /**
   * @brief Remove hooking at end of paths
   * @param path Path to remove hooking from
   */
  void removeHook(std::vector<Eigen::Vector2d> & path);

protected:
  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  std::unique_ptr<Smoother> _smoother;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlanner2D")};
  std::string _global_frame, _name;
  float _tolerance;
  int _downsampling_factor;
  bool _downsample_costmap;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;
  double _max_planning_time;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
