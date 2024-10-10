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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_smac_planner
{

class SmacPlannerHybrid : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
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
   * @param cancel_checker Function to check if the action has been canceled
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)  override;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  bool _debug_visualizations;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  GoalHeadingMode _goal_heading_mode;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    _planned_footprints_publisher;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    _expansions_publisher;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
  std::shared_ptr<rclcpp::ParameterEventHandler> _remote_param_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> _remote_resolution_handler;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
