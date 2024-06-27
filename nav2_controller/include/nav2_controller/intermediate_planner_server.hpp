// Copyright (c) 2019 Intel Corporation
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

/* Copyright 2023 Enjoy Robotics Zrt - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Modifications to this file is to be shared with the code owner.
 * Proprietary and confidential
 * Owner: Enjoy Robotics Zrt maintainer@enjoyrobotics.com, 2024
 */

#ifndef NAV2_CONTROLLER__INTERMEDIATE_PLANNER_SERVER_HPP_
#define NAV2_CONTROLLER__INTERMEDIATE_PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_core/planner_exceptions.hpp"

#include "service_robot_msgs/action/compute_local_path.hpp"

namespace nav2_controller
{
/**
 * @class nav2_controller::IntermediateIntermediatePlannerServer
 * @brief An action server implements the behavior tree's ComputeLocalPath
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class IntermediatePlannerServer
{
public:
  /**
   * @brief A constructor for nav2_planner::IntermediatePlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit IntermediatePlannerServer(
    nav2_util::LifecycleNode::SharedPtr parent,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief A destructor for nav2_planner::IntermediatePlannerServer
   */
  ~IntermediatePlannerServer();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn configure();
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn activate();
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn deactivate();
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn cleanup();

protected:
  using ActionToPose = service_robot_msgs::action::ComputeLocalPath;
  using ActionToPoseGoal = ActionToPose::Goal;
  using ActionServerToPose = nav2_util::SimpleActionServer<ActionToPose>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  template<typename T>
  bool getStartPose(
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Transform start and goal poses into the costmap
   * global frame for path planning plugins to utilize
   * @param start The starting pose to transform
   * @param goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  bool transformPosesToGlobalFrame(
    geometry_msgs::msg::PoseStamped & curr_start,
    geometry_msgs::msg::PoseStamped & curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param action_server Action server to terminate if required
   * @param goal Goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  template<typename T>
  bool validatePath(
    const geometry_msgs::msg::PoseStamped & curr_goal,
    const nav_msgs::msg::Path & path,
    const std::string & planner_id);

  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathToPose
   */
  void computePlan();

  /**
   * @brief The service callback to determine if the path is still valid
   * @param request to the service
   * @param response from the service
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

  void exceptionWarning(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id,
    const std::exception & ex);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Parent node (didn't make this its own node because otherwise
  // it behaved differently composed vs standalone: if composed,
  // it would be a part of the controller_server node, but if standalone,
  // it would be its own node)
  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("intermediate_planner_server")};

  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    intermediate_goal_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    spiral_markers_pub_;
  bool publish_spiral_markers_;

  // Tolerance parameters
  double tolerance_;
  int n_points_near_goal_;
  int points_per_rotation_;

  // Service to determine if the path is valid
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__INTERMEDIATE_PLANNER_SERVER_HPP_
