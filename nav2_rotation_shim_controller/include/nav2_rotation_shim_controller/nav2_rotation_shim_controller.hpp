// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_
#define NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "angles/angles.h"

namespace nav2_rotation_shim_controller
{

/**
 * @class nav2_rotation_shim_controller::RotationShimController
 * @brief Rotate to rough path heading controller shim plugin
 */
class RotationShimController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_rotation_shim_controller::RotationShimController
   */
  RotationShimController();

  /**
   * @brief Destrructor for nav2_rotation_shim_controller::RotationShimController
   */
  ~RotationShimController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Finds the point on the path that is roughly the sampling
   * point distance away from the robot for use.
   * May throw exception if a point at least that far away cannot be found
   * @return pt location of the output point
   */
  geometry_msgs::msg::PoseStamped getSampledPathPt();

  /**
   * @brief Find the goal point in path
   * May throw exception if the path is empty
   * @return pt location of the output point
   */
  geometry_msgs::msg::PoseStamped getSampledPathGoal();

  /**
   * @brief Uses TF to find the location of the sampled path point in base frame
   * @param pt location of the sampled path point
   * @return location of the pose in base frame
   */
  geometry_msgs::msg::Pose transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt);

  /**
   * @brief Rotates the robot to the rough heading
   * @param angular_distance Angular distance to the goal remaining
   * @param pose Starting pose of robot
   * @param velocity Starting velocity of robot
   * @return Twist command for rotation to rough heading
   */
  geometry_msgs::msg::TwistStamped computeRotateToHeadingCommand(
    const double & angular_distance,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief Checks if rotation is safe
   * @param cmd_vel Velocity to check over
   * @param angular_distance_to_heading Angular distance to heading requested
   * @param pose Starting pose of robot
   */
  void isCollisionFree(
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RotationShimController")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  nav2_core::Controller::Ptr primary_controller_;
  bool path_updated_;
  nav_msgs::msg::Path current_path_;
  double forward_sampling_distance_, angular_dist_threshold_, angular_disengage_threshold_;
  double rotate_to_heading_angular_vel_, max_angular_accel_;
  double control_duration_, simulate_ahead_time_;
  bool rotate_to_goal_heading_, in_rotation_;
  bool closed_loop_;
  double last_angular_vel_ = std::numeric_limits<double>::max();

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_rotation_shim_controller

#endif  // NAV2_ROTATION_SHIM_CONTROLLER__NAV2_ROTATION_SHIM_CONTROLLER_HPP_
