// Copyright (c) 2021 Anushree Sabnis
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

#ifndef NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/recovery.hpp"

#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_recoveries
{

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_recoveries::AssistedTeleop
 * @brief An action server recovery base class implementing the action server and basic factory.
 */
class AssistedTeleop : public nav2_core::Recovery
{
public:
  using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;
  using ActionServer = nav2_util::SimpleActionServer<AssistedTeleopAction>;

  /**
   * @brief AsistedTeleop constructor
   */
  AssistedTeleop();

  /**
   * @brief AsistedTeleop destructor
   */
  ~AssistedTeleop() = default;

  /**
   * @brief configure the server and Assisted Teleop Action on lifecycle setup
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override;

  // Activate server on lifecycle transition
  void activate() override;

  // Deactivate server on lifecycle transition
  void deactivate() override;

  // Cleanup server on lifecycle transition
  void cleanup() override;

protected:
  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_recoveries")};


  /**
   * @brief Utility function to project robot pose
   * @param twist_speed linear speed of robot
   * @param angular_vel angular speed about Z
   * @param projection_time time increment to project the pose
   * @param projected_pose Pose projected by given linear and angular speeds
   */
  void projectPose(
    geometry_msgs::msg::Vector3Stamped & twist_speed, double angular_vel,
    double projection_time, geometry_msgs::msg::Pose2D & projected_pose);

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Performs velocity reduction if a collision is imminent
   * @param twist_speed linear speed of robot
   * @param angular_vel angular speed about Z
   */
  geometry_msgs::msg::Twist::UniquePtr computeVelocity(
    geometry_msgs::msg::Vector3Stamped & twist_speed, double angular_vel);

  /**
   * @brief Move robot by specified velocity
   * @param cmd_vel Velocity with which to move the robot
   */
  void moveRobot(geometry_msgs::msg::Twist::UniquePtr cmd_vel);

  /**
   * @brief Main execution callbacks for the action server, enable's recovery's specific behavior as it waits for timeout
   */
  void execute();

  /**
   * @brief Stops the robot with a commanded velocity
   */
  void stopRobot();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Action Server
  std::shared_ptr<ActionServer> action_server_;

  // Publishers and Subscribers
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  // User defined parameters
  double projection_time_;
  double cycle_frequency_;
  double linear_velocity_threshold_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string cmd_vel_topic_;
  std::string input_vel_topic_;

  geometry_msgs::msg::PoseStamped current_pose;
  std::string recovery_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;
  geometry_msgs::msg::TransformStamped transform;

  // Parameters for Assisted Teleop
  double scaling_factor = 1;
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_
