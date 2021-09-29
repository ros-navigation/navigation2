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
  using ActionServer = nav2_util::SimpleActionServer<AssistedTeleopAction,
      rclcpp_lifecycle::LifecycleNode>;

  /**
   * @brief AsistedTeleop constructor
   */
  AssistedTeleop()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false)
  {
  }

  ~AssistedTeleop() = default;

  // configure the server and Assisted Teleop Action on lifecycle setup
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override;

  // Cleanup server on lifecycle transition
  void cleanup() override;

  // Activate server on lifecycle transition
  void activate() override;

  // Deactivate server on lifecycle transition
  void deactivate() override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Action Server
  std::shared_ptr<ActionServer> action_server_;

  // Publishers and Subscribers
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  // User defined parameters
  double projection_time_;
  double cycle_frequency_;
  double linear_velocity_threshold_;
  double enabled_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string cmd_vel_topic_;
  std::string input_vel_topic_;

  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::Pose2D projected_pose;
  std::string recovery_name_;
  std::chrono::time_point<std::chrono::steady_clock> assisted_teleop_end_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;

  // Parameters for Assisted Teleop
  bool go = false;
  double scaling_factor = 1;
  double speed_x = 0.0, speed_y = 0.0, angular_vel_ = 0.0;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_recoveries")};

  /**
   * @brief Utility function to obtain robot pose
   * @return bool indicating availability of pose
   */
  bool updatePose();

  /**
   * @brief Utility function to project robot pose
   * @param speed_x linear speed in X
   * @param speed_y linear speed in y
   * @param speed_x angular speed about Z
   */
  void projectPose(double speed_x, double speed_y, double angular_vel_, double projection_time);

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Check if pose is collision free
   */
  bool checkCollision();

  /**
   * @brief Move robot by specified velocity
   */
  void moveRobot();

  /**
   * @brief Main execution callbacks for the action server, enable's recovery's specific behavior as it waits for timeout
   */
  void execute();

  /**
   * @brief Stops the robot with a commanded velocity
   */
  void stopRobot();
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_