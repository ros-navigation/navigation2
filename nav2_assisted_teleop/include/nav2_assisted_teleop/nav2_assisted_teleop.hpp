// Copyright (c) 2022 Joshua Wallace
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


#ifndef NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_
#define NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_


#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace nav2_assisted_teleop
{

/**
 * @class nav2_assisted_teleop::AssistedTeleop
 * @brief This class contians the core functionality for
 * calculating the reduced/refined the teleop velocity command
 */
class AssistedTeleop
{
public:

  AssistedTeleop(rclcpp::Logger& logger);

  /**
   * @brief configure the assisted teleop
   *
   * @param node ros node handle
   */
  void on_configure(std::shared_ptr<nav2_util::LifecycleNode> node,
                    std::shared_ptr<tf2_ros::Buffer> tf,
                    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker);
  /**
   * @brief Reduce velocity if a collision is imminent
   * @param twist current speed
   * @return geometry_msgs::msg::Twist reduced velocity
   */
  geometry_msgs::msg::Twist computeVelocity(geometry_msgs::msg::Twist& twist);

private:
  /**
   * @brief preform a linear projection of a pose
   * @param pose the pose to project
   * @param twist speed to project the pose by
   * @param projected_time amount of time to project the pose by
   * @return geometry_msgs::msg::Pose the projected pose
   */
  geometry_msgs::msg::Pose2D projectPose(geometry_msgs::msg::Pose2D pose,
                                          geometry_msgs::msg::Twist twist,
                                          double projection_time);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  // params
  double transform_tolerance_;
  double projection_time_;

  // frames
  std::string global_frame_;
  std::string robot_base_frame_;

  rclcpp::Logger logger_;
};

/**
 * @class nav2_assisted_teleop::AssistedTeleopServer
 * @brief This class provides a action server refine/reduce commanded
 * velocities from a teleoperator to prevent collisions
 */
class AssistedTeleopServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_assisted_teleop::AssistedTeleopServer
   * @param options Additional options to control creation of the node.
   */
  explicit AssistedTeleopServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the nav2_assisted_teleop::AssistedTeleopServer
   */
  // ~AssistedTeleopServer();

protected:
  /**
   * @brief Configures assisted teleop params and member variable
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Activates assisted teleop server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Deactivates assisted teleop server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Calls cleanup on states and resets member variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  void inputVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Callback function for joy subscriber
   * @param msg received Twist message
   */
  void joyCallback(const sensor_msgs::msg::Joy msg);

  /**
   * @brief AssistedTeleop action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * While this action is running, commands from joystick are filtered and then published to the base.
   */
  void adjustVelocityIfInvalid();

  using Action = nav2_msgs::action::AssistedTeleop;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Action server that implements AssistedTeleop action
  std::unique_ptr<ActionServer> action_server_;

  // Helper class to make adjustments to velocity command
  rclcpp::Logger logger_;
  std::shared_ptr<AssistedTeleop> assisted_teleop_;


  // Transforms
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Sub/Pubs
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // costmap utils
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

  // Twist message
  geometry_msgs::msg::Twist input_twist_;
  sensor_msgs::msg::Joy joy_;

  // Parameters
  double transform_tolerance_;
  double projection_time_;
  std::string input_vel_topic_;
  std::string output_vel_topic_;
  std::string joystick_topic_;
  std::string robot_base_frame_;

  rclcpp::Clock steady_clock_;
};

}

#endif  // NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_
