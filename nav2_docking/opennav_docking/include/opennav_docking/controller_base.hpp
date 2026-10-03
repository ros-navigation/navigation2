// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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

#ifndef OPENNAV_DOCKING__CONTROLLER_BASE_HPP_
#define OPENNAV_DOCKING__CONTROLLER_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav_msgs/msg/path.hpp"

namespace opennav_docking
{

/**
 * @struct DockingOptions
 * @brief Describes how the target handed to ControllerBase::computeVelocityCommands is to be
 * driven.
 */
struct DockingOptions
{
  /// @brief If true, the robot drives in reverse along the trajectory.
  bool reverse{false};

  /// @brief True when leaving the dock
  bool undocking{false};
};

/**
 * @struct ControllerParameters
 * @brief Dynamic parameters every controller shares.
 */
struct ControllerParameters
{
  double rotate_to_heading_angular_vel;
  double rotate_to_heading_max_angular_accel;
  double projection_time;
  double simulation_time_step;
  double dock_collision_threshold;
};

/**
 * @class opennav_docking::ControllerParameterHandler
 * @brief Handles the dynamic parameters.
 */
class ControllerParameterHandler : public nav2_util::ParameterHandler<ControllerParameters>
{
public:
  /**
   * @brief Declare shared parameters in controller namespace.
   * @param node Lifecycle node
   * @param name The parameter namespace
   * @param logger Logger
   */
  ControllerParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
    const rclcpp::Logger & logger);

protected:
  /**
   * @brief Validate parameters before applying.
   * @param parameters List of parameters.
   * @return rcl_interfaces::msg::SetParametersResult Result of the update request.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Apply parameter after validation
   * @param parameters List of parameters updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  std::string name_;
};

/**
 * @class opennav_docking::ControllerBase
 * @brief Base class for docking controllers, and the pluginlib base type.
 *
 * A controller consumes a target pose near the dock and produces velocity commands.
 */
class ControllerBase
{
public:
  using Ptr = std::shared_ptr<ControllerBase>;

  /**
   * @brief Virtual destructor
   */
  virtual ~ControllerBase() = default;

  /**
   * @brief Configure the controller. Declares and reads ROS 2 parameters.
   *
   * @param parent Lifecycle node
   * @param name The name of this controller instance; also its parameter namespace
   * @param tf tf2_ros TF buffer
   */
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name, nav2::TransformBuffer::SharedPtr tf);

  /**
   * @brief Release the resources acquired in configure().
   */
  virtual void cleanup();

  /**
   * @brief Activate parameters callbacks.
   */
  virtual void activate();

  /**
   * @brief Deactivate parameters callbacks.
   */
  virtual void deactivate();

  /**
   * @brief Reset internal state.
   *
   * Called by the server once on entry to each control loop and once per docking retry.
   */
  virtual void reset() {}

  /**
   * @brief Compute a velocity command towards the target pose
   *
   * @param robot_pose Current pose of the robot.
   * @param velocity Current velocity of the robot.
   * @param target Target pose, in the robot's base frame.
   * @param options How the target is to be driven to.
   * @param dt Control loop duration [s].
   * @param cmd Output command velocity.
   * @returns True if the command is valid, false otherwise.
   */
  virtual bool computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    const geometry_msgs::msg::Pose & target,
    const DockingOptions & options,
    double dt,
    geometry_msgs::msg::Twist & cmd);

  /**
   * @brief Perform a command for in-place rotation.
   *
   * @param angular_distance_to_heading Angular distance to goal.
   * @param current_velocity Current angular velocity.
   * @param dt Control loop duration [s].
   * @returns TwistStamped command for in-place rotation.
   */
  virtual geometry_msgs::msg::Twist computeRotateToHeadingCommand(
    const double & angular_distance_to_heading,
    const geometry_msgs::msg::Twist & current_velocity,
    const double & dt);

  /**
   * @brief Gets the name of this controller instance
   */
  std::string getName() {return name_;}

  /**
   * @brief Declare and read the parameters specific to the derived control law.
   *
   * Called by configure() after the shared parameters have been read.
   *
   * @param node Lifecycle node
   */
  virtual void onConfigure(const nav2::LifecycleNode::SharedPtr & node) = 0;

  /**
   * @brief Apply the control law to produce a velocity command.
   *
   * Called with the shared parameter lock held; implementations must not take it again.
   *
   * @param target Target pose, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @param dt Control loop duration [s].
   * @returns Command velocity.
   */
  virtual geometry_msgs::msg::Twist computeCommand(
    const geometry_msgs::msg::Pose & target, bool reverse, double dt) = 0;

  /**
   * @brief Forward-simulate one step of the control law, for collision checking.
   *
   * Called with the shared parameter lock held; implementations must not take it again.
   *
   * @param dt Simulation time step [s].
   * @param target Target pose, in robot centric coordinates.
   * @param current Current pose along the projection, in robot centric coordinates.
   * @param reverse If true, robot will drive backwards to the goal.
   * @returns The pose one step further along the projection.
   */
  virtual geometry_msgs::msg::Pose predictNextPose(
    double dt, const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current, bool reverse) = 0;

protected:
  /**
   * @brief Check if a trajectory is collision free.
   *
   * @param target_pose Target pose, in robot centric coordinates.
   * @param is_docking If true, robot is docking. If false, robot is undocking.
   * @param backward If true, robot will drive backwards to goal.
   * @return True if trajectory is collision free.
   */
  bool isTrajectoryCollisionFree(
    const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward = false);

  /**
   * @brief Configure the collision checker.
   *
   * @param node Lifecycle node
   * @param costmap_topic Costmap topic
   * @param footprint_topic Footprint topic
   * @param transform_tolerance Transform tolerance
   */
  void configureCollisionChecker(
    const nav2::LifecycleNode::SharedPtr & node,
    std::string costmap_topic, std::string footprint_topic, double transform_tolerance);

  std::unique_ptr<ControllerParameterHandler> param_handler_;
  ControllerParameters * params_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("Controller")};
  rclcpp::Clock::SharedPtr clock_;

  // The trajectory of the robot while dock / undock for visualization / debug purposes
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  // Used for collision checking
  bool use_collision_detection_;
  double transform_tolerance_;
  nav2::TransformBuffer::SharedPtr tf2_buffer_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::string fixed_frame_, base_frame_;
  std::string name_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_BASE_HPP_
