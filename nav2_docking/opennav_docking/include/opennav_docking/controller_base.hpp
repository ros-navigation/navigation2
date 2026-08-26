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
#include <mutex>
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
#include "nav_msgs/msg/path.hpp"

namespace opennav_docking
{

/**
 * @struct TrajectoryOptions
 * @brief Describes how the trajectory handed to ControllerBase::setTrajectory is to be driven.
 */
struct TrajectoryOptions
{
  /// @brief If true, the robot drives in reverse along the trajectory.
  bool reverse{false};

  /// @brief True while approaching a dock, false while undocking.
  bool approaching{true};
};

/**
 * @class opennav_docking::ControllerBase
 * @brief Base class for docking controllers, and the pluginlib base type.
 *
 * A controller consumes a trajectory to the dock and produces velocity commands.
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
   * @brief Activate the trajectory publisher.
   */
  virtual void activate();

  /**
   * @brief Deactivate the trajectory publisher.
   */
  virtual void deactivate();

  /**
   * @brief Reset internal state.
   *
   * Called by the server once on entry to each control loop and once per docking retry.
   */
  virtual void reset() {}

  /**
   * @brief Set trajectory to follow.
   *
   * @param trajectory The trajectory to follow.
   * @param options How the trajectory is to be driven.
   */
  virtual void setTrajectory(
    const nav_msgs::msg::Path & trajectory,
    const TrajectoryOptions & options = TrajectoryOptions());

  /**
   * @brief Compute a velocity command towards the end of the cached trajectory
   *
   * @param robot_pose Current pose of the robot.
   * @param velocity Current velocity of the robot.
   * @param dt Control loop duration [s].
   * @param cmd Output command velocity.
   * @returns True if the command is valid, false otherwise.
   */
  virtual bool computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
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

protected:
  /**
   * @brief Declare and read the parameters specific to the derived control law.
   *
   * Called by configure() after the shared parameters have been read and before the dynamic
   * parameter callbacks are registered.
   *
   * @param node Lifecycle node
   */
  virtual void configureController(const nav2::LifecycleNode::SharedPtr & node) = 0;

  /**
   * @brief Apply the control law to produce a velocity command.
   *
   * Called with dynamic_params_lock_ held; implementations must not take it again.
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
   * Called with dynamic_params_lock_ held; implementations must not take it again.
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

  /**
   * @brief Apply one dynamic parameter update.
   *
   * Called with dynamic_params_lock_ held, once per updated parameter belonging to this
   * controller's namespace. Derived overrides should call this base implementation so the
   * shared parameters keep working.
   *
   * @param name The parameter name with this controller's namespace prefix stripped.
   * @param parameter The parameter being updated.
   */
  virtual void updateParameter(const std::string & name, const rclcpp::Parameter & parameter);

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
   * @brief Get the end of the trajectory expressed in the robot's base frame.
   *
   * @param target_pose Output target pose, in robot centric coordinates.
   * @return True if the target pose could be resolved.
   */
  bool getTargetInBaseFrame(geometry_msgs::msg::Pose & target_pose);

  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  virtual rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

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

  // Dynamic parameters handler
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  std::mutex dynamic_params_lock_;

  rclcpp::Logger logger_{rclcpp::get_logger("Controller")};
  rclcpp::Clock::SharedPtr clock_;

  // In-place rotation profile
  double rotate_to_heading_angular_vel_, rotate_to_heading_max_angular_accel_;

  // The trajectory to follow and how to drive it
  nav_msgs::msg::Path trajectory_;
  TrajectoryOptions trajectory_options_;

  // The trajectory of the robot while dock / undock for visualization / debug purposes
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  // Used for collision checking
  bool use_collision_detection_;
  double projection_time_;
  double simulation_time_step_;
  double dock_collision_threshold_;
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
