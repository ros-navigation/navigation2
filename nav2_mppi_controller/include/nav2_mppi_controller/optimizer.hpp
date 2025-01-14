// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/goal_checker.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critic_manager.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */
class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  ~Optimizer() {shutdown();}


  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    ParametersHandler * dynamic_parameters_handler);

  /**
   * @brief Shutdown for optimizer at process end
   */
  void shutdown();

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
  geometry_msgs::msg::TwistStamped evalControl(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal, nav2_core::GoalChecker * goal_checker);

  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
  models::Trajectories & getGeneratedTrajectories();

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
  xt::xtensor<float, 2> getOptimizedTrajectory();

  /**
   * @brief Set the maximum speed based on the speed limits callback
   * @param speed_limit Limit of the speed for use
   * @param percentage Whether the speed limit is absolute or relative
   */
  void setSpeedLimit(double speed_limit, bool percentage);

  /**
   * @brief Reset the optimization problem to initial conditions
   */
  void reset();

protected:
  /**
   * @brief Main function to generate, score, and return trajectories
   */
  void optimize();

  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
  void prepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal, nav2_core::GoalChecker * goal_checker);

  /**
   * @brief Obtain the main controller's parameters
   */
  void getParams();

  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
  void setMotionModel(const std::string & model);

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
  void shiftControlSequence();

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
  void generateNoisedTrajectories();

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
  void applyControlSequenceConstraints();

  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
  void updateStateVelocities(models::State & state) const;

  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
  void updateInitialStateVelocities(models::State & state) const;

  /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    xt::xtensor<float, 2> & trajectories,
    const xt::xtensor<float, 2> & state) const;

  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
  void updateControlSequence();

  /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
  geometry_msgs::msg::TwistStamped
  getControlFromSequenceAsTwist(const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
  bool isHolonomic() const;

  /**
   * @brief Using control frequence and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
  void setOffset(double controller_frequency);

  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
  bool fallback(bool fail);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  geometry_msgs::msg::Pose goal_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ = {
    state_, generated_trajectories_, path_, goal_,
    costs_, settings_.model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
