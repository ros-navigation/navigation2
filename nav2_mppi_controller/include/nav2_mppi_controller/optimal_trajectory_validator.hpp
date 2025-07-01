// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_

#include <Eigen/Core>
#include <string>
#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"

namespace mppi
{

enum class ValidationResult
{
  SUCCESS,
  SOFT_RESET,
  FAILURE,
};

/**
 * @class mppi::OptimalTrajectoryValidator
 * @brief Abstract base class for validating optimal trajectories from MPPI optimization
 */
class OptimalTrajectoryValidator
{
public:
  using Ptr = std::shared_ptr<OptimalTrajectoryValidator>;

  /**
   * @brief Constructor for mppi::OptimalTrajectoryValidator
   */
  OptimalTrajectoryValidator() = default;

  /**
   * @brief Destructor for mppi::OptimalTrajectoryValidator
   */
  virtual ~OptimalTrajectoryValidator() = default;

  /**
   * @brief Initialize the trajectory validator
   * @param node Weak pointer to the lifecycle node
   * @param name Name of the validator plugin
   * @param costmap Shared pointer to the costmap ROS wrapper
   * @param param_handler Pointer to the parameters handler
   * @param tf_buffer Shared pointer to the TF buffer
   */
  virtual void initialize(
    const nav2::LifecycleNode::WeakPtr & node,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    ParametersHandler * param_handler,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  {
    param_handler_ = param_handler;
    name_ = name;
    node_ = node;
    costmap_ = costmap;
    tf_buffer_ = tf_buffer;
  }

  /**
   * @brief Validate the optimal control sequence from MPPI optimization
   * @param optimal_control The optimal control sequence to validate
   * @param robot_pose The current pose of the robot
   * @param robot_speed The current speed of the robot
   * @param plan The planned path for the robot
   * @param goal The goal pose for the robot
   * @return True if the trajectory is valid, false otherwise
   */
  virtual ValidationResult validateTrajectory(
    const Eigen::ArrayXXf & /*optimal_control*/,
    const geometry_msgs::msg::PoseStamped /*robot_pose*/,
    const geometry_msgs::msg::Twist /*robot_speed*/,
    const nav_msgs::msg::Path & /*plan*/,
    const geometry_msgs::msg::Pose & /*goal*/)
  {
    return ValidationResult::SUCCESS;
  }

protected:
  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  ParametersHandler * param_handler_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_
