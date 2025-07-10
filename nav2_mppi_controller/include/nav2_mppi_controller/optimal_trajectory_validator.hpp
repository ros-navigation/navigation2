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
#include "nav2_mppi_controller/models/optimizer_settings.hpp"

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
   * @param settings Settings for the MPPI optimizer
   */
  virtual void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    ParametersHandler * param_handler,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const models::OptimizerSettings & settings)
  {
    param_handler_ = param_handler;
    name_ = name;
    node_ = parent;
    costmap_ros_ = costmap;
    tf_buffer_ = tf_buffer;

    auto node = node_.lock();
    auto getParam = param_handler_->getParamGetter(name_);
    getParam(collision_lookahead_time_, "collision_lookahead_time", 2.0f);
    traj_samples_to_evaluate_ = collision_lookahead_time_ / settings.model_dt;
    if (traj_samples_to_evaluate_ > settings.time_steps) {
      traj_samples_to_evaluate_ = settings.time_steps;
      RCLCPP_WARN(
        node->get_logger(),
        "Collision lookahead time is greater than the number of trajectory samples, "
        "setting it to the maximum number of samples (%u).",
        traj_samples_to_evaluate_);
    }

    getParam(consider_footprint_, "consider_footprint", false);
    if (consider_footprint_) {
      collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
            nav2_costmap_2d::Costmap2D *>>(costmap_ros_->getCostmap());
    }
  }

  /**
   * @brief Validate the optimal trajectory from MPPI optimization
   * Could be used to check for collisions, progress towards goal,
   * distance from path, min distance from obstacles, dynamic feasibility, etc.
   * @param optimal_trajectory The optimal trajectory to validate
   * @param control_sequence The control sequence to validate
   * @param robot_pose The current pose of the robot
   * @param robot_speed The current speed of the robot
   * @param plan The planned path for the robot
   * @param goal The goal pose for the robot
   * @return True if the trajectory is valid, false otherwise
   */
  virtual ValidationResult validateTrajectory(
    const Eigen::ArrayXXf & optimal_trajectory,
    const models::ControlSequence & /*control_sequence*/,
    const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
    const geometry_msgs::msg::Twist & /*robot_speed*/,
    const nav_msgs::msg::Path & /*plan*/,
    const geometry_msgs::msg::Pose & /*goal*/)
  {
    // The Optimizer automatically ensures that we are within Kinematic
    // and dynamic constraints, no need to check for those again.

    // Check for collisions. This is highly unlikely to occur since the Obstacle/Cost Critics
    // penalize collisions severely, but it is still possible if those critics are not used or the
    // optimized trajectory is very near obstacles and the dynamic constraints cause invalidity.
    auto costmap = costmap_ros_->getCostmap();
    for (size_t i = 0; i < traj_samples_to_evaluate_; ++i) {
      const double x = static_cast<double>(optimal_trajectory(i, 0));
      const double y = static_cast<double>(optimal_trajectory(i, 1));

      if (consider_footprint_) {
        const double theta = static_cast<double>(optimal_trajectory(i, 2));
        double footprint_cost = collision_checker_->footprintCostAtPose(
          x, y, theta, costmap_ros_->getRobotFootprint());
        if (footprint_cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
          return ValidationResult::SOFT_RESET;
        }
      } else {
        unsigned int x_i = 0u, y_i = 0u;
        if (!costmap->worldToMap(x, y, x_i, y_i)) {
          continue;  // Out of bounds, skip this point
        }
        unsigned char cost = costmap->getCost(x_i, y_i);
        if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
          cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          return ValidationResult::SOFT_RESET;
        }
      }
    }

    return ValidationResult::SUCCESS;
  }

protected:
  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  ParametersHandler * param_handler_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  float collision_lookahead_time_{1.0f};
  unsigned int traj_samples_to_evaluate_{0u};
  bool consider_footprint_{false};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMAL_TRAJECTORY_VALIDATOR_HPP_
