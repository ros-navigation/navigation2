// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
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
// limitations under the License. Reserved.

#ifndef NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_
#define NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_

#include <memory>  // For shared_ptr<>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{
/**
 * @class SampleMotionModel
 * @brief Abstract interface for sample motion model to adhere to with pluginlib
 */
class SampleMotionModel
{
public:
  using Ptr = std::shared_ptr<nav2_localization::SampleMotionModel>;

  /**
   * @brief Calculates the most likely pose that the robot is now in, following its motion.
   * @param prev_odom The robot's odometry at the previous time step (i.e. before the robot has moved).
   * @param curr_odom The robot's odometry at the current time step (i.e. after the robot has moved).
   * @param prev_pose The robot's pose estimation at the previous time step.
   * @return The most likely pose of the robot at the current time step, based on the model's estimation.
   */
  virtual geometry_msgs::msg::TransformStamped getMostLikelyPose(
    const geometry_msgs::msg::TransformStamped & prev_odom,
    const geometry_msgs::msg::TransformStamped & curr_odom,
    const geometry_msgs::msg::TransformStamped & prev_pose) = 0;

  /**
   * @brief Configures the model, during the "Configuring" state of the parent lifecycle node.
   * @param node Pointer to the parent lifecycle node.
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) = 0;

  /**
   * @brief Activates the model, during the "Activating" state of the parent lifecycle node.
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivates the model, during the "Decativating" state of the parent lifecycle node.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Cleans up the model, during the "Cleaningup" state of the parent lifecycle node.
   */
  virtual void cleanup() = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__INTERFACES__SAMPLE_MOTION_MODEL_BASE_HPP_
