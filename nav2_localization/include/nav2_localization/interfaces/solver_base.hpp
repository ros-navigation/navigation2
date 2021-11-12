// Copyright (c) 2021 Jose M. TORRES-CAMARA and Khaled SAAD
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

#ifndef NAV2_LOCALIZATION__INTERFACES__SOLVER_BASE_HPP_
#define NAV2_LOCALIZATION__INTERFACES__SOLVER_BASE_HPP_

#include <memory>  // For shared_ptr<>

// Other Interfaces
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"

// Types
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// Others
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_localization
{

/**
 * @class Solver
 * @brief Abstract interface for solvers (such as Monte-Carlo Localization or Kalman Filter) to adhere to with pluginlib
 */
class Solver
{
public:
  Solver() {}

  using Ptr = std::shared_ptr<nav2_localization::Solver>;

  /**
     * @brief Estimates a pose fusing odometry and sensor information
     * @param curr_odom Current pose odometry-based estimation
     * @param scan Current measurement
     * @return Estimation of the current position
     */
  virtual geometry_msgs::msg::PoseWithCovarianceStamped estimatePose(
    const nav_msgs::msg::Odometry & curr_odom,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) = 0;

  /**
   * @brief Initializes the filter being used with a given pose
   * @param init_pose The pose at which to initialize the robot pose
   */
  virtual void initPose(const geometry_msgs::msg::PoseWithCovarianceStamped & init_pose)
  {
    prev_pose_= init_pose;
  }

  /**
   * @brief Initializes the filter being used with a given pose
   * @param init_odom The odometry at the initial estimated pose
   */
  void initOdom(const nav_msgs::msg::Odometry & init_odom)
  {
    prev_odom_ = init_odom;
  }

  /**
  * @brief Configures the solver, during the "Configuring" state of the parent lifecycle node.
  * @param node Pointer to the parent lifecycle node
  * @param motionSampler The Sample Motion Model to use
  * @param matcher The 2D Matcher to use
  */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    SampleMotionModel::Ptr & motionSampler,
    Matcher2d::Ptr & matcher)
  {
    node_ = node;
    motion_sampler_ = motionSampler;
    matcher_ = matcher;
  }

  /**
     * @brief Activates the solver, during the "Activating" state of the parent lifecycle node.
     */
  virtual void activate() = 0;

  /**
     * @brief Deactivates the solver, during the "Deactivating" state of the parent lifecycle node.
     */
  virtual void deactivate() = 0;

  /**
     * @brief Cleans up the solver, during the "Cleaning up" state of the parent lifecycle node.
     */
  virtual void cleanup() = 0;

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  SampleMotionModel::Ptr motion_sampler_;
  Matcher2d::Ptr matcher_;
  nav_msgs::msg::Odometry prev_odom_;  // Previous odometry
  geometry_msgs::msg::PoseWithCovarianceStamped prev_pose_;  // Previous pose estimation
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__INTERFACES__SOLVER_BASE_HPP_
