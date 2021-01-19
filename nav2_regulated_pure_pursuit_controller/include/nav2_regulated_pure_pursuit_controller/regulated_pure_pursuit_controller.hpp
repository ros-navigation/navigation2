// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  RegulatedPurePursuitController() = default;

  ~RegulatedPurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;


  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  void rotateToHeading(double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &, const double &);

  bool inCollision(const double & x, const double & y);

  double costAtPose(const double & x, const double & y);

  void applyConstraints(
    double & linear_vel,
    const double & dist_error, const double & lookahead_dist,
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost);

  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};

  double desired_linear_vel_;
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  double max_linear_accel_;
  double max_linear_decel_;
  bool use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  bool use_approach_vel_scaling_;
  double min_approach_linear_velocity_;
  double control_duration_;
  double max_allowed_time_to_collision_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;

  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>> carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
