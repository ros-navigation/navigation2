// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "tf2/utils.h"

namespace opennav_docking
{

Controller::Controller(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf,
  std::string fixed_frame, std::string base_frame)
: tf2_buffer_(tf), fixed_frame_(fixed_frame), base_frame_(base_frame)
{
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.k_phi", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.k_delta", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.beta", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.lambda", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_linear_min", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_linear_max", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.v_angular_max", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.slowdown_radius", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic",
    rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue(
      std::string("local_costmap/published_footprint")));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  node->get_parameter("controller.k_phi", k_phi_);
  node->get_parameter("controller.k_delta", k_delta_);
  node->get_parameter("controller.beta", beta_);
  node->get_parameter("controller.lambda", lambda_);
  node->get_parameter("controller.v_linear_min", v_linear_min_);
  node->get_parameter("controller.v_linear_max", v_linear_max_);
  node->get_parameter("controller.v_angular_max", v_angular_max_);
  node->get_parameter("controller.slowdown_radius", slowdown_radius_);
  control_law_ = std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, v_linear_min_, v_linear_max_,
    v_angular_max_);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&Controller::dynamicParametersCallback, this, std::placeholders::_1));

  node->get_parameter("controller.use_collision_detection", use_collision_detection_);
  node->get_parameter("controller.projection_time", projection_time_);
  node->get_parameter("controller.simulation_time_step", simulation_time_step_);
  node->get_parameter("controller.transform_tolerance", transform_tolerance_);

  if (use_collision_detection_) {
    std::string costmap_topic, footprint_topic;
    node->get_parameter("controller.costmap_topic", costmap_topic);
    node->get_parameter("controller.footprint_topic", footprint_topic);
    node->get_parameter("controller.dock_collision_threshold", dock_collision_threshold_);
    configureCollisionChecker(node, costmap_topic, footprint_topic, transform_tolerance_);
  }

  trajectory_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("docking_trajectory", 1);
}

Controller::~Controller()
{
  control_law_.reset();
  trajectory_pub_.reset();
  collision_checker_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
}

bool Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool is_docking,
  bool backward)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  cmd = control_law_->calculateRegularVelocity(pose, backward);
  return isTrajectoryCollisionFree(pose, is_docking, backward);
}

bool Controller::isTrajectoryCollisionFree(
  const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward)
{
  // Visualization of the trajectory
  nav_msgs::msg::Path trajectory;
  trajectory.header.frame_id = base_frame_;
  trajectory.header.stamp = clock_->now();

  // First pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = base_frame_;
  trajectory.poses.push_back(next_pose);

  // Get the transform from base_frame to fixed_frame
  geometry_msgs::msg::TransformStamped base_to_fixed_transform;
  try {
    base_to_fixed_transform = tf2_buffer_->lookupTransform(
      fixed_frame_, base_frame_, trajectory.header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_, "Could not get transform from %s to %s: %s",
      base_frame_.c_str(), fixed_frame_.c_str(), ex.what());
    return false;
  }

  // Generate path
  double distance = std::numeric_limits<double>::max();
  unsigned int max_iter = static_cast<unsigned int>(ceil(projection_time_ / simulation_time_step_));

  do{
    // Apply velocities to calculate next pose
    next_pose.pose = control_law_->calculateNextPose(
      simulation_time_step_, target_pose, next_pose.pose, backward);

    // Add the pose to the trajectory for visualization
    trajectory.poses.push_back(next_pose);

    // Transform pose from base_frame into fixed_frame
    geometry_msgs::msg::PoseStamped local_pose = next_pose;
    local_pose.header.stamp = trajectory.header.stamp;
    tf2::doTransform(local_pose, local_pose, base_to_fixed_transform);

    // Determine the distance at which to check for collisions
    // Skip the final segment of the trajectory for docking
    // and the initial segment for undocking
    // This avoids false positives when the robot is at the dock
    double dock_collision_distance = is_docking ?
      nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose) :
      std::hypot(next_pose.pose.position.x, next_pose.pose.position.y);

    // If this distance is greater than the dock_collision_threshold, check for collisions
    if (use_collision_detection_ &&
      dock_collision_distance > dock_collision_threshold_ &&
      !collision_checker_->isCollisionFree(nav_2d_utils::poseToPose2D(local_pose.pose)))
    {
      RCLCPP_WARN(
        logger_, "Collision detected at pose: (%.2f, %.2f, %.2f) in frame %s",
        local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z,
        local_pose.header.frame_id.c_str());
      trajectory_pub_->publish(trajectory);
      return false;
    }

    // Check if we reach the goal
    distance = nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose);
  }while(distance > 1e-2 && trajectory.poses.size() < max_iter);

  trajectory_pub_->publish(trajectory);

  return true;
}

void Controller::configureCollisionChecker(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string costmap_topic, std::string footprint_topic, double transform_tolerance)
{
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node, footprint_topic, *tf2_buffer_, base_frame_, transform_tolerance);
  collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, node->get_name());
}

rcl_interfaces::msg::SetParametersResult
Controller::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller.k_phi") {
        k_phi_ = parameter.as_double();
      } else if (name == "controller.k_delta") {
        k_delta_ = parameter.as_double();
      } else if (name == "controller.beta") {
        beta_ = parameter.as_double();
      } else if (name == "controller.lambda") {
        lambda_ = parameter.as_double();
      } else if (name == "controller.v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == "controller.v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == "controller.v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == "controller.slowdown_radius") {
        slowdown_radius_ = parameter.as_double();
      } else if (name == "controller.projection_time") {
        projection_time_ = parameter.as_double();
      } else if (name == "controller.simulation_time_step") {
        simulation_time_step_ = parameter.as_double();
      } else if (name == "controller.dock_collision_threshold") {
        dock_collision_threshold_ = parameter.as_double();
      }

      // Update the smooth control law with the new params
      control_law_->setCurvatureConstants(k_phi_, k_delta_, beta_, lambda_);
      control_law_->setSlowdownRadius(slowdown_radius_);
      control_law_->setSpeedLimit(v_linear_min_, v_linear_max_, v_angular_max_);
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_docking
