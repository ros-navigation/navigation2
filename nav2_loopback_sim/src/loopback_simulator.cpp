// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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
#include "nav2_loopback_sim/loopback_simulator.hpp"

namespace nav2_loopback_sim
{

LoopbackSimulator::LoopbackSimulator()
: Node("loopback_simulator_node")
{
  // Initialise subscribers
  vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));

  init_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&LoopbackSimulator::initposeCallback, this, std::placeholders::_1));

  // Frequency parameter for timer function
  double timer_frequency{0.0};
  declare_parameter("timer_frequency", 10.0);
  get_parameter("timer_frequency", timer_frequency);

  // Create a timer
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_frequency),
    std::bind(&LoopbackSimulator::timerCallback, this));

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  //  tf
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (init_pose_set_ == false) {return;}

  double dt = 0.1;  // Adjust the time step

  //  Transform initpose from map to odom frame
  if (transform_initpose_once_ == true) {
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "odom",
        init_pose_.header.frame_id,
        tf2::TimePointZero);

      tf2::doTransform(init_pose_, odom_updated_pose_, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        rclcpp::get_logger("logger_name"),
        "Could not transform init pose from Map to Odom.");
    }

    transform_initpose_once_ = false;
  }

  // Update the transformed pose in the local variable
  odom_updated_pose_.header.frame_id = "odom";
  odom_updated_pose_.header.stamp = this->now();

  tf2::Quaternion base_link_orientation(
    odom_updated_pose_.pose.pose.orientation.x,
    odom_updated_pose_.pose.pose.orientation.y,
    odom_updated_pose_.pose.pose.orientation.z,
    odom_updated_pose_.pose.pose.orientation.w
  );

  tf2::Quaternion angular_change;
  angular_change.setRPY(msg->angular.x * dt, msg->angular.y * dt, msg->angular.z * dt);
  base_link_orientation *= angular_change;

  odom_updated_pose_.pose.pose.orientation.x = base_link_orientation.x();
  odom_updated_pose_.pose.pose.orientation.y = base_link_orientation.y();
  odom_updated_pose_.pose.pose.orientation.z = base_link_orientation.z();
  odom_updated_pose_.pose.pose.orientation.w = base_link_orientation.w();

  // Forward integrate robot position and orientation using velocity commands
  tf2::Matrix3x3 rotation_matrix(base_link_orientation);
  tf2::Vector3 linear_velocity(msg->linear.x, msg->linear.y, msg->linear.z);
  tf2::Vector3 rotated_linear_velocity = rotation_matrix * linear_velocity;

  odom_updated_pose_.pose.pose.position.x += rotated_linear_velocity.x() * dt;
  odom_updated_pose_.pose.pose.position.y += rotated_linear_velocity.y() * dt;
  odom_updated_pose_.pose.pose.position.z += rotated_linear_velocity.z() * dt;

  // Broadcast "odom" to "base_link" transform using the local variable
  geometry_msgs::msg::TransformStamped odom_to_base_transform;
  odom_to_base_transform.header.stamp = odom_updated_pose_.header.stamp;
  odom_to_base_transform.header.frame_id = "odom";
  odom_to_base_transform.child_frame_id = "base_link";
  odom_to_base_transform.transform.translation.x = odom_updated_pose_.pose.pose.position.x;
  odom_to_base_transform.transform.translation.y = odom_updated_pose_.pose.pose.position.y;
  odom_to_base_transform.transform.translation.z = odom_updated_pose_.pose.pose.position.z;
  odom_to_base_transform.transform.rotation = odom_updated_pose_.pose.pose.orientation;

  tf_broadcaster_->sendTransform(odom_to_base_transform);
}

void LoopbackSimulator::initposeCallback(
  const
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose wrt map");
  init_pose_ = *msg;
  init_pose_set_ = true;

  if (init_odom_base_published_ == false) {
    // Publish Identity transform between odom to base_footprint
    geometry_msgs::msg::TransformStamped odom_to_base_transform;
    odom_to_base_transform.header.stamp = this->now();
    odom_to_base_transform.header.frame_id = "odom";
    odom_to_base_transform.child_frame_id = "base_footprint";
    odom_to_base_transform.transform.translation.x = 0.0;
    odom_to_base_transform.transform.translation.y = 0.0;
    odom_to_base_transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    odom_to_base_transform.transform.rotation.x = q.x();
    odom_to_base_transform.transform.rotation.y = q.y();
    odom_to_base_transform.transform.rotation.z = q.z();
    odom_to_base_transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(odom_to_base_transform);
    init_odom_base_published_ = true;
  }
}

void LoopbackSimulator::timerCallback()
{
  if (init_pose_set_ == false) {return;}
  // RCLCPP_INFO(this->get_logger(), "Publishing map->odom tf");

  // Publish map to odom transform
  geometry_msgs::msg::TransformStamped map_to_odom_transform;
  map_to_odom_transform.header.stamp = this->now();
  map_to_odom_transform.header.frame_id = "map";
  map_to_odom_transform.child_frame_id = "odom";
  map_to_odom_transform.transform.translation.x = init_pose_.pose.pose.position.x;
  map_to_odom_transform.transform.translation.y = init_pose_.pose.pose.position.y;
  map_to_odom_transform.transform.translation.z = init_pose_.pose.pose.position.z;
  map_to_odom_transform.transform.rotation = init_pose_.pose.pose.orientation;

  tf_broadcaster_->sendTransform(map_to_odom_transform);
}

}  // namespace nav2_loopback_sim
