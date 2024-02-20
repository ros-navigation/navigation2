// Copyright (c) 2023 Smit Dumore
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

LoopbackSimulator::LoopbackSimulator(const rclcpp::NodeOptions & options)
: Node("loopback_simulator_node", options)
{
  // Initialize subscribers
  vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&LoopbackSimulator::twistCallback, this, std::placeholders::_1));

  init_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&LoopbackSimulator::initPoseCallback, this, std::placeholders::_1));

  // Initialize publishers
  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  // Frequency parameter for timer function
  double timer_frequency{0.0};
  declare_parameter("timer_frequency", 10.0);
  get_parameter("timer_frequency", timer_frequency);

  // Delta time parameter for forward integration
  declare_parameter("dt", 0.1);
  get_parameter("dt", dt_);

  // Create a timer
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_frequency),
    std::bind(&LoopbackSimulator::timerCallback, this));

  // Transforms
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LoopbackSimulator::~LoopbackSimulator()
{
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
}


void LoopbackSimulator::publishTransform(
  const geometry_msgs::msg::Pose & pose,
  const std::string & parent_frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(dt_);
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;

  tf_broadcaster_->sendTransform(transformStamped);
}

void LoopbackSimulator::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!init_pose_set_) {
    RCLCPP_WARN(
      rclcpp::get_logger("logger_name"),
      "Initial pose has not been set. Ignoring incoming velocity messages.");
    return;
  }

  // get base_link location in the odom frame
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  // Convert the quaternion in the transform to tf2::Quaternion
  tf2::Quaternion current_orientation;
  tf2::fromMsg(transformStamped.transform.rotation, current_orientation);

  // Calculate the change in orientation based on the twist message
  tf2::Quaternion angular_change;
  angular_change.setRPY(msg->angular.x * dt_, msg->angular.y * dt_, msg->angular.z * dt_);

  // Integrate the change in orientation with the current orientation
  tf2::Quaternion new_orientation = current_orientation * angular_change;

  // Calculate the change in position based on the twist message
  tf2::Matrix3x3 rotation_matrix(new_orientation);
  tf2::Vector3 linear_velocity(msg->linear.x, msg->linear.y, msg->linear.z);
  tf2::Vector3 rotated_linear_velocity = rotation_matrix * linear_velocity;

  // Integrate the change in position with the current position
  geometry_msgs::msg::Pose new_position;
  new_position.position.x =
    transformStamped.transform.translation.x + rotated_linear_velocity.x() * dt_;
  new_position.position.y =
    transformStamped.transform.translation.y + rotated_linear_velocity.y() * dt_;
  new_position.position.z =
    transformStamped.transform.translation.z + rotated_linear_velocity.z() * dt_;
  new_position.orientation = tf2::toMsg(new_orientation);

  // Publish the updated transform between odom and base_link
  publishTransform(new_position, "odom", "base_link");
}

void LoopbackSimulator::initPoseCallback(
  const
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose!");
  init_pose_ = *msg;
  init_pose_set_ = true;

  if (!zero_odom_base_published_) {
    // Publish first init pose between map and odom
    publishTransform(init_pose_.pose.pose, "map", "odom");
    geometry_msgs::msg::Pose zero_pose{};

    // Publish identity transform between odom and base_footprint
    publishTransform(zero_pose, "odom", "base_footprint");

    relocalization_pose_ = init_pose_.pose.pose;
    zero_odom_base_published_ = true;
    return;
  }

  // Updating map->base_link transform based on set pose
  geometry_msgs::msg::TransformStamped map_to_base_link_transform;
  map_to_base_link_transform.header = init_pose_.header;
  map_to_base_link_transform.header.frame_id = "map";
  map_to_base_link_transform.child_frame_id = "base_link";
  map_to_base_link_transform.transform.translation.x = init_pose_.pose.pose.position.x;
  map_to_base_link_transform.transform.translation.y = init_pose_.pose.pose.position.y;
  map_to_base_link_transform.transform.translation.z = init_pose_.pose.pose.position.z;
  map_to_base_link_transform.transform.rotation = init_pose_.pose.pose.orientation;

  geometry_msgs::msg::TransformStamped odom_to_base_link_transform;
  try {
    odom_to_base_link_transform = tf_buffer_->lookupTransform(
      "odom",
      "base_link",
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  // Calculating map->odom transform
  geometry_msgs::msg::TransformStamped map_to_odom_transform;
  tf2::doTransform(map_to_base_link_transform, map_to_odom_transform, odom_to_base_link_transform);

  // Updating relocalization pose of odom frame
  relocalization_pose_.position.x = map_to_odom_transform.transform.translation.x;
  relocalization_pose_.position.y = map_to_odom_transform.transform.translation.y;
  relocalization_pose_.position.z = map_to_odom_transform.transform.translation.z;
  relocalization_pose_.orientation = map_to_odom_transform.transform.rotation;

  // Publishing updated position of odom and base_link wrt map
  publishTransform(relocalization_pose_, "map", "odom");
  publishTransform(init_pose_.pose.pose, "map", "base_link");
}

void LoopbackSimulator::timerCallback()
{
  if (!init_pose_set_) {
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "Publishing map->odom tf");

  // Continously publishing map->odom transform
  publishTransform(relocalization_pose_, "map", "odom");

  // publish bogus lidar data
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

  scan_msg->header.frame_id = "base_link";
  scan_msg->header.stamp = this->now();

  // Fill in the scan data
  scan_msg->angle_min = -M_PI / 2.0;
  scan_msg->angle_max = M_PI / 2.0;
  scan_msg->angle_increment = M_PI / 180.0;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = 0.1;
  scan_msg->range_min = 0.0;
  scan_msg->range_max = 10.0;

  // Fill in the scan data
  int num_readings = 180;
  scan_msg->ranges.resize(num_readings);
  for (int i = 0; i < num_readings; ++i) {
    scan_msg->ranges[i] = 5.0;  // Set all ranges to 5 meters
  }

  scan_publisher_->publish(*scan_msg);
}

}  // namespace nav2_loopback_sim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_loopback_sim::LoopbackSimulator)
