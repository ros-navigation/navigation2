/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2022, Floatic, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "nav2_fake_localization/fake_localization.hpp"

namespace nav2_fake_localization {
FakeOdomNode::FakeOdomNode()
    : Node("fake_localization", rclcpp::NodeOptions()) {

  typedef std::chrono::duration<int> seconds_type;
  seconds_type buffer_timeout(1);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_pos_received_ = false;
  this->declare_parameter("global_frame_id", "map");
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "base_footprint");
  this->declare_parameter("delta_x", 0.0);
  this->declare_parameter("delta_y", 0.0);
  this->declare_parameter("delta_yaw", 0.0);
  this->declare_parameter("transform_tolerance", 0.1);

  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  delta_x_ = this->get_parameter("delta_x").as_double();
  delta_y_ = this->get_parameter("delta_y").as_double();
  delta_yaw_ = this->get_parameter("delta_yaw").as_double();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -delta_yaw_);
  tf_offset_ = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));

  odom_sub_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
  odom_filter_ =
      std::make_shared<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>(
          odom_sub_, *tf_buffer_, base_frame_id_, 100,
          this->get_node_logging_interface(), this->get_node_clock_interface(),
          buffer_timeout);
  odom_filter_->registerCallback(&FakeOdomNode::sub_odom, this);

  initialpose_sub_.subscribe(this, "initialpose");
  initialpose_filter_ = std::make_shared<
      tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>>(
      initialpose_sub_, *tf_buffer_, global_frame_id_, 1,
      this->get_node_logging_interface(), this->get_node_clock_interface(),
      buffer_timeout);
  initialpose_filter_->registerCallback(&FakeOdomNode::sub_initialpose, this);
}

FakeOdomNode::~FakeOdomNode() {}

void FakeOdomNode::sub_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Transform txi;
  tf2::convert(msg->pose.pose, txi);
  txi = tf_offset_ * txi;

  geometry_msgs::msg::TransformStamped odom_to_map;
  try {
    geometry_msgs::msg::TransformStamped txi_inv;
    txi_inv.header.frame_id = base_frame_id_;
    txi_inv.header.stamp = msg->header.stamp;
    tf2::convert(txi.inverse(), txi_inv.transform);

    tf_buffer_->transform(txi_inv, odom_to_map, odom_frame_id_);
  } catch (tf2::TransformException &e) {
    RCLCPP_ERROR(get_logger(), "Failed to transform to %s from %s: %s\n",
                 odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
    return;
  }

  geometry_msgs::msg::TransformStamped trans;

  trans.header.stamp = msg->header.stamp;
  trans.header.stamp.nanosec += 1e8;
  trans.header.frame_id = global_frame_id_;
  trans.child_frame_id = msg->header.frame_id;
  tf2::Transform odom_to_map_tf2;
  tf2::convert(odom_to_map.transform, odom_to_map_tf2);
  tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
  tf2::convert(odom_to_map_inv, trans.transform);
  tf_broadcaster_->sendTransform(trans);
}

void FakeOdomNode::sub_initialpose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  tf2::Transform pose;
  tf2::convert(msg->pose.pose, pose);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(get_logger(),
                "Frame ID of \"initialpose\" (%s) is different from the global "
                "frame %s",
                msg->header.frame_id.c_str(), global_frame_id_.c_str());
  }

  // set offset so that current pose is set to "initialpose"
  geometry_msgs::msg::TransformStamped base_in_map;
  try {
    // just get the latest
    base_in_map = tf_buffer_->lookupTransform(base_frame_id_, global_frame_id_,
                                              rclcpp::Time(0));
  } catch (tf2::TransformException &e) {
    RCLCPP_WARN(get_logger(), "Failed to lookup transform! %s", e.what());
    return;
  }

  tf2::Transform base_in_map2;
  tf2::convert(base_in_map.transform, base_in_map2);
  tf2::Transform delta = pose * base_in_map2;
  tf_offset_ = delta * tf_offset_;
}

} // namespace nav2_fake_localization

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_fake_localization::FakeOdomNode>();
  rclcpp::Rate loop(20);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
