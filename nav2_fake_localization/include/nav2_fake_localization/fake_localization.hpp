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
/** \author Ioan Sucan - ros1 */

/**
 * Ported by Jihoon
 * Date: 09.2022
 **/

#ifndef NAV2_FAKE_LOCALIZATION_FAKE_LOCALIZATION_HPP_
#define NAV2_FAKE_LOCALIZATION_FAKE_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <angles/angles.h>
#include <memory>

#include <chrono>

namespace nav2_fake_localization {

class FakeOdomNode : public rclcpp::Node {
public:
  FakeOdomNode();
  ~FakeOdomNode();

protected:
  void sub_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void sub_initialpose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
  // subscriber
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>> odom_filter_;

  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>
      initialpose_sub_;
  std::shared_ptr<
      tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>>
      initialpose_filter_;

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string global_frame_id_, odom_frame_id_, base_frame_id_;
  double delta_x_, delta_y_, delta_yaw_;
  bool base_pos_received_;
  double tf_tolerance_;
  tf2::Transform tf_offset_;
};
} // namespace nav2_fake_localization

#endif
