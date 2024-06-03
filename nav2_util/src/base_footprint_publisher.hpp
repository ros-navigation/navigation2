// Copyright (c) 2023 Open Navigation LLC
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

#ifndef BASE_FOOTPRINT_PUBLISHER_HPP_
#define BASE_FOOTPRINT_PUBLISHER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace nav2_util
{

/**
 * @brief A TF2 listener that overrides the subscription callback
 * to inject base footprint publisher removing Z, Pitch, and Roll for
 * 3D state estimation but desiring a 2D frame for navigation, visualization, or other reasons
 */
class BaseFootprintPublisherListener : public tf2_ros::TransformListener
{
public:
  BaseFootprintPublisherListener(tf2::BufferCore & buffer, bool spin_thread, rclcpp::Node & node)
  : tf2_ros::TransformListener(buffer, spin_thread)
  {
    node.declare_parameter(
      "base_link_frame", rclcpp::ParameterValue(std::string("base_link")));
    node.declare_parameter(
      "base_footprint_frame", rclcpp::ParameterValue(std::string("base_footprint")));
    base_link_frame_ = node.get_parameter("base_link_frame").as_string();
    base_footprint_frame_ = node.get_parameter("base_footprint_frame").as_string();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  }

  /**
   * @brief Overrides TF2 subscription callback to inject base footprint publisher
   */
  void subscription_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static) override
  {
    TransformListener::subscription_callback(msg, is_static);

    if (is_static) {
      return;
    }

    for (unsigned int i = 0; i != msg->transforms.size(); i++) {
      auto & t = msg->transforms[i];
      if (t.child_frame_id == base_link_frame_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = t.header.stamp;
        transform.header.frame_id = base_link_frame_;
        transform.child_frame_id = base_footprint_frame_;

        // Project to Z-zero
        transform.transform.translation = t.transform.translation;
        transform.transform.translation.z = 0.0;

        // Remove Roll and Pitch
        tf2::Quaternion q;
        q.setRPY(0, 0, tf2::getYaw(t.transform.rotation));
        q.normalize();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
        return;
      }
    }
  }

protected:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string base_link_frame_, base_footprint_frame_;
};

/**
 * @class nav2_util::BaseFootprintPublisher
 * @brief Republishes the ``base_link`` frame as ``base_footprint``
 * stripping away the Z, Roll, and Pitch of the full 3D state to provide
 * a 2D projection for navigation when state estimation is full 3D
 */
class BaseFootprintPublisher : public rclcpp::Node
{
public:
  /**
   * @brief A constructor
   */
  explicit BaseFootprintPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("base_footprint_publisher", options)
  {
    RCLCPP_INFO(get_logger(), "Creating base footprint publisher");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    listener_publisher_ = std::make_shared<BaseFootprintPublisherListener>(
      *tf_buffer_, true, *this);
  }

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<BaseFootprintPublisherListener> listener_publisher_;
};

}  // end namespace nav2_util

#endif  // BASE_FOOTPRINT_PUBLISHER_HPP_
