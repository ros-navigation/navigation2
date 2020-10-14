// Copyright (c) 2020 Fetullah Atas
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

#include "nav2_waypoint_follower/plugins/photo_at_waypoint.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <string>


namespace nav2_waypoint_follower
{
PhotoAtWaypoint::PhotoAtWaypoint()
{
}

PhotoAtWaypoint::~PhotoAtWaypoint()
{
}

void PhotoAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();

  node->declare_parameter(plugin_name + ".enabled", rclcpp::ParameterValue(true));
  node->declare_parameter(
    plugin_name + ".camera_image_topic_name",
    rclcpp::ParameterValue("/fake_realsense_camera/image_raw"));
  node->declare_parameter(plugin_name + ".save_images_dir", rclcpp::ParameterValue("/home/atas/"));
  node->declare_parameter(plugin_name + ".image_format", rclcpp::ParameterValue(".png"));

  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".camera_image_topic_name", camera_image_topic_name_);
  node->get_parameter(plugin_name + ".save_images_dir", directory_to_save_images_);
  node->get_parameter(plugin_name + ".image_format", image_format_);

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      rmw_qos_profile_default.history,
      rmw_qos_profile_default.depth
  ));
  qos.reliability(rmw_qos_profile_default.reliability);

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing photo at waypoint plugin, subscribing to camera topic named; %s",
      camera_image_topic_name_.c_str());
    camera_image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      camera_image_topic_name_, qos,
      std::bind(&PhotoAtWaypoint::imageCallback, this, std::placeholders::_1));
  }
}

bool PhotoAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    RCLCPP_WARN(
      logger_,
      "Photo at waypoint plugin is disabled. Not performing anything"
    );
    return true;
  }
  try {
    global_mutex_.lock();
    std::string stamped_name_for_curr_frame = directory_to_save_images_ + std::to_string(
      curr_waypoint_index) + "_" +
      std::to_string(curr_pose.header.stamp.sec) + image_format_;
    cv::imwrite(stamped_name_for_curr_frame, curr_frame_);
    RCLCPP_INFO(
      logger_,
      "Photo has been taken sucessfully at waypoint %i", curr_waypoint_index);
    global_mutex_.unlock();
  } catch (...) {
    RCLCPP_ERROR(
      logger_,
      "Photo at waypoint plugin caught an exception while trying taking a photo at waypoint %i.",
      curr_waypoint_index);
    RCLCPP_ERROR(
      logger_,
      "Not going to take a photo!, Make sure that the image topic named: %s is valid and active!",
      camera_image_topic_name_.c_str());
    return false;
  }
  return true;
}

void PhotoAtWaypoint::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  cv::Mat frame = cv_bridge_ptr->image;
  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  global_mutex_.lock();
  frame.copyTo(curr_frame_);
  global_mutex_.unlock();
}
}      // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::PhotoAtWaypoint,
  nav2_core::WaypointTaskExecutor)
