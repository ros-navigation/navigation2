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
#include <memory>


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
    plugin_name + ".image_topic",
    rclcpp::ParameterValue("/camera/color/image_raw"));
  node->declare_parameter(plugin_name + ".save_dir", rclcpp::ParameterValue("/home/username/"));
  node->declare_parameter(plugin_name + ".image_format", rclcpp::ParameterValue(".png"));

  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".image_topic", image_topic_);
  node->get_parameter(plugin_name + ".save_dir", directory_to_save_images_);
  node->get_parameter(plugin_name + ".image_format", image_format_);

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing photo at waypoint plugin, subscribing to camera topic named; %s",
      image_topic_.c_str());
    camera_image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SystemDefaultsQoS(),
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
    std::experimental::filesystem::path save_dir = directory_to_save_images_;
    std::experimental::filesystem::path file_name = std::to_string(
      curr_waypoint_index) + "_" +
      std::to_string(curr_pose.header.stamp.sec) + image_format_;

    std::experimental::filesystem::path full_path_image_path = save_dir / file_name;


    global_mutex_.lock();
    cv::Mat curr_frame_mat;
    auto curr_frame_msg_as_shared_ptr = std::make_shared<sensor_msgs::msg::Image>(curr_frame_msg_);
    deepCopyMsg2Mat(curr_frame_msg_as_shared_ptr, curr_frame_mat);
    cv::imwrite(full_path_image_path.c_str(), curr_frame_mat);
    global_mutex_.unlock();
    RCLCPP_INFO(
      logger_,
      "Photo has been taken sucessfully at waypoint %i", curr_waypoint_index);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Couldn't take photo at waypoint %i! Caught exception: %s",
      curr_waypoint_index, e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(
      logger_,
      "An unknown execption caught while taking photo at waypoint %i!"
      "Make sure that the image topic named: %s is valid and active!",
      curr_waypoint_index,
      image_topic_.c_str());
    return false;
  }
  return true;
}

void PhotoAtWaypoint::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  global_mutex_.lock();
  curr_frame_msg_ = *msg;
  global_mutex_.unlock();
}

int PhotoAtWaypoint::encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported mat type");
  }
}

std::string PhotoAtWaypoint::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void PhotoAtWaypoint::deepCopyMsg2Mat(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  cv::Mat & mat)
{
  cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  cv::Mat frame = cv_bridge_ptr->image;
  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  frame.copyTo(mat);
}

}      // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::PhotoAtWaypoint,
  nav2_core::WaypointTaskExecutor)
