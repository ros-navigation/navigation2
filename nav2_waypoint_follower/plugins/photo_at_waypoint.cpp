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

  curr_frame_msg_ = std::make_shared<sensor_msgs::msg::Image>();

  node->declare_parameter(plugin_name + ".enabled", rclcpp::ParameterValue(true));
  node->declare_parameter(
    plugin_name + ".image_topic",
    rclcpp::ParameterValue("/camera/color/image_raw"));
  node->declare_parameter(
    plugin_name + ".save_dir",
    rclcpp::ParameterValue("/tmp/waypoint_images"));
  node->declare_parameter(plugin_name + ".image_format", rclcpp::ParameterValue("png"));

  std::string save_dir_as_string;
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".image_topic", image_topic_);
  node->get_parameter(plugin_name + ".save_dir", save_dir_as_string);
  node->get_parameter(plugin_name + ".image_format", image_format_);

  // get inputted save directory and make sure it exists, if not log and create  it
  save_dir_ = save_dir_as_string;
  try {
    if (!std::experimental::filesystem::exists(save_dir_)) {
      RCLCPP_WARN(
        logger_,
        "Provided save directory for photo at waypoint plugin does not exist,"
        "provided directory is: %s, the directory will be created automatically.",
        save_dir_.c_str()
      );
      if (!std::experimental::filesystem::create_directory(save_dir_)) {
        RCLCPP_ERROR(
          logger_,
          "Failed to create directory!: %s required by photo at waypoint plugin, "
          "exiting the plugin with failure!",
          save_dir_.c_str()
        );
        is_enabled_ = false;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Exception (%s) thrown while attempting to create image capture directory."
      " This task executor is being disabled as it cannot save images.", e.what());
    is_enabled_ = false;
  }

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Photo at waypoint plugin is disabled.");
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
    // construct the full path to image filename
    std::experimental::filesystem::path file_name = std::to_string(
      curr_waypoint_index) + "_" +
      std::to_string(curr_pose.header.stamp.sec) + "." + image_format_;
    std::experimental::filesystem::path full_path_image_path = save_dir_ / file_name;

    // save the taken photo at this waypoint to given directory
    std::lock_guard<std::mutex> guard(global_mutex_);
    cv::Mat curr_frame_mat;
    deepCopyMsg2Mat(curr_frame_msg_, curr_frame_mat);
    cv::imwrite(full_path_image_path.c_str(), curr_frame_mat);
    RCLCPP_INFO(
      logger_,
      "Photo has been taken sucessfully at waypoint %i", curr_waypoint_index);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Couldn't take photo at waypoint %i! Caught exception: %s \n"
      "Make sure that the image topic named: %s is valid and active!",
      curr_waypoint_index,
      e.what(), image_topic_.c_str());
    return false;
  }
  return true;
}

void PhotoAtWaypoint::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(global_mutex_);
  curr_frame_msg_ = msg;
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
