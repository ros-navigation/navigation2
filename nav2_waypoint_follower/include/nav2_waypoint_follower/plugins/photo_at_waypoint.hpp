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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_

/**
 * While C++17 isn't the project standard. We have to force LLVM/CLang
 * to ignore deprecated declarations
 */
#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM


#include <filesystem>
#include <mutex>
#include <string>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"


namespace nav2_waypoint_follower
{

class PhotoAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  /**
  * @brief Construct a new Photo At Waypoint object
  *
  */
  PhotoAtWaypoint();

  /**
   * @brief Destroy the Photo At Waypoint object
   *
   */
  ~PhotoAtWaypoint();

  /**
   * @brief declares and loads parameters used
   *
   * @param parent parent node that plugin will be created within
   * @param plugin_name should be provided in nav2_params.yaml==> waypoint_follower
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

  /**
   * @brief
   *
   * @param msg
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief given a shared pointer to sensor::msg::Image type, make a deep copy to inputted cv Mat
   *
   * @param msg
   * @param mat
   */
  static void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & mat);

protected:
  // to ensure safety when accessing global var curr_frame_
  std::mutex global_mutex_;
  // the taken photos will be saved under this directory
  std::filesystem::path save_dir_;
  // .png ? .jpg ? or some other well known format
  std::string image_format_;
  // the topic to subscribe in order capture a frame
  std::string image_topic_;
  // whether plugin is enabled
  bool is_enabled_;
  // current frame;
  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_;
  // global logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  // ros subscriber to get camera image
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscriber_;
};
}  // namespace nav2_waypoint_follower

#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__PHOTO_AT_WAYPOINT_HPP_
