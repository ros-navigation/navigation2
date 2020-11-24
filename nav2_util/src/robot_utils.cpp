// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
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

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "rclcpp/logger.hpp"

namespace nav2_util
{

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame,
  const std::string robot_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("getCurrentPose");
  geometry_msgs::msg::PoseStamped robot_pose;

  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_frame;
  robot_pose.header.stamp = rclcpp::Time();

  try {
    global_pose = tf_buffer.transform(
      robot_pose, global_frame,
      tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger,
      "No Transform available Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger,
      "Connectivity Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger,
      "Extrapolation Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(
      logger,
      "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s",
      global_frame.c_str(), robot_frame.c_str());
  }

  return false;
}

}  // end namespace nav2_util
