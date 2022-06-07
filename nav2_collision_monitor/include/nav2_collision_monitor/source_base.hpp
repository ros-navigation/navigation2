// Copyright (c) 2022 Samsung Research Russia
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

#ifndef NAV2_COLLISION_MONITOR__SOURCE_BASE_HPP_
#define NAV2_COLLISION_MONITOR__SOURCE_BASE_HPP_

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

class SourceBase
{
public:
  SourceBase(
    const nav2_util::LifecycleNode::WeakPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & source_name,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance,
    const tf2::Duration & data_timeout);
  virtual ~SourceBase();

  virtual void configure() = 0;

  virtual void getData(
    std::vector<Point> & data, const rclcpp::Time & curr_time) = 0;

  void setFixedFrameId(const std::string & fixed_frame_id);

protected:
  // @brief Supporting routine obtaining all ROS-parameters.
  // @param source_topic Output name of source subscription topic
  void getParameters(std::string & source_topic);

  bool sourceValid(const rclcpp::Time & curr_time);

  bool getSourceBaseTransform(
    const rclcpp::Time & curr_time,
    const rclcpp::Time & source_time,
    tf2::Transform & tf2_transform);

  // ----- Variables -----

  // Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  // Store collision monitor node logger for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Basic parameters
  std::string source_name_;
  std::string source_frame_id_;
  std::string base_frame_id_;
  std::string fixed_frame_id_;

  // Transform tolerance
  tf2::Duration transform_tolerance_;
  // Maximum allowed time shift between data and collision monitor node
  tf2::Duration data_timeout_;

  // Latest data timestamp
  rclcpp::Time data_stamp_;
};  // class SourceBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_BASE_HPP_
