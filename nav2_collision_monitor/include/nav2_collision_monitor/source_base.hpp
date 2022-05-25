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

#include "tf2_ros/buffer.h"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

enum SourceType
{
  SOURCE_BASE = 0,
  SCAN = 1,
  PCL_2 = 2
};

class SourceBase
{
public:
SourceBase();
SourceBase(
  nav2_util::LifecycleNode * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_topic,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift);
virtual ~SourceBase();

SourceType getSourceType();
std::string getSourceTypeStr();

void getData(std::vector<Point> & data, const rclcpp::Time & curr_time, const Velocity & velocity);

protected:
bool getTransform(
  const std::string to_frame,
  const std::string from_frame,
  tf2::Transform & tf2_transform);

void fixData(const rclcpp::Time & curr_time, const Velocity & velocity);

protected:
// Collision Monitor node
nav2_util::LifecycleNode * node_;

// TF buffer
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

// Basic parameters
SourceType source_type_;
std::string source_topic_;
std::string source_frame_id_;
std::string base_frame_id_;

// Data obtained from source in base_frame_id_
Data data_;
mutex_t data_mutex_;
Data data_fixed_;

// Transform tolerance
double transform_tolerance_;
// Maximum allowed time shift between source and collision monitor node clocks
double max_time_shift_;

};  // class SourceBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_BASE_HPP_
