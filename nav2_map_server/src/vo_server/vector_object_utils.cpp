// Copyright (c) 2023 Samsung R&D Institute Russia
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

#include "nav2_map_server/vector_object_utils.hpp"

namespace nav2_map_server
{

bool lookupShapeTransform(
  const std_msgs::msg::Header & header,
  const std::string & target_frame,
  const nav2::TransformBuffer::SharedPtr & tf_buffer,
  const double transform_tolerance,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer->lookupTransform(
      target_frame, header.frame_id, rclcpp::Time(header.stamp),
      tf2::durationFromSec(transform_tolerance));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace nav2_map_server
