// Copyright (c) 2022 Adam Aposhian
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

#include <memory>

#include "nav2_util/path_utils.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_util
{

void append_transform_to_path(
  nav_msgs::msg::Path & path,
  tf2::Transform & transform)
{
  // Add a new empty pose
  path.poses.emplace_back();
  // Get the previous, last pose (after the emplace_back so the reference isn't invalidated)
  auto & previous_pose = *(path.poses.end() - 2);
  auto & new_pose = path.poses.back();
  geometry_msgs::msg::TransformStamped transform_msg(tf2::toMsg(
      tf2::Stamped<tf2::Transform>(
        transform, tf2::getTimestamp(previous_pose), tf2::getFrameId(previous_pose))));
  tf2::doTransform(
    previous_pose,
    new_pose,
    transform_msg
  );
}

void Straight::append(nav_msgs::msg::Path & path, double spacing) const
{
  auto num_points = std::floor(length_ / spacing);
  path.poses.reserve(path.poses.size() + num_points);
  tf2::Transform translation(tf2::Quaternion::getIdentity(), tf2::Vector3(spacing, 0.0, 0.0));
  for (size_t i = 1; i <= num_points; ++i) {
    append_transform_to_path(path, translation);
  }
}

void Arc::append(nav_msgs::msg::Path & path, double spacing) const
{
  double length = radius_ * std::abs(radians_);
  size_t num_points = std::floor(length / spacing);
  double radians_per_step = radians_ / num_points;
  tf2::Transform transform(
    tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), radians_per_step),
    tf2::Vector3(radius_ * cos(radians_per_step), radius_ * sin(radians_per_step), 0.0));
  path.poses.reserve(path.poses.size() + num_points);
  for (size_t i = 0; i < num_points; ++i) {
    append_transform_to_path(path, transform);
  }
}

nav_msgs::msg::Path generate_path(
  geometry_msgs::msg::PoseStamped start,
  double spacing,
  std::initializer_list<std::unique_ptr<PathSegment>> segments)
{
  nav_msgs::msg::Path path;
  path.poses.push_back(start);
  for (const auto & segment : segments) {
    segment->append(path, spacing);
  }
  return path;
}

}  // namespace nav2_util
