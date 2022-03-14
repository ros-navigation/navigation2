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

#include "path_utils.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace path_utils
{

void append_transform_to_path(
  nav_msgs::msg::Path & path,
  tf2::Transform & relative_transform)
{
  // Add a new empty pose
  path.poses.emplace_back();
  // Get the previous, last pose (after the emplace_back so the reference isn't invalidated)
  auto & previous_pose = *(path.poses.end() - 2);
  auto & new_pose = path.poses.back();

  // get map_transform of previous_pose
  tf2::Transform map_transform;
  tf2::fromMsg(previous_pose.pose, map_transform);

  tf2::Transform full_transform;
  full_transform.mult(map_transform, relative_transform);

  tf2::toMsg(full_transform, new_pose.pose);
  new_pose.header.frame_id = previous_pose.header.frame_id;
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

double chord_length(double radius, double radians)
{
  return 2 * radius * sin(radians / 2);
}

void Arc::append(nav_msgs::msg::Path & path, double spacing) const
{
  double length = radius_ * std::abs(radians_);
  size_t num_points = std::floor(length / spacing);
  double radians_per_step = radians_ / num_points;
  tf2::Transform transform(
    tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), radians_per_step),
    tf2::Vector3(chord_length(radius_, std::abs(radians_per_step)), 0.0, 0.0));
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
  path.header = start.header;
  path.poses.push_back(start);
  for (const auto & segment : segments) {
    segment->append(path, spacing);
  }
  return path;
}

}  // namespace path_utils
