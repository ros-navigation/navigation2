// Copyright (c) 2019 Intel Corporation
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
#include <vector>
#include <memory>

#include "nav2_costmap_2d/footprint_subscriber.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.hpp"
#pragma GCC diagnostic pop

namespace nav2_costmap_2d
{

bool
FootprintSubscriber::getFootprintRaw(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  if (!footprint_received_) {
    return false;
  }

  auto current_footprint = std::atomic_load(&footprint_);
  footprint = toPointVector(current_footprint->polygon);
  footprint_header = current_footprint->header;

  return true;
}

bool
FootprintSubscriber::getFootprintInRobotFrame(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  if (!getFootprintRaw(footprint, footprint_header)) {
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, tf_, footprint_header.frame_id, robot_base_frame_,
      transform_tolerance_, footprint_header.stamp))
  {
    return false;
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  std::vector<geometry_msgs::msg::Point> temp;
  transformFootprint(-x, -y, 0, footprint, temp);
  transformFootprint(0, 0, -theta, temp, footprint);

  footprint_header.frame_id = robot_base_frame_;
  footprint_header.stamp = current_pose.header.stamp;

  return true;
}

void
FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace nav2_costmap_2d
