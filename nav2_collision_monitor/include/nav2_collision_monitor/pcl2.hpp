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

#ifndef NAV2_COLLISION_MONITOR__PCL2_HPP_
#define NAV2_COLLISION_MONITOR__PCL2_HPP_

#include "nav2_collision_monitor/source_base.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_collision_monitor
{

class PCL2 : public SourceBase
{
public:
PCL2(
  nav2_util::LifecycleNode * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_topic,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift,
  const double min_z, const double max_z);
virtual ~PCL2();

private:
// Minimum and maximum height of PCL projected to 2D space
double min_z_, max_z_;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr data_sub_;
void dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

};  // class PCL2

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__PCL2_HPP_
