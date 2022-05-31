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

#ifndef NAV2_COLLISION_MONITOR__SCAN_HPP_
#define NAV2_COLLISION_MONITOR__SCAN_HPP_

#include "nav2_collision_monitor/source_base.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_collision_monitor
{

class Scan : public SourceBase
{
public:
Scan(
  const nav2_util::LifecycleNode::WeakPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string source_name,
  const std::string base_frame_id,
  const double transform_tolerance,
  const double max_time_shift);
virtual ~Scan();

virtual bool init();

protected:
void dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr data_sub_;

};  // class Scan

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SCAN_HPP_
