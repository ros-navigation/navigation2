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

#ifndef NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
#define NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_

#include "nav2_collision_monitor/source_base.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_collision_monitor
{

class PointCloud : public SourceBase
{
public:
  PointCloud(
    const nav2_util::LifecycleNode::WeakPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & source_name,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance,
    const tf2::Duration & data_timeout);
  virtual ~PointCloud();

  virtual void configure();

  virtual void getData(std::vector<Point> & data, const rclcpp::Time & curr_time);

protected:
  // @brief Getting sensor-specific ROS-parameters
  // Implementation for Polygon class. Calls SourceBase::getParameters() inside.
  // @param source_topic Output name of source subscription topic
  void getParameters(std::string & source_topic);

  void dataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Minimum and maximum height of PointCloud projected to 2D space
  double min_height_, max_height_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr data_sub_;

  // Data obtained from source
  sensor_msgs::msg::PointCloud2::ConstSharedPtr data_;
};  // class PointCloud

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POINTCLOUD_HPP_
