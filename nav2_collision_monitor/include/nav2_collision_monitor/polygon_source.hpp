// Copyright (c) 2023 Pixel Robotics GmbH
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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_SOURCE_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "geometry_msgs/msg/polygon_instance_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Implementation for polygon source
 */
class PolygonSource : public Source
{
public:
  /**
   * @brief PolygonSource constructor
   * @param node Collision Monitor node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement,
   * considering the difference between current time and latest source time
   */
  PolygonSource(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);
  /**
   * @brief PolygonSource destructor
   */
  ~PolygonSource();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from polygon source to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   * @return false if an invalid source should block the robot
   */
  bool getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data);

  /**
   * @brief Converts a PolygonInstanceStamped to a std::vector<Point>
   * @param polygon Input Polygon to be converted
   * @param data Output vector of Point
   */
  void convertPolygonStampedToPoints(
    const geometry_msgs::msg::PolygonStamped & polygon,
    std::vector<Point> & data) const;

protected:
  /**
   * @brief Getting sensor-specific ROS-parameters
   * @param source_topic Output name of source subscription topic
   */
  void getParameters(std::string & source_topic);

  /**
   * @brief PolygonSource data callback
   * @param msg Shared pointer to PolygonSource message
   */
  void dataCallback(geometry_msgs::msg::PolygonInstanceStamped::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief PolygonSource data subscriber
  rclcpp::Subscription<geometry_msgs::msg::PolygonInstanceStamped>::SharedPtr data_sub_;

  /// @brief Latest data obtained
  std::vector<geometry_msgs::msg::PolygonInstanceStamped> data_;

  /// @brief distance between sampled points on polygon edges
  double sampling_distance_;
};  // class PolygonSource

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_SOURCE_HPP_
