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

/**
 * @brief Basic data source class
 */
class SourceBase
{
public:
  /**
   * @brief SourceBase constructor
   */
  SourceBase(
    const nav2_util::LifecycleNode::WeakPtr & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & source_name,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance,
    const tf2::Duration & data_timeout);
  /**
   * @brief SourceBase destructor
   */
  virtual ~SourceBase();

  /**
   * @brief Data source configuration routine.
   * Empty virtual method intended to be used in child implementations.
   */
  virtual void configure() = 0;

  /**
   * @brief Sets fixed frame ID. Used in data frame interpolation
   * @param fixed_frame_id Fixed frame ID
   */
  void setFixedFrameId(const std::string & fixed_frame_id);

  /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is converted to base_frame_id coordinate system at curr_time.
   */
  virtual void getData(
    const rclcpp::Time & curr_time, std::vector<Point> & data) = 0;

protected:
  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param source_topic Output name of source subscription topic
   */
  void getParameters(std::string & source_topic);

  /**
   * @brief Checks whether the source might be considered as valid
   * @param curr_time Current node time for source verification
   * @return True if data source is valid, otherwise false
   */
  bool sourceValid(const rclcpp::Time & curr_time);

  /**
   * @brief Obtains a transform from source_frame_id at source_time ->
   * to base_frame_id at current node time
   * @param curr_time Current node timestamp to interpolate data to
   * @param source_time Source timestamp to convert data from
   * @param tf2_transform Output source->base transform
   * @return True if got correct transform, otherwise false
   */
  bool getSourceBaseTransform(
    const rclcpp::Time & curr_time,
    const rclcpp::Time & source_time,
    tf2::Transform & tf2_transform);

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Basic parameters
  /// @brief Name of data source
  std::string source_name_;
  /// @brief Source frame ID
  std::string source_frame_id_;
  /// @brief Base frame ID
  std::string base_frame_id_;
  /// @brief Fixed frame ID. Used to get source->base time inerpolated transform.
  std::string fixed_frame_id_;

  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Maximum allowed time shift between data and collision monitor node
  tf2::Duration data_timeout_;

  /// @brief Latest data timestamp
  rclcpp::Time data_stamp_;
};  // class SourceBase

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_BASE_HPP_
