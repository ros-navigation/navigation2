// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__SOURCE_HPP_
#define NAV2_COLLISION_MONITOR__SOURCE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Basic data source class
 */
class Source
{
public:
  /**
   * @brief Source constructor
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
  Source(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);
  /**
   * @brief Source destructor
   */
  virtual ~Source();

  /**
   * @brief Adds latest data from source to the data array.
   * Empty virtual method intended to be used in child implementations.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   * @return false if an invalid source should block the robot
   */
  virtual bool getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) = 0;

  /**
   * @brief Obtains source enabled state
   * @return Whether source is enabled
   */
  bool getEnabled() const;

  /**
   * @brief Obtains the name of the data source
   * @return Name of the data source
   */
  std::string getSourceName() const;

  /**
   * @brief Obtains the source_timeout parameter of the data source
   * @return source_timeout parameter value of the data source
   */
  rclcpp::Duration getSourceTimeout() const;

protected:
  /**
   * @brief Source configuration routine.
   * @return True in case of everything is configured correctly, or false otherwise
   */
  bool configure();

  /**
   * @brief Supporting routine obtaining ROS-parameters common for all data sources
   * @param source_topic Output name of source subscription topic
   */
  void getCommonParameters(std::string & source_topic);

  /**
   * @brief Checks whether the source data might be considered as valid
   * @param source_time Timestamp of latest obtained data
   * @param curr_time Current node time for source verification
   * @return True if data source is valid, otherwise false
   */
  bool sourceValid(
    const rclcpp::Time & source_time,
    const rclcpp::Time & curr_time) const;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Obtain the transform to get data from source frame and time where it was received to the
   * base frame and current time (if base_shift_correction_ is true) or the transform  without time
   * shift considered which is less accurate but much more faster option not dependent on state
   * estimation frames.
   * @param curr_time Current node time
   * @param data_header Current header  which contains the frame_id and the stamp
   * @param tf_transform Output source->base_frame_id_ transform
   * @return True if got correct transform, otherwise false
   */
  bool getTransform(
    const rclcpp::Time & curr_time,
    const std_msgs::msg::Header & data_header,
    tf2::Transform & tf_transform) const;

  // ----- Variables -----

  /// @brief Collision Monitor node
  nav2_util::LifecycleNode::WeakPtr node_;
  /// @brief Collision monitor node logger stored for further usage
  rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};
  /// @brief Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Basic parameters
  /// @brief Name of data source
  std::string source_name_;

  // Global variables
  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief Robot base frame ID
  std::string base_frame_id_;
  /// @brief Global frame ID for correct transform calculation
  std::string global_frame_id_;
  /// @brief Transform tolerance
  tf2::Duration transform_tolerance_;
  /// @brief Maximum time interval in which data is considered valid
  rclcpp::Duration source_timeout_;
  /// @brief Whether to correct source data towards to base frame movement,
  /// considering the difference between current time and latest source time
  bool base_shift_correction_;
  /// @brief Whether source is enabled
  bool enabled_;
};  // class Source

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__SOURCE_HPP_
