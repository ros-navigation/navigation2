// Copyright (c) 2022 Samsung Research America
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

#ifndef OPENNAV_FOLLOWING__PARAMETER_HANDLER_HPP_
#define OPENNAV_FOLLOWING__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_util/parameter_handler.hpp"
#include "nav2_ros_common/node_utils.hpp"

namespace opennav_following
{

struct Parameters
{
  // Frequency to run control loops
  double controller_frequency;
  // Timeout to detect the object
  double detection_timeout;
  // Timeout to detect the object while rotating to it
  double rotate_to_object_timeout;
  // Timeout after which a static object is considered as goal reached
  double static_object_timeout;
  // Tolerances for arriving at the safe_distance pose
  double linear_tolerance, angular_tolerance;
  // Maximum number of times the robot will retry to approach the object
  int max_retries;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame;
  // Desired distance to keep from the object
  double desired_distance;
  // Skip perception orientation
  bool skip_orientation;
  // Should the robot search for the object by rotating or go to last known heading
  bool search_by_rotating;
  // Search angle relative to current robot orientation when rotating to find objects
  double search_angle;
  // Tolerance for transforming coordinates
  double transform_tolerance;
  // Parameters for OdomSmoother
  std::string odom_topic;
  double odom_duration;
  bool use_collision_detection;
  double filter_coef;
};

/**
 * @class opennav_following::ParameterHandler
 * @brief Handles parameters and dynamic parameters for Planner Server
 */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters>
{
public:
  /**
   * @brief Constructor for opennav_following::ParameterHandler
   */
  ParameterHandler(
    const nav2::LifecycleNode::SharedPtr & node,
    const rclcpp::Logger & logger);

protected:
  /**
   * @brief Validate incoming parameter updates before applying them.
   * This callback is triggered when one or more parameters are about to be updated.
   * It checks the validity of parameter values and rejects updates that would lead
   * to invalid or inconsistent configurations
   * @param parameters List of parameters that are being updated.
   * @return rcl_interfaces::msg::SetParametersResult Result indicating whether the update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Apply parameter updates after validation
   * This callback is executed when parameters have been successfully updated.
   * It updates the internal configuration of the node with the new parameter values.
   * @param parameters List of parameters that have been updated.
   */
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters) override;

  const std::vector<std::string> default_ids_{"GridBased"};
  const std::vector<std::string> default_types_{"nav2_navfn_planner::NavfnPlanner"};
};

}  // namespace opennav_following

#endif  // OPENNAV_FOLLOWING__PARAMETER_HANDLER_HPP_
