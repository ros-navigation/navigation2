// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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

#ifndef OPENNAV_DOCKING__CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "opennav_docking/graceful_controller.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::Controller
 * @brief Default controller based on GracefulController
 * plugin interface.
 */
class Controller : public GracefulController
{
public:
  /**
   * @brief Create a controller instance. Configure ROS 2 parameters.
   *
   * @param node Lifecycle node
   * @param tf tf2_ros TF buffer
   * @param fixed_frame Fixed frame
   * @param base_frame Robot base frame
   */
  Controller(
    const nav2::LifecycleNode::SharedPtr & node, nav2::TransformBuffer::SharedPtr tf,
    std::string fixed_frame, std::string base_frame);

  /**
   * @brief A destructor for opennav_docking::Controller
   */
  ~Controller();

  /**
   * @brief Compute a velocity command using control law.
   * @param pose Target pose, in robot centric coordinates.
   * @param cmd Command velocity.
   * @param is_docking If true, robot is docking. If false, robot is undocking.
   * @param backward If true, robot will drive backwards to goal.
   * @returns True if command is valid, false otherwise.
   */
  bool computeVelocityCommand(
    const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool is_docking,
    bool backward = false);
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
