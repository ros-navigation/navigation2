// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
#define OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_

#include <string>
#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"


namespace opennav_docking_core
{

/**
 * @enum DockDirection
 * @brief An enum class representing the direction of the dock
 */
enum class DockDirection { UNKNOWN, FORWARD, BACKWARD };

/**
 * @class ChargingDock
 * @brief Abstract interface for a charging dock for the docking framework
 */
class ChargingDock
{
public:
  using Ptr = std::shared_ptr<ChargingDock>;

  /**
   * @brief Virtual destructor
   */
  virtual ~ChargingDock() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) = 0;

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) = 0;

  /**
   * @brief Have we made contact with dock? This can be implemented in a variety
   * of ways: by establishing communications with the dock, by monitoring the
   * the drive motor efforts, etc.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool isDocked() = 0;

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly after the docking loop to check if successful.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool isCharging() = 0;

  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool disableCharging() = 0;

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging() = 0;

  /**
   * @brief Gets if this is a charging-typed dock
   */
  virtual bool isCharger() {return true;}

  /**
   * @brief Indicates the direction of the dock. This is used to determine if the
   * robot should drive forwards or backwards onto the dock.
   * @return DockDirection The direction of the dock
   */
  DockDirection getDockDirection() {return dock_direction_;}

  /**
   * @brief Determines whether the robot should rotate 180º to face away from the dock.
   * For example, to perform a backward docking without detections.
   * @return bool If the robot should rotate to face away from the dock.
   */
  bool shouldRotateToDock() {return rotate_to_dock_;}

  std::string getName() {return name_;}

protected:
  std::string name_;
  DockDirection dock_direction_{DockDirection::UNKNOWN};
  bool rotate_to_dock_{false};
};

}  // namespace opennav_docking_core

#endif  // OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
