// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE_RECOVERY_H_
#define NAV2_CORE_RECOVERY_H_

#include <string>
#include "rclcpp/rclcpp.h"
#include "tf2_ros/Buffer.h"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for recoveries to adhere to with pluginlib
 */
class Recovery
{
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~Recovery() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(const rclcpp::LifecycleNode * parent,
    const std::string & name, tf2_ros::Buffer * tf) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active recovery and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive recovery and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to shutdown recovery and any threads involved in execution.
   */
  virtual void shutdown() = 0;

  /**
   * @brief Method Execute recovery behavior
   * @param  name The name of this planner
   * @return true if successful, false is failed to execute fully
   */
  virtual bool executeRecovery() = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE_RECOVERY_H_
