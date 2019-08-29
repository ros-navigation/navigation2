/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Samsung Research America
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE_GLOBAL_PLANNER_H_
#define NAV2_CORE_GLOBAL_PLANNER_H_

#include <string>
#include "nav2_msgs/msg/path.h"
#include "geometry_msgs/msg/pose_stamped.h"

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
  virtual void configure(const rclcpp::Node* parent,
    const std::string& name, tf2::Buffer * tf,
    nav2_costmap_2d::Costmap2DROS * costmap_ros) = 0;

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
   * @brief Method Execute recovery behavior
   * @param  name The name of this planner
   * @return true if successful, false is failed to execute fully
   */
  virtual bool executeRecovery(
    const nav2_msgs::RecoveryRequest & req) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE_GLOBAL_PLANNER_H_
