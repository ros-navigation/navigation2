/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
#ifndef NAV_CORE2_GLOBAL_PLANNER_H
#define NAV_CORE2_GLOBAL_PLANNER_H

#include <nav_core2/common.h>
#include <nav_2d_msgs/Path2D.h>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <string>

namespace nav_core2
{

/**
 * @class GlobalPlanner
 * @brief Provides an interface for global planners used in navigation.
 */
class GlobalPlanner
{
public:
  /**
   * @brief Virtual Destructor
   */
  virtual ~GlobalPlanner() {}

  /**
   * @brief  Initialization function for the GlobalPlanner
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void initialize(std::string name, CostmapROSPtr costmap_ros) = 0;

  /**
   * @brief Run the global planner to make a plan starting at the start and ending at the goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual nav_2d_msgs::Path2D makePlan(const nav_2d_msgs::Pose2DStamped& start,
                                       const nav_2d_msgs::Pose2DStamped& goal) = 0;
};
}  // namespace nav_core2

#endif  // NAV_CORE2_GLOBAL_PLANNER_H
