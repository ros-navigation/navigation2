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
#ifndef DWB_CRITICS__PATH_ALIGN_HPP_
#define DWB_CRITICS__PATH_ALIGN_HPP_

#include <vector>
#include <string>
#include "dwb_critics/path_dist.hpp"

namespace dwb_critics
{
/**
 * @class PathAlignCritic
 * @brief Scores trajectories based on how far from the global path the front of the robot ends up.
 *
 * This uses the costmap grid as a proxy for calculating which way the robot should be facing relative
 * to the global path. Instead of scoring how far the center of the robot is away from the global path,
 * this critic calculates how far a point forward_point_distance in front of the robot is from the global
 * path. This biases the planner toward trajectories that line up with the global plan.
 *
 * When the robot is near the end of the path, the scale of this critic is set to zero. When the projected
 * point is past the global goal, we no longer want this critic to try to align to a part of the global path
 * that isn't there.
 */
class PathAlignCritic : public PathDistCritic
{
public:
  PathAlignCritic()
  : zero_scale_(false), forward_point_distance_(0.0) {}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double getScale() const override;
  double scorePose(const geometry_msgs::msg::Pose2D & pose) override;

protected:
  bool zero_scale_;
  double forward_point_distance_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PATH_ALIGN_HPP_
