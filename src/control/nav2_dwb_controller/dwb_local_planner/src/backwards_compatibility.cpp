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
#include <dwb_local_planner/backwards_compatibility.h>
#include <nav_2d_utils/parameters.h>
#include <string>
#include <vector>

namespace dwb_local_planner
{

using nav_2d_utils::moveParameter;

std::string getBackwardsCompatibleDefaultGenerator(const ros::NodeHandle& nh)
{
  bool use_dwa;
  nh.param("use_dwa", use_dwa, true);
  if (use_dwa)
  {
    return "dwb_plugins::LimitedAccelGenerator";
  }
  else
  {
    return "dwb_plugins::StandardTrajectoryGenerator";
  }
}

void loadBackwardsCompatibleParameters(const ros::NodeHandle& nh)
{
  std::vector<std::string> critic_names;
  ROS_INFO_NAMED("DWBLocalPlanner", "No critics configured! Using the default set.");
  critic_names.push_back("RotateToGoal");       // discards trajectories that move forward when already at goal
  critic_names.push_back("Oscillation");        // discards oscillating motions (assisgns cost -1)
  critic_names.push_back("ObstacleFootprint");  // discards trajectories that move into obstacles
  critic_names.push_back("GoalAlign");          // prefers trajectories that make the nose go towards (local) nose goal
  critic_names.push_back("PathAlign");          // prefers trajectories that keep the robot nose on nose path
  critic_names.push_back("PathDist");           // prefers trajectories on global path
  critic_names.push_back("GoalDist");           // prefers trajectories that go towards (local) goal,
                                                //         based on wave propagation
  nh.setParam("critics", critic_names);
  moveParameter(nh, "path_distance_bias", "PathAlign/scale", 32.0, false);
  moveParameter(nh, "goal_distance_bias", "GoalAlign/scale", 24.0, false);
  moveParameter(nh, "path_distance_bias", "PathDist/scale", 32.0);
  moveParameter(nh, "goal_distance_bias", "GoalDist/scale", 24.0);
  moveParameter(nh, "occdist_scale",      "ObstacleFootprint/scale", 0.01);

  moveParameter(nh, "max_scaling_factor", "ObstacleFootprint/max_scaling_factor", 0.2);
  moveParameter(nh, "scaling_speed",      "ObstacleFootprint/scaling_speed", 0.25);
}

}  // namespace dwb_local_planner
