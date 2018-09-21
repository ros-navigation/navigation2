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

#ifndef DWB_LOCAL_PLANNER_DEBUG_DWB_LOCAL_PLANNER_H
#define DWB_LOCAL_PLANNER_DEBUG_DWB_LOCAL_PLANNER_H

#include <dwb_local_planner/dwb_local_planner.h>
#include <dwb_msgs/DebugLocalPlan.h>
#include <dwb_msgs/GenerateTwists.h>
#include <dwb_msgs/GenerateTrajectory.h>
#include <dwb_msgs/ScoreTrajectory.h>
#include <dwb_msgs/GetCriticScore.h>
#include <string>

namespace dwb_local_planner
{

/**
 * @brief A version of DWBLocalPlanner with ROS services for the major components.
 *
 * Advertises three services: GenerateTwists, GenerateTrajectory and DebugLocalPlan
 */
class DebugDWBLocalPlanner: public DWBLocalPlanner
{
public:
  /**
   * @brief Override the DWB constructor to also advertise the services
   */
  void initialize(std::string name, TFListenerPtr tf, CostmapROSPtr costmap_ros) override;
protected:
  bool generateTwistsService(dwb_msgs::GenerateTwists::Request  &req,
                             dwb_msgs::GenerateTwists::Response &res);
  bool generateTrajectoryService(dwb_msgs::GenerateTrajectory::Request  &req,
                                 dwb_msgs::GenerateTrajectory::Response &res);
  bool scoreTrajectoryService(dwb_msgs::ScoreTrajectory::Request  &req,
                              dwb_msgs::ScoreTrajectory::Response &res);
  bool getCriticScoreService(dwb_msgs::GetCriticScore::Request  &req,
                             dwb_msgs::GetCriticScore::Response &res);
  bool debugLocalPlanService(dwb_msgs::DebugLocalPlan::Request  &req,
                             dwb_msgs::DebugLocalPlan::Response &res);

  TrajectoryCritic::Ptr getCritic(std::string name);

  ros::ServiceServer twist_gen_service_, generate_traj_service_, score_service_, critic_service_, debug_service_;
};

}  // namespace dwb_local_planner

#endif  // DWB_LOCAL_PLANNER_DEBUG_DWB_LOCAL_PLANNER_H
