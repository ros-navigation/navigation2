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

#include <dwb_local_planner/debug_dwb_local_planner.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <string>

namespace dwb_local_planner
{

void DebugDWBLocalPlanner::initialize(std::string name, TFListenerPtr tf, CostmapROSPtr costmap_ros)
{
  DWBLocalPlanner::initialize(name, tf, costmap_ros);
  ros::NodeHandle private_nh("~/" + name);

  debug_service_ = private_nh.advertiseService("debug_local_plan",
                                               &DebugDWBLocalPlanner::debugLocalPlanService, this);
  twist_gen_service_ = private_nh.advertiseService("generate_twists",
                                                   &DebugDWBLocalPlanner::generateTwistsService, this);
  score_service_ = private_nh.advertiseService("score_trajectory",
                                               &DebugDWBLocalPlanner::scoreTrajectoryService, this);
  critic_service_ = private_nh.advertiseService("get_critic_score",
                                                &DebugDWBLocalPlanner::getCriticScoreService, this);
  generate_traj_service_ = private_nh.advertiseService("generate_traj",
                                                       &DebugDWBLocalPlanner::generateTrajectoryService, this);
}

bool DebugDWBLocalPlanner::generateTwistsService(dwb_msgs::GenerateTwists::Request  &req,
                                                 dwb_msgs::GenerateTwists::Response &res)
{
  res.twists = traj_generator_->getTwists(req.current_vel);
  return true;
}

bool DebugDWBLocalPlanner::generateTrajectoryService(dwb_msgs::GenerateTrajectory::Request  &req,
                                                     dwb_msgs::GenerateTrajectory::Response &res)
{
  res.traj = traj_generator_->generateTrajectory(req.start_pose, req.start_vel, req.cmd_vel);
  return true;
}

bool DebugDWBLocalPlanner::scoreTrajectoryService(dwb_msgs::ScoreTrajectory::Request  &req,
                                                  dwb_msgs::ScoreTrajectory::Response &res)
{
  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);

  nav_2d_msgs::Path2D transformed_plan;
  nav_2d_msgs::Pose2DStamped goal_pose;
  prepareGlobalPlan(req.pose, transformed_plan, goal_pose);

  for (TrajectoryCritic::Ptr critic : critics_)
  {
    if (critic->prepare(req.pose.pose, req.velocity, goal_pose.pose, transformed_plan) == false)
    {
      ROS_WARN_NAMED("DebugDWBLocalPlanner", "A scoring function failed to prepare");
    }
  }
  res.score = scoreTrajectory(req.traj);
  return true;
}

TrajectoryCritic::Ptr DebugDWBLocalPlanner::getCritic(std::string name)
{
  for (TrajectoryCritic::Ptr critic : critics_)
  {
    if (critic->getName() == name)
      return critic;
  }
  return nullptr;
}

bool DebugDWBLocalPlanner::getCriticScoreService(dwb_msgs::GetCriticScore::Request  &req,
                                                 dwb_msgs::GetCriticScore::Response &res)
{
  TrajectoryCritic::Ptr critic = getCritic(req.critic_name);
  if (critic == nullptr)
  {
    ROS_WARN_NAMED("DebugDWBLocalPlanner", "Critic %s not found!", req.critic_name.c_str());
    return false;
  }

  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);

  nav_2d_msgs::Path2D transformed_plan;
  nav_2d_msgs::Pose2DStamped goal_pose;
  prepareGlobalPlan(req.pose, transformed_plan, goal_pose);

  critic->prepare(req.pose.pose, req.velocity, goal_pose.pose, transformed_plan);

  res.score.raw_score = critic->scoreTrajectory(req.traj);
  res.score.scale = critic->getScale();
  res.score.name = req.critic_name;

  return true;
}

bool DebugDWBLocalPlanner::debugLocalPlanService(dwb_msgs::DebugLocalPlan::Request &req,
                                                 dwb_msgs::DebugLocalPlan::Response &res)
{
  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);
  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results = std::make_shared<dwb_msgs::LocalPlanEvaluation>();
  try
  {
    computeVelocityCommands(req.pose, req.velocity, results);
    res.results = *results;
    return true;
  }
  catch (const nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("DebugDWBLocalPlanner", "Exception in computeVelocityCommands: %s", e.what());
    return false;
  }
}

}  // namespace dwb_local_planner


//  register this planner as a LocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwb_local_planner::DebugDWBLocalPlanner, nav_core2::LocalPlanner)
