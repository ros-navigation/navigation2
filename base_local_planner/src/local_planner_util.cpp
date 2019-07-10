/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <base_local_planner/local_planner_util.h>

#include <base_local_planner/goal_functions.h>

namespace base_local_planner {

void LocalPlannerUtil::initialize(
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2D* costmap,
    std::string global_frame) {
  if(!initialized_) {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    initialized_ = true;
  }
  else{
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) {
    config = default_limits_;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimits(config);
}

costmap_2d::Costmap2D* LocalPlannerUtil::getCostmap() {
  return costmap_;
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits() {
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return limits_;
}


bool LocalPlannerUtil::getGoal(geometry_msgs::PoseStamped& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        global_frame_,
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  //reset the global plan
  global_plan_.clear();

  global_plan_ = orig_global_plan;

  return true;
}

bool LocalPlannerUtil::getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *tf_,
      global_plan_,
      global_pose,
      *costmap_,
      global_frame_,
      transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}




} // namespace
