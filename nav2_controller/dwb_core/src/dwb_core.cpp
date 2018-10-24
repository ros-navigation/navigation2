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

#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include <utility>
#include "dwb_core/dwb_core.h"
#include "dwb_core/illegal_trajectory_tracker.h"
#include "nav_2d_utils/conversions.h"
#include "nav_2d_utils/parameters.h"
#include "nav_2d_utils/tf_help.h"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "dwb_core/exceptions.h"

namespace dwb_core
{

void loadBackwardsCompatibleParameters(const std::shared_ptr<rclcpp::Node> & nh)
{
  std::vector<std::string> critic_names;
  RCLCPP_INFO(nh->get_logger(),
    "DWBLocalPlanner", "No critics configured! Using the default set.");
  critic_names.push_back("RotateToGoal");       // discards trajectories that move forward when
                                                //   already at goal
  critic_names.push_back("Oscillation");        // discards oscillating motions (assisgns cost -1)
  critic_names.push_back("ObstacleFootprint");  // discards trajectories that move into obstacles
  critic_names.push_back("GoalAlign");          // prefers trajectories that make the
                                                //   nose go towards (local) nose goal
  critic_names.push_back("PathAlign");          // prefers trajectories that keep the
                                                //   robot nose on nose path
  critic_names.push_back("PathDist");           // prefers trajectories on global path
  critic_names.push_back("GoalDist");           // prefers trajectories that go towards
                                                //   (local) goal, based on wave propagation
  nh->set_parameters({rclcpp::Parameter("critics", critic_names)});
  /* *INDENT-OFF* */
  nav_2d_utils::moveParameter(nh, "path_distance_bias", "PathAlign/scale", 32.0, false);
  nav_2d_utils::moveParameter(nh, "goal_distance_bias", "GoalAlign/scale", 24.0, false);
  nav_2d_utils::moveParameter(nh, "path_distance_bias", "PathDist/scale", 32.0);
  nav_2d_utils::moveParameter(nh, "goal_distance_bias", "GoalDist/scale", 24.0);
  nav_2d_utils::moveParameter(nh, "occdist_scale",      "ObstacleFootprint/scale", 0.01);

  nav_2d_utils::moveParameter(nh, "max_scaling_factor", "ObstacleFootprint/max_scaling_factor", 0.2);  // NOLINT
  nav_2d_utils::moveParameter(nh, "scaling_speed",      "ObstacleFootprint/scaling_speed", 0.25);
  /* *INDENT-ON* */
}

DWBLocalPlanner::DWBLocalPlanner()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  goal_checker_loader_("dwb_core", "dwb_core::GoalChecker"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void DWBLocalPlanner::initialize(
  std::shared_ptr<rclcpp::Node> & private_nh, TFBufferPtr tf,
  CostmapROSPtr costmap_ros)
{
  nh_ = private_nh;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  nh_->get_parameter_or("prune_plan", prune_plan_, true);
  nh_->get_parameter_or("prune_distance", prune_distance_, 1.0);
  pub_.initialize(nh_);

  // Plugins
  std::string traj_generator_name;
  nh_->get_parameter_or("trajectory_generator_name", traj_generator_name,
    std::string("dwb_plugins::StandardTrajectoryGenerator"));
  traj_generator_ = std::move(traj_gen_loader_.createUniqueInstance(traj_generator_name));
  traj_generator_->initialize(nh_);

  std::string goal_checker_name;
  nh_->get_parameter_or("goal_checker_name", goal_checker_name,
    std::string("dwb_plugins::SimpleGoalChecker"));
  goal_checker_ = std::move(goal_checker_loader_.createUniqueInstance(goal_checker_name));
  goal_checker_->initialize(nh_);

  loadCritics();
}

std::string DWBLocalPlanner::resolveCriticClassName(std::string base_name)
{
  if (base_name.find("Critic") == std::string::npos) {
    base_name = base_name + "Critic";
  }

  if (base_name.find("::") == std::string::npos) {
    for (unsigned int j = 0; j < default_critic_namespaces_.size(); j++) {
      std::string full_name = default_critic_namespaces_[j] + "::" + base_name;
      if (critic_loader_.isClassAvailable(full_name)) {
        return full_name;
      }
    }
  }
  return base_name;
}

void DWBLocalPlanner::loadCritics()
{
  nh_->get_parameter("default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.size() == 0) {
    default_critic_namespaces_.push_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!nh_->get_parameter("critics", critic_names)) {
    loadBackwardsCompatibleParameters(nh_);
  }

  nh_->get_parameter("critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string plugin_name = critic_names[i];
    std::string plugin_class;
    nh_->get_parameter_or(plugin_name + "/class", plugin_class, plugin_name);
    plugin_class = resolveCriticClassName(plugin_class);

    TrajectoryCritic::Ptr plugin = std::move(critic_loader_.createUniqueInstance(plugin_class));
    RCLCPP_INFO(nh_->get_logger(),
      "Using critic \"%s\" (%s)", plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    plugin->initialize(nh_, costmap_ros_);
  }
}

bool DWBLocalPlanner::isGoalReached(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity)
{
  if (global_plan_.poses.size() == 0) {
    RCLCPP_WARN(rclcpp::get_logger(
        "DWBLocalPlanner"), "Cannot check if the goal is reached without the goal being set!");
    return false;
  }

  nav_2d_msgs::msg::Pose2DStamped local_start_pose, goal_pose, local_goal_pose;
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), pose, local_start_pose);
  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), goal_pose, local_goal_pose);

  return goal_checker_->isGoalReached(local_start_pose.pose, local_goal_pose.pose, velocity);
}

void DWBLocalPlanner::setPlan(const nav_2d_msgs::msg::Path2D & path)
{
  for (TrajectoryCritic::Ptr critic : critics_) {
    critic->reset();
  }

  pub_.publishGlobalPlan(path);
  global_plan_ = path;
}

nav_2d_msgs::msg::Twist2DStamped DWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity)
{
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr;
  if (pub_.shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }

  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel = computeVelocityCommands(pose, velocity, results);
    pub_.publishEvaluation(results);
    return cmd_vel;
  } catch (const nav_core2::PlannerException & e) {
    pub_.publishEvaluation(results);
    throw;
  }
}

void DWBLocalPlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_.publishTransformedPlan(transformed_plan);
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), goal_pose, goal_pose);
}

nav_2d_msgs::msg::Twist2DStamped DWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = nh_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan = transformGlobalPlan(pose);
  nav_2d_msgs::msg::Pose2DStamped goal_pose;

  prepareGlobalPlan(pose, transformed_plan, goal_pose);

  for (TrajectoryCritic::Ptr critic : critics_) {
    if (critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan) == false) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = nh_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    pub_.publishLocalPlan(pose.header, best.traj);
    pub_.publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const nav_core2::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr critic : critics_) {
      critic->debrief(empty_cmd);
    }
    pub_.publishLocalPlan(pose.header, empty_traj);
    pub_.publishCostGrid(costmap_ros_, critics_);

    throw;
  }
}

dwb_msgs::msg::TrajectoryScore DWBLocalPlanner::coreScoringAlgorithm(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  nav_2d_msgs::msg::Twist2D twist;
  dwb_msgs::msg::Trajectory2D traj;
  dwb_msgs::msg::TrajectoryScore best, worst;
  best.total = -1;
  worst.total = -1;
  IllegalTrajectoryTracker tracker;

  traj_generator_->startNewIteration(velocity);
  while (traj_generator_->hasMoreTwists()) {
    twist = traj_generator_->nextTwist();
    traj = traj_generator_->generateTrajectory(pose, velocity, twist);

    try {
      dwb_msgs::msg::TrajectoryScore score = scoreTrajectory(traj, best.total);
      tracker.addLegalTrajectory();
      if (results) {
        results->twists.push_back(score);
      }
      if (best.total < 0 || score.total < best.total) {
        best = score;
        if (results) {
          results->best_index = results->twists.size() - 1;
        }
      }
      if (worst.total < 0 || score.total > worst.total) {
        worst = score;
        if (results) {
          results->worst_index = results->twists.size() - 1;
        }
      }
    } catch (const nav_core2::IllegalTrajectoryException & e) {
      if (results) {
        dwb_msgs::msg::TrajectoryScore failed_score;
        failed_score.traj = traj;

        dwb_msgs::msg::CriticScore cs;
        cs.name = e.getCriticName();
        cs.raw_score = -1.0;
        failed_score.scores.push_back(cs);
        failed_score.total = -1.0;
        results->twists.push_back(failed_score);
      }
      tracker.addIllegalTrajectory(e);
    }
  }

  if (best.total < 0) {
    if (debug_trajectory_details_) {
      RCLCPP_ERROR(rclcpp::get_logger("DWBLocalPlanner"), "%s", tracker.getMessage().c_str());
      for (auto const & x : tracker.getPercentages()) {
        RCLCPP_ERROR(rclcpp::get_logger(
            "DWBLocalPlanner"), "%.2f: %10s/%s", x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::msg::TrajectoryScore DWBLocalPlanner::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj,
  double best_score)
{
  dwb_msgs::msg::TrajectoryScore score;
  score.traj = traj;

  for (TrajectoryCritic::Ptr critic : critics_) {
    dwb_msgs::msg::CriticScore cs;
    cs.name = critic->getName();
    cs.scale = critic->getScale();

    if (cs.scale == 0.0) {
      score.scores.push_back(cs);
      continue;
    }

    double critic_score = critic->scoreTrajectory(traj);
    cs.raw_score = critic_score;
    score.scores.push_back(cs);
    score.total += critic_score * cs.scale;
    if (best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}

double getSquareDistance(
  const geometry_msgs::msg::Pose2D & pose_a,
  const geometry_msgs::msg::Pose2D & pose_b)
{
  double x_diff = pose_a.x - pose_b.x;
  double y_diff = pose_a.y - pose_b.y;
  return x_diff * x_diff + y_diff * y_diff;
}

nav_2d_msgs::msg::Path2D DWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  nav_2d_msgs::msg::Path2D transformed_plan;
  if (global_plan_.poses.size() == 0) {
    throw nav_core2::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(tf_, global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav_core2::PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;
  double sq_dist_threshold = dist_threshold * dist_threshold;
  nav_2d_msgs::msg::Pose2DStamped stamped_pose;
  stamped_pose.header.frame_id = global_plan_.header.frame_id;

  for (unsigned int i = 0; i < global_plan_.poses.size(); i++) {
    bool should_break = false;
    if (getSquareDistance(robot_pose.pose, global_plan_.poses[i]) > sq_dist_threshold) {
      if (transformed_plan.poses.size() == 0) {
        // we need to skip to a point on the plan that is within a certain distance of the robot
        continue;
      } else {
        // we're done transforming points
        should_break = true;
      }
    }

    // now we'll transform until points are outside of our distance threshold
    stamped_pose.pose = global_plan_.poses[i];

    nav_2d_msgs::msg::Pose2DStamped newer_pose;
    nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), stamped_pose, newer_pose);
    transformed_plan.poses.push_back(newer_pose.pose);
    if (should_break) {break;}
  }

  // Prune both plans based on robot position
  // Note that this part of the algorithm assumes that the global plan starts
  // near the robot (at one point) Otherwise it may take a few iterations to
  // converge to the proper behavior
  if (prune_plan_) {
    assert(global_plan_.poses.size() >= transformed_plan.poses.size());
    std::vector<geometry_msgs::msg::Pose2D>::iterator it = transformed_plan.poses.begin();
    std::vector<geometry_msgs::msg::Pose2D>::iterator global_it = global_plan_.poses.begin();
    double sq_prune_dist = prune_distance_ * prune_distance_;
    while (it != transformed_plan.poses.end()) {
      const geometry_msgs::msg::Pose2D & w = *it;
      // Fixed error bound of 1 meter for now. Can reduce to a portion of the
      // map size or based on the resolution
      if (getSquareDistance(robot_pose.pose, w) < sq_prune_dist) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("DWBLocalPlanner"),
          "Nearest waypoint to <%f, %f> is <%f, %f>\n",
          robot_pose.pose.x,
          robot_pose.pose.y,
          w.x,
          w.y);
        break;
      }
      it = transformed_plan.poses.erase(it);
      global_it = global_plan_.poses.erase(global_it);
    }
    pub_.publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.size() == 0) {
    throw nav_core2::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace dwb_core


//  register this planner as a LocalPlanner plugin
// PLUGINLIB_EXPORT_CLASS(dwb_core::DWBLocalPlanner, nav_core2::LocalPlanner)
