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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/parameters.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_core
{

DWBLocalPlanner::DWBLocalPlanner()
: traj_gen_loader_("dwb_core", "dwb_core::TrajectoryGenerator"),
  critic_loader_("dwb_core", "dwb_core::TrajectoryCritic")
{
}

void DWBLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = node;
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  dwb_plugin_name_ = name;
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".critics");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".default_critic_namespaces");
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".prune_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".prune_distance",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".debug_trajectory_details",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".trajectory_generator_name",
    rclcpp::ParameterValue(std::string("dwb_plugins::StandardTrajectoryGenerator")));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".transform_tolerance",
    rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".shorten_transformed_plan",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node_, dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    rclcpp::ParameterValue(true));

  std::string traj_generator_name;

  double transform_tolerance;
  node_->get_parameter(dwb_plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  RCLCPP_INFO(node_->get_logger(), "Setting transform_tolerance to %f", transform_tolerance);

  node_->get_parameter(dwb_plugin_name_ + ".prune_plan", prune_plan_);
  node_->get_parameter(dwb_plugin_name_ + ".prune_distance", prune_distance_);
  node_->get_parameter(dwb_plugin_name_ + ".debug_trajectory_details", debug_trajectory_details_);
  node_->get_parameter(dwb_plugin_name_ + ".trajectory_generator_name", traj_generator_name);
  node_->get_parameter(
    dwb_plugin_name_ + ".short_circuit_trajectory_evaluation",
    short_circuit_trajectory_evaluation_);
  node->get_parameter(dwb_plugin_name_ + ".shorten_transformed_plan", shorten_transformed_plan_);

  pub_ = std::make_unique<DWBPublisher>(node_, dwb_plugin_name_);
  pub_->on_configure();

  traj_generator_ = traj_gen_loader_.createUniqueInstance(traj_generator_name);

  traj_generator_->initialize(node_, dwb_plugin_name_);

  try {
    loadCritics();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Couldn't load critics! Caught exception: %s", e.what());
    throw;
  }
}

void
DWBLocalPlanner::activate()
{
  pub_->on_activate();
}

void
DWBLocalPlanner::deactivate()
{
  pub_->on_deactivate();
}

void
DWBLocalPlanner::cleanup()
{
  pub_->on_cleanup();

  traj_generator_.reset();
}

std::string
DWBLocalPlanner::resolveCriticClassName(std::string base_name)
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

void
DWBLocalPlanner::loadCritics()
{
  node_->get_parameter(dwb_plugin_name_ + ".default_critic_namespaces", default_critic_namespaces_);
  if (default_critic_namespaces_.size() == 0) {
    default_critic_namespaces_.push_back("dwb_critics");
  }

  std::vector<std::string> critic_names;
  if (!node_->get_parameter(dwb_plugin_name_ + ".critics", critic_names)) {
    loadBackwardsCompatibleParameters();
  }

  node_->get_parameter(dwb_plugin_name_ + ".critics", critic_names);
  for (unsigned int i = 0; i < critic_names.size(); i++) {
    std::string critic_plugin_name = critic_names[i];
    std::string plugin_class;

    declare_parameter_if_not_declared(
      node_, dwb_plugin_name_ + "." + critic_plugin_name + ".class",
      rclcpp::ParameterValue(critic_plugin_name));
    node_->get_parameter(dwb_plugin_name_ + "." + critic_plugin_name + ".class", plugin_class);

    plugin_class = resolveCriticClassName(plugin_class);

    TrajectoryCritic::Ptr plugin = critic_loader_.createUniqueInstance(plugin_class);
    RCLCPP_INFO(
      node_->get_logger(),
      "Using critic \"%s\" (%s)", critic_plugin_name.c_str(), plugin_class.c_str());
    critics_.push_back(plugin);
    try {
      plugin->initialize(node_, critic_plugin_name, dwb_plugin_name_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Couldn't initialize critic plugin!");
      throw;
    }
    RCLCPP_INFO(node_->get_logger(), "Critic plugin initialized");
  }
}

void
DWBLocalPlanner::loadBackwardsCompatibleParameters()
{
  std::vector<std::string> critic_names;
  RCLCPP_INFO(
    node_->get_logger(),
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
  node_->set_parameters({rclcpp::Parameter(dwb_plugin_name_ + ".critics", critic_names)});

  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".path_distance_bias");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".goal_distance_bias");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".occdist_scale");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".max_scaling_factor");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".scaling_speed");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".PathAlign.scale");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".GoalAlign.scale");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".PathDist.scale");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".GoalDist.scale");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".ObstacleFootprint.scale");
  declare_parameter_if_not_declared(
    node_,
    dwb_plugin_name_ + ".ObstacleFootprint.max_scaling_factor");
  declare_parameter_if_not_declared(node_, dwb_plugin_name_ + ".ObstacleFootprint.scaling_speed");

  /* *INDENT-OFF* */
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".path_distance_bias",
    dwb_plugin_name_ + ".PathAlign.scale", 32.0, false);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".goal_distance_bias",
    dwb_plugin_name_ + ".GoalAlign.scale", 24.0, false);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".path_distance_bias",
    dwb_plugin_name_ + ".PathDist.scale", 32.0);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".goal_distance_bias",
    dwb_plugin_name_ + ".GoalDist.scale", 24.0);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".occdist_scale",
    dwb_plugin_name_ + ".ObstacleFootprint.scale", 0.01);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".max_scaling_factor",
    dwb_plugin_name_ + ".ObstacleFootprint.max_scaling_factor", 0.2);
  nav_2d_utils::moveParameter(node_, dwb_plugin_name_ + ".scaling_speed",
    dwb_plugin_name_ + ".ObstacleFootprint.scaling_speed", 0.25);
  /* *INDENT-ON* */
}

void
DWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  auto path2d = nav_2d_utils::pathToPath2D(path);
  for (TrajectoryCritic::Ptr critic : critics_) {
    critic->reset();
  }

  traj_generator_->reset();

  pub_->publishGlobalPlan(path2d);
  global_plan_ = path2d;
}

geometry_msgs::msg::TwistStamped
DWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results = nullptr;
  if (pub_->shouldRecordEvaluation()) {
    results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  }

  try {
    nav_2d_msgs::msg::Twist2DStamped cmd_vel2d = computeVelocityCommands(
      nav_2d_utils::poseStampedToPose2D(pose),
      nav_2d_utils::twist3Dto2D(velocity), results);
    pub_->publishEvaluation(results);
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist = nav_2d_utils::twist2Dto3D(cmd_vel2d.velocity);
    return cmd_vel;
  } catch (const nav2_core::PlannerException & e) {
    pub_->publishEvaluation(results);
    throw;
  }
}

void
DWBLocalPlanner::prepareGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose, nav_2d_msgs::msg::Path2D & transformed_plan,
  nav_2d_msgs::msg::Pose2DStamped & goal_pose, bool publish_plan)
{
  transformed_plan = transformGlobalPlan(pose);
  if (publish_plan) {
    pub_->publishTransformedPlan(transformed_plan);
  }

  goal_pose.header.frame_id = global_plan_.header.frame_id;
  goal_pose.pose = global_plan_.poses.back();
  nav_2d_utils::transformPose(
    tf_, costmap_ros_->getGlobalFrameID(), goal_pose,
    goal_pose, transform_tolerance_);
}

nav_2d_msgs::msg::Twist2DStamped
DWBLocalPlanner::computeVelocityCommands(
  const nav_2d_msgs::msg::Pose2DStamped & pose,
  const nav_2d_msgs::msg::Twist2D & velocity,
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results)
{
  if (results) {
    results->header.frame_id = pose.header.frame_id;
    results->header.stamp = node_->now();
  }

  nav_2d_msgs::msg::Path2D transformed_plan;
  nav_2d_msgs::msg::Pose2DStamped goal_pose;

  prepareGlobalPlan(pose, transformed_plan, goal_pose);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (TrajectoryCritic::Ptr critic : critics_) {
    if (critic->prepare(pose.pose, velocity, goal_pose.pose, transformed_plan) == false) {
      RCLCPP_WARN(rclcpp::get_logger("DWBLocalPlanner"), "A scoring function failed to prepare");
    }
  }

  try {
    dwb_msgs::msg::TrajectoryScore best = coreScoringAlgorithm(pose.pose, velocity, results);

    // Return Value
    nav_2d_msgs::msg::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.velocity = best.traj.velocity;

    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr critic : critics_) {
      critic->debrief(cmd_vel.velocity);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, best.traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    return cmd_vel;
  } catch (const dwb_core::NoLegalTrajectoriesException & e) {
    nav_2d_msgs::msg::Twist2D empty_cmd;
    dwb_msgs::msg::Trajectory2D empty_traj;
    // debrief stateful scoring functions
    for (TrajectoryCritic::Ptr critic : critics_) {
      critic->debrief(empty_cmd);
    }

    lock.unlock();

    pub_->publishLocalPlan(pose.header, empty_traj);
    pub_->publishCostGrid(costmap_ros_, critics_);

    throw;
  }
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::coreScoringAlgorithm(
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
    } catch (const dwb_core::IllegalTrajectoryException & e) {
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
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "DWBLocalPlanner"), "%.2f: %10s/%s", x.second,
          x.first.first.c_str(), x.first.second.c_str());
      }
    }
    throw NoLegalTrajectoriesException(tracker);
  }

  return best;
}

dwb_msgs::msg::TrajectoryScore
DWBLocalPlanner::scoreTrajectory(
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
    if (short_circuit_trajectory_evaluation_ && best_score > 0 && score.total > best_score) {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      break;
    }
  }

  return score;
}

double
getSquareDistance(
  const geometry_msgs::msg::Pose2D & pose_a,
  const geometry_msgs::msg::Pose2D & pose_b)
{
  double x_diff = pose_a.x - pose_b.x;
  double y_diff = pose_a.y - pose_b.y;

  return x_diff * x_diff + y_diff * y_diff;
}

nav_2d_msgs::msg::Path2D
DWBLocalPlanner::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  if (global_plan_.poses.size() == 0) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
  if (!nav_2d_utils::transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw dwb_core::
          PlannerTFException("Unable to transform robot pose into global plan's frame");
  }

  // we'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;
  double sq_dist_threshold = dist_threshold * dist_threshold;

  // If prune_plan is enabled (it is by default) then we want to restrict the
  // plan to distances within that range as well.
  double sq_prune_dist = prune_distance_ * prune_distance_;

  // Set the maximum distance we'll include points before getting to the part
  // of the path where the robot is located (the start of the plan). Basically,
  // these are the points the robot has already passed.
  double sq_transform_start_threshold;
  if (prune_plan_) {
    sq_transform_start_threshold = std::min(sq_dist_threshold, sq_prune_dist);
  } else {
    sq_transform_start_threshold = sq_dist_threshold;
  }

  // Set the maximum distance we'll include points after the part of the part of
  // the plan near the robot (the end of the plan). This determines the amount
  // of the plan passed on to the critics
  double sq_transform_end_threshold;
  if (shorten_transformed_plan_) {
    sq_transform_end_threshold = std::min(sq_dist_threshold, sq_prune_dist);
  } else {
    sq_transform_end_threshold = sq_dist_threshold;
  }

  // Find the first pose in the plan that's less than sq_transform_start_threshold
  // from the robot.
  auto transformation_begin = std::find_if(
    begin(global_plan_.poses), end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return getSquareDistance(robot_pose.pose, global_plan_pose) < sq_transform_start_threshold;
    });

  // Find the first pose in the end of the plan that's further than sq_transform_end_threshold
  // from the robot
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return getSquareDistance(robot_pose.pose, global_plan_pose) > sq_transform_end_threshold;
    });

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose2D from global
  // frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      nav_2d_msgs::msg::Pose2DStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose;
      nav_2d_utils::transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose.pose;
    };

  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration.
  if (prune_plan_) {
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    pub_->publishGlobalPlan(global_plan_);
  }

  if (transformed_plan.poses.size() == 0) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }
  return transformed_plan;
}

}  // namespace dwb_core

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  dwb_core::DWBLocalPlanner,
  nav2_core::Controller)
