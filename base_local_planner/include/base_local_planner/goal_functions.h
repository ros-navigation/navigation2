/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
#ifndef BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_
#define BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>

#include <string>
#include <cmath>

#include <angles/angles.h>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace base_local_planner {

  /**
   * @brief  return squared distance to check if the goal position has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return distance to goal
   */
  double getGoalPositionDistance(const geometry_msgs::msg::PoseStamped& global_pose, double goal_x, double goal_y);

  /**
   * @brief  return angle difference to goal to check if the goal orientation has been achieved
   * @param  global_pose The pose of the robot in the global frame
   * @param  goal_x The desired x value for the goal
   * @param  goal_y The desired y value for the goal
   * @return angular difference
   */
  double getGoalOrientationAngleDifference(const geometry_msgs::msg::PoseStamped& global_pose, double goal_th);

  /**
   * @brief  Publish a plan for visualization purposes
   * @param  path The plan to publish
   * @param  pub The published to use
   * @param  r,g,b,a The color and alpha value to use when publishing
   */
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path, const ros::Publisher& pub);

  /**
   * @brief  Trim off parts of the global plan that are far enough behind the robot
   * @param global_pose The pose of the robot in the global frame
   * @param plan The plan to be pruned
   * @param global_plan The plan to be pruned in the frame of the planner
   */
  void prunePlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& plan, std::vector<geometry_msgs::msg::PoseStamped>& global_plan);

  /**
   * @brief  Transforms the global plan of the robot from the planner frame to the frame of the costmap,
   * selects only the (first) part of the plan that is within the costmap area.
   * @param tf A reference to a transform listener
   * @param global_plan The plan to be transformed
   * @param robot_pose The pose of the robot in the global frame (same as costmap)
   * @param costmap A reference to the costmap being used so the window size for transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param transformed_plan Populated with the transformed plan
   */
  bool transformGlobalPlan(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
      const geometry_msgs::msg::PoseStamped& global_robot_pose,
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan);

  /**
   * @brief  Returns last pose in plan
   * @param tf A reference to a transform listener
   * @param global_plan The plan being followed
   * @param global_frame The global frame of the local planner
   * @param goal_pose the pose to copy into
   * @return True if achieved, false otherwise
   */
  bool getGoalPose(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
      const std::string& global_frame,
      geometry_msgs::msg::PoseStamped &goal_pose);

  /**
   * @brief  Check if the goal pose has been achieved
   * @param tf A reference to a transform listener
   * @param global_plan The plan being followed
   * @param costmap_ros A reference to the costmap object being used by the planner
   * @param global_frame The global frame of the local planner
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_vel The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_vel The translational velocity below which the robot is considered stopped
   * @param xy_goal_tolerance The translational tolerance on reaching the goal
   * @param yaw_goal_tolerance The rotational tolerance on reaching the goal
   * @return True if achieved, false otherwise
   */
  bool isGoalReached(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      geometry_msgs::msg::PoseStamped& global_pose,
      const nav_msgs::msg::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance);

  /**
   * @brief  Check whether the robot is stopped or not
   * @param base_odom The current odometry information for the robot
   * @param rot_stopped_velocity The rotational velocity below which the robot is considered stopped
   * @param trans_stopped_velocity The translational velocity below which the robot is considered stopped
   * @return True if the robot is stopped, false otherwise
   */
  bool stopped(const nav_msgs::msg::Odometry& base_odom,
      const double& rot_stopped_velocity,
      const double& trans_stopped_velocity);
};
#endif
