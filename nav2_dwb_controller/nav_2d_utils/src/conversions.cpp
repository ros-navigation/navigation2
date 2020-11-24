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

#include "nav_2d_utils/conversions.hpp"
#include <vector>
#include <string>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop
#include "nav2_util/geometry_utils.hpp"

namespace nav_2d_utils
{
using nav2_util::geometry_utils::orientationAroundZAxis;

geometry_msgs::msg::Twist twist2Dto3D(const nav_2d_msgs::msg::Twist2D & cmd_vel_2d)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = cmd_vel_2d.x;
  cmd_vel.linear.y = cmd_vel_2d.y;
  cmd_vel.angular.z = cmd_vel_2d.theta;
  return cmd_vel;
}

nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist & cmd_vel)
{
  nav_2d_msgs::msg::Twist2D cmd_vel_2d;
  cmd_vel_2d.x = cmd_vel.linear.x;
  cmd_vel_2d.y = cmd_vel.linear.y;
  cmd_vel_2d.theta = cmd_vel.angular.z;
  return cmd_vel_2d;
}

// nav_2d_msgs::msg::Pose2DStamped stampedPoseToPose2D(const tf2::Stamped<tf2::Pose>& pose)
// {
//   nav_2d_msgs::msg::Pose2DStamped pose2d;
//   pose2d.header.stamp = pose.stamp_;
//   pose2d.header.frame_id = pose.frame_id_;
//   pose2d.pose.x = pose.getOrigin().getX();
//   pose2d.pose.y = pose.getOrigin().getY();
//   pose2d.pose.theta = tf::getYaw(pose.getRotation());
//   return pose2d;
// }

nav_2d_msgs::msg::Pose2DStamped poseStampedToPose2D(const geometry_msgs::msg::PoseStamped & pose)
{
  nav_2d_msgs::msg::Pose2DStamped pose2d;
  pose2d.header = pose.header;
  pose2d.pose.x = pose.pose.position.x;
  pose2d.pose.y = pose.pose.position.y;
  pose2d.pose.theta = tf2::getYaw(pose.pose.orientation);
  return pose2d;
}

geometry_msgs::msg::Pose2D poseToPose2D(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf2::getYaw(pose.orientation);
  return pose2d;
}

geometry_msgs::msg::Pose pose2DToPose(const geometry_msgs::msg::Pose2D & pose2d)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose2d.x;
  pose.position.y = pose2d.y;
  pose.orientation = orientationAroundZAxis(pose2d.theta);
  return pose;
}

geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const nav_2d_msgs::msg::Pose2DStamped & pose2d)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose2d.header;
  pose.pose = pose2DToPose(pose2d.pose);
  return pose;
}

geometry_msgs::msg::PoseStamped pose2DToPoseStamped(
  const geometry_msgs::msg::Pose2D & pose2d,
  const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose.position.x = pose2d.x;
  pose.pose.position.y = pose2d.y;
  pose.pose.orientation = orientationAroundZAxis(pose2d.theta);
  return pose;
}

nav_msgs::msg::Path posesToPath(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path path;
  if (poses.empty()) {
    return path;
  }
  path.poses.resize(poses.size());
  path.header.frame_id = poses[0].header.frame_id;
  path.header.stamp = poses[0].header.stamp;
  for (unsigned int i = 0; i < poses.size(); i++) {
    path.poses[i] = poses[i];
  }
  return path;
}

nav_2d_msgs::msg::Path2D pathToPath2D(const nav_msgs::msg::Path & path)
{
  nav_2d_msgs::msg::Path2D path2d;
  path2d.header = path.header;
  for (auto & pose : path.poses) {
    path2d.poses.push_back(poseToPose2D(pose.pose));
  }
  return path2d;
}


nav_msgs::msg::Path poses2DToPath(
  const std::vector<geometry_msgs::msg::Pose2D> & poses,
  const std::string & frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.poses.resize(poses.size());
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  for (unsigned int i = 0; i < poses.size(); i++) {
    path.poses[i] = pose2DToPoseStamped(poses[i], frame, stamp);
  }
  return path;
}

nav_msgs::msg::Path pathToPath(const nav_2d_msgs::msg::Path2D & path2d)
{
  nav_msgs::msg::Path path;
  path.header = path2d.header;
  path.poses.resize(path2d.poses.size());
  for (unsigned int i = 0; i < path.poses.size(); i++) {
    path.poses[i].header = path2d.header;
    path.poses[i].pose = pose2DToPose(path2d.poses[i]);
  }
  return path;
}

}  // namespace nav_2d_utils
