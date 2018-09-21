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

#ifndef NAV_2D_UTILS_CONVERSIONS_H
#define NAV_2D_UTILS_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <nav_2d_msgs/Twist2D.h>
#include <nav_2d_msgs/Path2D.h>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <vector>
#include <string>

namespace nav_2d_utils
{

geometry_msgs::Twist twist2Dto3D(const nav_2d_msgs::Twist2D& cmd_vel_2d);
nav_2d_msgs::Pose2DStamped stampedPoseToPose2D(const tf::Stamped<tf::Pose>& pose);
nav_2d_msgs::Pose2DStamped poseStampedToPose2D(const geometry_msgs::PoseStamped& pose);
geometry_msgs::Pose pose2DToPose(const geometry_msgs::Pose2D& pose2d);
geometry_msgs::PoseStamped pose2DToPoseStamped(const nav_2d_msgs::Pose2DStamped& pose2d);
geometry_msgs::PoseStamped pose2DToPoseStamped(const geometry_msgs::Pose2D& pose2d,
                                               const std::string& frame, const ros::Time& stamp);
nav_msgs::Path posesToPath(const std::vector<geometry_msgs::PoseStamped>& poses);
nav_2d_msgs::Path2D posesToPath2D(const std::vector<geometry_msgs::PoseStamped>& poses);
nav_msgs::Path poses2DToPath(const std::vector<geometry_msgs::Pose2D>& poses,
                             const std::string& frame, const ros::Time& stamp);
nav_msgs::Path pathToPath(const nav_2d_msgs::Path2D& path2d);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_CONVERSIONS_H
