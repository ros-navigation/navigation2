/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#include <vector>

#include "gtest/gtest.h"
#include "nav_2d_utils/conversions.hpp"

using nav_2d_utils::posesToPath;
using nav_2d_utils::pathToPath;

// geometry_msgs::msg::Pose2D origin;

TEST(nav_2d_utils, PosesToPathEmpty)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  nav_msgs::msg::Path path = posesToPath(poses);

  EXPECT_EQ(path.poses.size(), 0ul);
}


TEST(nav_2d_utils, PosesToPathNonEmpty)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped pose1;
  geometry_msgs::msg::PoseStamped pose2;
  poses.push_back(pose1);
  poses.push_back(pose2);

  nav_msgs::msg::Path path = posesToPath(poses);

  EXPECT_EQ(path.poses.size(), 2ul);
}

TEST(nav_2d_utils, PathToPathEmpty)
{
  nav_2d_msgs::msg::Path2D path2d;
  nav_msgs::msg::Path path = pathToPath(path2d);
  EXPECT_EQ(path.poses.size(), 0ul);
}

TEST(nav_2d_utils, PathToPathNoNEmpty)
{
  nav_2d_msgs::msg::Path2D path2d;
  geometry_msgs::msg::Pose2D pose1;
  geometry_msgs::msg::Pose2D pose2;
  path2d.poses.push_back(pose1);
  path2d.poses.push_back(pose2);

  nav_msgs::msg::Path path = pathToPath(path2d);
  EXPECT_EQ(path.poses.size(), 2ul);
}
