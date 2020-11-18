/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Wilco Bonestroo
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

#include <math.h>
#include <vector>

#include <memory>
#include <string>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/prefer_forward.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"

TEST(PreferForward, StartNode)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  // rclcpp::spin_some(node->get_node_base_interface());

  std::string name = "test";
  std::string ns = "ns";
  critic->initialize(node, name, ns, costmap_ros);
  // rclcpp::spin_some(node->get_node_base_interface());

  critic->onInit();

  // rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_EQ(node->get_parameter(ns + "." + name + ".penalty").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter(ns + "." + name + ".strafe_x").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter(ns + "." + name + ".strafe_theta").as_double(), 0.2);
  EXPECT_EQ(node->get_parameter(ns + "." + name + ".theta_scale").as_double(), 10.0);
}

TEST(PreferForward, NegativeVelocityX)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;

  // score must be equal to the penalty (1.0) for any negative x velocity
  trajectory.velocity.x = -1.0;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = -0.00001;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = std::numeric_limits<double>::lowest();
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = -std::numeric_limits<double>::min();
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);
}

TEST(PreferForward, Strafe)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;

  // score must be equal to the penalty (1.0) when x vel is lower than 0.1
  // and theta is between -0.2 and 0.2
  trajectory.velocity.x = 0.05;
  trajectory.velocity.theta = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = 0.0999999;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.x = 0.000001;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.theta = -0.19;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);

  trajectory.velocity.theta = 0.19;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.0);
}

TEST(PreferForward, Normal)
{
  auto critic = std::make_shared<dwb_critics::PreferForwardCritic>();
  dwb_msgs::msg::Trajectory2D trajectory;

  // score must be equal to the theta * scaling factor (10.0)
  trajectory.velocity.x = 0.2;
  trajectory.velocity.theta = -0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * 10.0);

  trajectory.velocity.theta = 0.1;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.1 * 10.0);

  trajectory.velocity.theta = -0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * 10.0);

  trajectory.velocity.theta = 0.2;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 0.2 * 10.0);

  trajectory.velocity.theta = 1.5;
  EXPECT_EQ(critic->scoreTrajectory(trajectory), 1.5 * 10.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
