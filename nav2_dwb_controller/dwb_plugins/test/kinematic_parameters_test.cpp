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

#include <string>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "dwb_plugins/kinematic_parameters.hpp"

using rcl_interfaces::msg::Parameter;
using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterEvent;

TEST(KinematicParameters, SetAllParameters) {
  std::string nodeName = "test_node";
  auto node = std::make_shared<nav2::LifecycleNode>(nodeName);
  dwb_plugins::KinematicsHandler kh;
  kh.initialize(node, nodeName);
  kh.activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter(nodeName + ".min_vel_x", 12.34),
    rclcpp::Parameter(nodeName + ".max_vel_x", 23.45),
    rclcpp::Parameter(nodeName + ".min_vel_y", 34.56),
    rclcpp::Parameter(nodeName + ".max_vel_y", 45.67),
    rclcpp::Parameter(nodeName + ".acc_lim_x", 56.78),
    rclcpp::Parameter(nodeName + ".acc_lim_y", 67.89),
    rclcpp::Parameter(nodeName + ".decel_lim_x", -78.90),
    rclcpp::Parameter(nodeName + ".decel_lim_y", -89.01),
    rclcpp::Parameter(nodeName + ".min_speed_xy", 90.12),
    rclcpp::Parameter(nodeName + ".max_speed_xy", 123.456),
    rclcpp::Parameter(nodeName + ".max_vel_theta", 345.678),
    rclcpp::Parameter(nodeName + ".acc_lim_theta", 234.567),
    rclcpp::Parameter(nodeName + ".decel_lim_theta", -456.789),
    rclcpp::Parameter(nodeName + ".min_speed_theta", 567.890),
  });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();

  EXPECT_EQ(kp.getMinX(), 12.34);
  EXPECT_EQ(kp.getMaxX(), 23.45);
  EXPECT_EQ(kp.getMinY(), 34.56);
  EXPECT_EQ(kp.getMaxY(), 45.67);
  EXPECT_EQ(kp.getAccX(), 56.78);
  EXPECT_EQ(kp.getAccY(), 67.89);
  EXPECT_EQ(kp.getDecelX(), -78.90);
  EXPECT_EQ(kp.getDecelY(), -89.01);
  EXPECT_EQ(kp.getMinSpeedXY(), 90.12);
  EXPECT_EQ(kp.getMaxSpeedXY(), 123.456);
  EXPECT_EQ(kp.getAccTheta(), 234.567);
  EXPECT_EQ(kp.getMaxTheta(), 345.678);
  EXPECT_EQ(kp.getDecelTheta(), -456.789);
  EXPECT_EQ(kp.getMinSpeedTheta(), 567.890);

  results = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter(nodeName + ".decel_lim_x", 1.0)
  });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  
  EXPECT_EQ(kp.getDecelX(), -78.90);

  results = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter(nodeName + ".max_vel_x", -1.0)
  });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  
  EXPECT_EQ(kp.getMaxX(), 23.45);

  results = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter(nodeName + ".acc_lim_x", -0.1)
  });

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);
  
  EXPECT_EQ(kp.getAccX(), 56.78);
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
