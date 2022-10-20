/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Samsung Research Russia
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
 *
 * Author: Alexey Merzlyakov
 */

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "dwb_plugins/kinematic_parameters.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = 1e-5;

static const char NODE_NAME[] = "test_node";
static const double MAX_VEL_X = 40.0;
static const double MAX_VEL_Y = 30.0;
static const double MAX_VEL_THETA = 15.0;
static const double MAX_VEL_LINEAR = 50.0;

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    const std::string node_name = NODE_NAME;
    node_ = nav2_util::LifecycleNode::make_shared(node_name);

    node_->declare_parameter(
      node_name + ".max_vel_x", rclcpp::ParameterValue(MAX_VEL_X));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_x", MAX_VEL_X));

    node_->declare_parameter(
      node_name + ".max_vel_y", rclcpp::ParameterValue(MAX_VEL_Y));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_y", MAX_VEL_Y));

    node_->declare_parameter(
      node_name + ".max_vel_theta", rclcpp::ParameterValue(MAX_VEL_THETA));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_vel_theta", MAX_VEL_THETA));

    node_->declare_parameter(
      node_name + ".max_speed_xy", rclcpp::ParameterValue(MAX_VEL_LINEAR));
    node_->set_parameter(
      rclcpp::Parameter(node_name + ".max_speed_xy", MAX_VEL_LINEAR));
  }

  ~TestNode() {}

protected:
  nav2_util::LifecycleNode::SharedPtr node_;
};

TEST_F(TestNode, TestPercentLimit)
{
  dwb_plugins::KinematicsHandler kh;
  kh.initialize(node_, NODE_NAME);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);

  // Set speed limit 30% from maximum robot speed
  kh.setSpeedLimit(30, true);

  // Update KinematicParameters values from KinematicsHandler
  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA * 0.3, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR * 0.3, EPSILON);

  // Restore maximum speed to its default
  kh.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);

  // Update KinematicParameters values from KinematicsHandler
  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);
}

TEST_F(TestNode, TestAbsoluteLimit)
{
  dwb_plugins::KinematicsHandler kh;
  kh.initialize(node_, NODE_NAME);

  dwb_plugins::KinematicParameters kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);

  // Set speed limit 35.0 m/s
  kh.setSpeedLimit(35.0, false);

  // Update KinematicParameters values from KinematicsHandler
  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA * 35.0 / MAX_VEL_LINEAR, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), 35.0, EPSILON);

  // Restore maximum speed to its default
  kh.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);

  // Update KinematicParameters values from KinematicsHandler
  kp = kh.getKinematics();
  EXPECT_NEAR(kp.getMaxX(), MAX_VEL_X, EPSILON);
  EXPECT_NEAR(kp.getMaxY(), MAX_VEL_Y, EPSILON);
  EXPECT_NEAR(kp.getMaxTheta(), MAX_VEL_THETA, EPSILON);
  EXPECT_NEAR(kp.getMaxSpeedXY(), MAX_VEL_LINEAR, EPSILON);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
