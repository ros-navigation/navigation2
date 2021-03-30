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

#include "gtest/gtest.h"
#include "dwb_plugins/kinematic_parameters.hpp"

using rcl_interfaces::msg::Parameter;
using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterEvent;

class KinematicsHandlerTest : public dwb_plugins::KinematicsHandler
{
public:
  void simulate_event(
    const ParameterEvent::SharedPtr event)
  {
    on_parameter_event_callback(event);
  }
};

TEST(KinematicParameters, SetAllParameters) {
  std::string nodeName = "test_node";
  auto node = nav2_util::LifecycleNode::make_shared(nodeName);
  KinematicsHandlerTest kh;
  kh.initialize(node, nodeName);

  rcl_interfaces::msg::ParameterEvent event;
  Parameter p_minX, p_maxX, p_minY, p_maxY, p_accX, p_decelX, p_accY, p_decelY, p_minSpeedXY,
    p_maxSpeedXY, p_maxTheta, p_accTheta, p_decelTheta, p_minSpeedTheta;

  p_minX.name = nodeName + ".min_vel_x";
  p_minX.value.set__double_value(12.34);
  p_minX.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_maxX.name = nodeName + ".max_vel_x";
  p_maxX.value.set__double_value(23.45);
  p_maxX.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_minY.name = nodeName + ".min_vel_y";
  p_minY.value.set__double_value(34.56);
  p_minY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_maxY.name = nodeName + ".max_vel_y";
  p_maxY.value.set__double_value(45.67);
  p_maxY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_accX.name = nodeName + ".acc_lim_x";
  p_accX.value.set__double_value(56.78);
  p_accX.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_accY.name = nodeName + ".acc_lim_y";
  p_accY.value.set__double_value(67.89);
  p_accY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_decelX.name = nodeName + ".decel_lim_x";
  p_decelX.value.set__double_value(78.90);
  p_decelX.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_decelY.name = nodeName + ".decel_lim_y";
  p_decelY.value.set__double_value(89.01);
  p_decelY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_minSpeedXY.name = nodeName + ".min_speed_xy";
  p_minSpeedXY.value.set__double_value(90.12);
  p_minSpeedXY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_maxSpeedXY.name = nodeName + ".max_speed_xy";
  p_maxSpeedXY.value.set__double_value(123.456);
  p_maxSpeedXY.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_maxTheta.name = nodeName + ".max_vel_theta";
  p_maxTheta.value.set__double_value(345.678);
  p_maxTheta.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_accTheta.name = nodeName + ".acc_lim_theta";
  p_accTheta.value.set__double_value(234.567);
  p_accTheta.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_decelTheta.name = nodeName + ".decel_lim_theta";
  p_decelTheta.value.set__double_value(456.789);
  p_decelTheta.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  p_minSpeedTheta.name = nodeName + ".min_speed_theta";
  p_minSpeedTheta.value.set__double_value(567.890);
  p_minSpeedTheta.value.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

  event.changed_parameters.push_back(p_minX);
  event.changed_parameters.push_back(p_maxX);
  event.changed_parameters.push_back(p_minY);
  event.changed_parameters.push_back(p_maxY);
  event.changed_parameters.push_back(p_accX);
  event.changed_parameters.push_back(p_accY);
  event.changed_parameters.push_back(p_decelX);
  event.changed_parameters.push_back(p_decelY);
  event.changed_parameters.push_back(p_minSpeedXY);
  event.changed_parameters.push_back(p_maxSpeedXY);
  event.changed_parameters.push_back(p_maxTheta);
  event.changed_parameters.push_back(p_accTheta);
  event.changed_parameters.push_back(p_decelTheta);
  event.changed_parameters.push_back(p_minSpeedTheta);

  kh.simulate_event(std::make_shared<ParameterEvent>(event));

  dwb_plugins::KinematicParameters kp = kh.getKinematics();

  EXPECT_EQ(kp.getMinX(), 12.34);
  EXPECT_EQ(kp.getMaxX(), 23.45);
  EXPECT_EQ(kp.getMinY(), 34.56);
  EXPECT_EQ(kp.getMaxY(), 45.67);
  EXPECT_EQ(kp.getAccX(), 56.78);
  EXPECT_EQ(kp.getAccY(), 67.89);
  EXPECT_EQ(kp.getDecelX(), 78.90);
  EXPECT_EQ(kp.getDecelY(), 89.01);
  EXPECT_EQ(kp.getMinSpeedXY(), 90.12);
  EXPECT_EQ(kp.getMaxSpeedXY(), 123.456);
  EXPECT_EQ(kp.getAccTheta(), 234.567);
  EXPECT_EQ(kp.getMaxTheta(), 345.678);
  EXPECT_EQ(kp.getDecelTheta(), 456.789);
  EXPECT_EQ(kp.getMinSpeedTheta(), 567.890);
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
