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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"

using nav2_controller::SimpleProgressChecker;
using nav2_controller::PoseProgressChecker;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

void checkMacro(
  nav2_core::ProgressChecker & pc,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  int delay,
  bool expected_result)
{
  pc.reset();
  geometry_msgs::msg::PoseStamped pose0, pose1;
  pose0.pose.position.x = x0;
  pose0.pose.position.y = y0;
  pose0.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta0);
  pose1.pose.position.x = x1;
  pose1.pose.position.y = y1;
  pose1.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta1);
  EXPECT_TRUE(pc.check(pose0));
  rclcpp::sleep_for(std::chrono::milliseconds(delay));
  if (expected_result) {
    EXPECT_TRUE(pc.check(pose1));
  } else {
    EXPECT_FALSE(pc.check(pose1));
  }
}

TEST(SimpleProgressChecker, progress_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("progress_checker");

  nav2_core::ProgressChecker * pc = new SimpleProgressChecker;
  pc->reset();
  delete pc;
  EXPECT_TRUE(true);
}

TEST(SimpleProgressChecker, unit_tests)
{
  auto x = std::make_shared<TestLifecycleNode>("progress_checker");

  SimpleProgressChecker pc;
  pc.initialize(x, "nav2_controller");

  double time_allowance = 0.5;
  int half_time_allowance_ms = static_cast<int>(time_allowance * 0.5 * 1000);
  int twice_time_allowance_ms = static_cast<int>(time_allowance * 2.0 * 1000);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("nav2_controller.movement_time_allowance", time_allowance)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(
    x->get_parameter("nav2_controller.movement_time_allowance").as_double(),
    time_allowance);

  // BELOW time allowance (set to time_allowance)
  // no movement
  checkMacro(pc, 0, 0, 0, 0, 0, 0, half_time_allowance_ms, true);
  // translation below required_movement_radius (default 0.5)
  checkMacro(pc, 0, 0, 0, 0.25, 0, 0, half_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 0.25, 0, half_time_allowance_ms, true);
  // translation above required_movement_radius (default 0.5)
  checkMacro(pc, 0, 0, 0, 1, 0, 0, half_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 1, 0, half_time_allowance_ms, true);

  // ABOVE time allowance (set to time_allowance)
  // no movement
  checkMacro(pc, 0, 0, 0, 0, 0, 0, twice_time_allowance_ms, false);
  // translation below required_movement_radius (default 0.5)
  checkMacro(pc, 0, 0, 0, 0.25, 0, 0, twice_time_allowance_ms, false);
  checkMacro(pc, 0, 0, 0, 0, 0.25, 0, twice_time_allowance_ms, false);
  // translation above required_movement_radius (default 0.5)
  checkMacro(pc, 0, 0, 0, 1, 0, 0, twice_time_allowance_ms, true);
  checkMacro(pc, 0, 0, 0, 0, 1, 0, twice_time_allowance_ms, true);
}

TEST(PoseProgressChecker, pose_progress_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("pose_progress_checker");

  PoseProgressChecker * rpc = new PoseProgressChecker;
  rpc->reset();
  delete rpc;
  EXPECT_TRUE(true);
}

TEST(PoseProgressChecker, unit_tests)
{
  auto x = std::make_shared<TestLifecycleNode>("pose_progress_checker");

  PoseProgressChecker rpc;
  rpc.initialize(x, "nav2_controller");

  double time_allowance = 0.5;
  int half_time_allowance_ms = static_cast<int>(time_allowance * 0.5 * 1000);
  int twice_time_allowance_ms = static_cast<int>(time_allowance * 2.0 * 1000);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("nav2_controller.movement_time_allowance", time_allowance)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(
    x->get_parameter("nav2_controller.movement_time_allowance").as_double(),
    time_allowance);

  // BELOW time allowance (set to time_allowance)
  // no movement
  checkMacro(rpc, 0, 0, 0, 0, 0, 0, half_time_allowance_ms, true);
  // translation below required_movement_radius (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0.25, 0, 0, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0.25, 0, half_time_allowance_ms, true);
  // rotation below required_movement_angle (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0, 0, 0.25, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -0.25, half_time_allowance_ms, true);
  // translation above required_movement_radius (default 0.5)
  checkMacro(rpc, 0, 0, 0, 1, 0, 0, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 1, 0, half_time_allowance_ms, true);
  // rotation above required_movement_angle (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0, 0, 1, half_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -1, half_time_allowance_ms, true);

  // ABOVE time allowance (set to time_allowance)
  // no movement
  checkMacro(rpc, 0, 0, 0, 0, 0, 0, twice_time_allowance_ms, false);
  // translation below required_movement_radius (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0.25, 0, 0, twice_time_allowance_ms, false);
  checkMacro(rpc, 0, 0, 0, 0, 0.25, 0, twice_time_allowance_ms, false);
  // rotation below required_movement_angle (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0, 0, 0.25, twice_time_allowance_ms, false);
  checkMacro(rpc, 0, 0, 0, 0, 0, -0.25, twice_time_allowance_ms, false);
  // translation above required_movement_radius (default 0.5)
  checkMacro(rpc, 0, 0, 0, 1, 0, 0, twice_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 1, 0, twice_time_allowance_ms, true);
  // rotation above required_movement_angle (default 0.5)
  checkMacro(rpc, 0, 0, 0, 0, 0, 1, twice_time_allowance_ms, true);
  checkMacro(rpc, 0, 0, 0, 0, 0, -1, twice_time_allowance_ms, true);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
