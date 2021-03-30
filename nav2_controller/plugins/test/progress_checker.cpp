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
#include "nav_2d_utils/conversions.hpp"
#include "nav2_util/lifecycle_node.hpp"

using nav2_controller::SimpleProgressChecker;

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
  double x0, double y0,
  double x1, double y1,
  int delay,
  bool expected_result)
{
  pc.reset();
  geometry_msgs::msg::PoseStamped pose0, pose1;
  pose0.pose.position.x = x0;
  pose0.pose.position.y = y0;
  pose1.pose.position.x = x1;
  pose1.pose.position.y = y1;
  EXPECT_TRUE(pc.check(pose0));
  rclcpp::sleep_for(std::chrono::seconds(delay));
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
  checkMacro(pc, 0, 0, 0, 0, 1, true);
  checkMacro(pc, 0, 0, 1, 0, 1, true);
  checkMacro(pc, 0, 0, 0, 1, 1, true);
  checkMacro(pc, 0, 0, 1, 0, 11, true);
  checkMacro(pc, 0, 0, 0, 1, 11, true);
  checkMacro(pc, 0, 0, 0, 0, 11, false);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
