// Copyright (c) 2026 iam-shanmukha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "nav2_bt_navigator/navigators/navigate_to_pose.hpp"
#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "rclcpp/rclcpp.hpp"

class NavigateToPoseNavigatorWrapper : public nav2_bt_navigator::NavigateToPoseNavigator
{
public:
  double getSearchWindow() const {return search_window_;}
};

class NavigateThroughPosesNavigatorWrapper
  : public nav2_bt_navigator::NavigateThroughPosesNavigator
{
public:
  double getSearchWindow() const {return search_window_;}
};

template<typename NavigatorT>
double configureAndGetSearchWindow(const std::string & navigator_name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{navigator_name + ".search_window", 5.0}});
  auto node = std::make_shared<nav2::LifecycleNode>("test_bt_navigator", options);

  nav2_core::FeedbackUtils feedback_utils;
  feedback_utils.robot_frame = "base_link";
  feedback_utils.global_frame = "map";
  feedback_utils.transform_tolerance = 0.1;
  feedback_utils.tf = std::make_shared<nav2::TransformBuffer>(node->get_clock());

  nav2_core::NavigatorMuxer muxer;
  NavigatorT navigator;
  EXPECT_TRUE(
    navigator.on_configure(
      node, std::vector<std::string>{}, feedback_utils, &muxer, nullptr));

  EXPECT_TRUE(node->has_parameter(navigator_name + ".search_window"));
  double search_window = navigator.getSearchWindow();
  navigator.on_cleanup();
  return search_window;
}

TEST(NavigatorParams, NavigateToPoseReadsSearchWindow)
{
  EXPECT_DOUBLE_EQ(
    configureAndGetSearchWindow<NavigateToPoseNavigatorWrapper>("navigate_to_pose"), 5.0);
}

TEST(NavigatorParams, NavigateThroughPosesReadsSearchWindow)
{
  EXPECT_DOUBLE_EQ(
    configureAndGetSearchWindow<NavigateThroughPosesNavigatorWrapper>(
      "navigate_through_poses"), 5.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
