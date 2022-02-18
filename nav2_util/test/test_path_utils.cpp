// Copyright (c) 2022 Adam Aposhian
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/path_utils.hpp"
#include "gtest/gtest.h"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

using namespace nav2_util;

TEST(PathUtils, test_generate_straight)
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "test_frame";

  constexpr double path_length = 2.0;
  constexpr double spacing = 1.0;

  auto path = generate_path(
    start, spacing, {
    std::make_unique<Straight>(path_length)
  });
  EXPECT_EQ(path.poses.size(), 3u);
  for (const auto & pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, start.header.frame_id);
  }
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.z, 0.0);

  EXPECT_NEAR(path.poses[1].pose.position.x, 1.0, 0.1);
  EXPECT_NEAR(path.poses[1].pose.position.y, 0.0, 0.1);
  EXPECT_NEAR(path.poses[1].pose.position.z, 0.0, 0.1);

  EXPECT_NEAR(path.poses[2].pose.position.x, 2.0, 0.1);
  EXPECT_NEAR(path.poses[2].pose.position.y, 0.0, 0.1);
  EXPECT_NEAR(path.poses[2].pose.position.z, 0.0, 0.1);
}

TEST(PathUtils, test_generate_all)
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "test_frame";

  constexpr double spacing = 0.1;

  auto path = generate_path(
    start, spacing, {
    std::make_unique<Straight>(1.0),
    std::make_unique<LeftTurn>(1.0),
    std::make_unique<RightTurn>(1.0),
    std::make_unique<LeftTurnAround>(1.0),
    std::make_unique<RightTurnAround>(1.0),
    std::make_unique<LeftCircle>(1.0),
    std::make_unique<RightCircle>(1.0),
    std::make_unique<Arc>(1.0, M_2_PI), // another circle
  });
  constexpr double expected_path_length = 1.0 + M_PI_2 + M_PI_2 + M_PI + 3 * (M_2_PI);
  EXPECT_NEAR(path.poses.size(), 1 + static_cast<std::size_t>(expected_path_length / spacing), 5);
  for (const auto & pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, start.header.frame_id);
  }

  // Check the last pose
  auto & last_pose = path.poses.back();
  auto & last_position = last_pose.pose.position;
  EXPECT_NEAR(last_position.x, 3.0, 0.5);
  EXPECT_NEAR(last_position.y, 6.0, 0.5);
  EXPECT_DOUBLE_EQ(last_position.z, 0.0);
}
