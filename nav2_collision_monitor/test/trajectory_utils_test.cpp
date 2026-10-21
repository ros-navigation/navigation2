// Copyright (c) 2026 Polymath Robotics
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

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/trajectory.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"

#include "nav2_collision_monitor/trajectory_utils.hpp"
#include "nav2_collision_monitor/types.hpp"

using nav2_collision_monitor::toTrajectory;

static constexpr double EPSILON = 1e-6;
static constexpr double MAX_TIME_STEP = 0.1;

namespace
{

builtin_interfaces::msg::Time stamp(const double seconds)
{
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>(std::round((seconds - msg.sec) * 1e9));
  return msg;
}

// Straight line along +x in "odom": point i is stamped 100 + i * 0.1 and is at x = i * 0.1
nav_msgs::msg::Trajectory makeLine(const std::size_t size)
{
  nav_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = stamp(100.0);
  trajectory.header.frame_id = "odom";
  for (std::size_t i = 0; i < size; i++) {
    nav_msgs::msg::TrajectoryPoint point;
    point.header.stamp = stamp(100.0 + 0.1 * static_cast<double>(i));
    point.header.frame_id = "odom";
    point.pose.position.x = 0.1 * static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

}  // namespace

TEST(TrajectoryUtilsTest, testToTrajectory)
{
  // The trajectory frame is the base frame turned 90 degrees and shifted to (1, 2)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI / 2.0);
  const tf2::Transform to_base(q, tf2::Vector3(1.0, 2.0, 0.0));

  nav2_collision_monitor::Trajectory trajectory;
  std::string reason;
  ASSERT_TRUE(toTrajectory(makeLine(5), to_base, MAX_TIME_STEP, trajectory, reason));
  ASSERT_EQ(trajectory.size(), 5u);
  for (std::size_t i = 0; i < trajectory.size(); i++) {
    // Times are stamp differences from the first point, and poses are in the base frame
    EXPECT_NEAR(trajectory[i].time_s, 0.1 * static_cast<double>(i), EPSILON);
    EXPECT_NEAR(trajectory[i].pose.x, 1.0, EPSILON);
    EXPECT_NEAR(trajectory[i].pose.y, 2.0 + 0.1 * static_cast<double>(i), EPSILON);
    EXPECT_NEAR(trajectory[i].pose.theta, M_PI / 2.0, EPSILON);
  }

  // A point frame_id may be left empty
  nav_msgs::msg::Trajectory message = makeLine(3);
  for (auto & point : message.points) {
    point.header.frame_id.clear();
  }
  EXPECT_TRUE(
    toTrajectory(message, tf2::Transform::getIdentity(), MAX_TIME_STEP, trajectory, reason));
}

TEST(TrajectoryUtilsTest, testToTrajectoryRejectsInvalid)
{
  struct Case
  {
    std::string expected_reason;
    std::function<void(nav_msgs::msg::Trajectory &)> mutate;
  };
  const std::vector<Case> cases{
    {"empty", [](auto & t) {t.points.clear();}},
    {"invalid pose", [](auto & t) {
        t.points[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
      }},
    {"invalid pose", [](auto & t) {t.points[1].pose.orientation.w = 0.0;}},
    {"increase", [](auto & t) {t.points[2].header.stamp = t.points[1].header.stamp;}},
    {"increase", [](auto & t) {t.points[2].header.stamp = stamp(100.05);}},
    {"trajectory frame", [](auto & t) {t.points[2].header.frame_id = "map";}},
    {"after the previous point", [](auto & t) {t.points[2].header.stamp = stamp(100.25);}},
  };

  for (const auto & [expected_reason, mutate] : cases) {
    nav_msgs::msg::Trajectory message = makeLine(4);
    mutate(message);
    nav2_collision_monitor::Trajectory trajectory;
    std::string reason;
    EXPECT_FALSE(
      toTrajectory(message, tf2::Transform::getIdentity(), MAX_TIME_STEP, trajectory, reason));
    EXPECT_NE(reason.find(expected_reason), std::string::npos) << reason;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
