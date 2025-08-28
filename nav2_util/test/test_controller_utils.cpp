// Copyright (c) 2020 Samsung Research
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

#include <cmath>
#include "nav2_util/controller_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "gtest/gtest.h"


using CircleSegmentIntersectionParam = std::tuple<
  std::pair<double, double>,
  std::pair<double, double>,
  double,
  std::pair<double, double>
>;

class CircleSegmentIntersectionTest
  : public ::testing::TestWithParam<CircleSegmentIntersectionParam>
{};

TEST_P(CircleSegmentIntersectionTest, circleSegmentIntersection)
{
  auto pair1 = std::get<0>(GetParam());
  auto pair2 = std::get<1>(GetParam());
  auto r = std::get<2>(GetParam());
  auto expected_pair = std::get<3>(GetParam());
  auto pair_to_point = [](std::pair<double, double> p) -> geometry_msgs::msg::Point {
      geometry_msgs::msg::Point point;
      point.x = p.first;
      point.y = p.second;
      point.z = 0.0;
      return point;
    };
  auto p1 = pair_to_point(pair1);
  auto p2 = pair_to_point(pair2);
  auto actual = nav2_util::circleSegmentIntersection(p1, p2, r);
  auto expected_point = pair_to_point(expected_pair);
  EXPECT_DOUBLE_EQ(actual.x, expected_point.x);
  EXPECT_DOUBLE_EQ(actual.y, expected_point.y);
  // Expect that the intersection point is actually r away from the origin
  EXPECT_DOUBLE_EQ(r, std::hypot(actual.x, actual.y));
}

INSTANTIATE_TEST_SUITE_P(
  InterpolationTest,
  CircleSegmentIntersectionTest,
  testing::Values(
    // Origin to the positive X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {2.0, 0.0},
  1.0,
  {1.0, 0.0}
},
    // Origin to the negative X axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-2.0, 0.0},
  1.0,
  {-1.0, 0.0}
},
    // Origin to the positive Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  1.0,
  {0.0, 1.0}
},
    // Origin to the negative Y axis
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, -2.0},
  1.0,
  {0.0, -1.0}
},
    // non-origin to the X axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {4.0, 0.0},
  {-1.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // non-origin to the Y axis with non-unit circle, with the second point inside
    CircleSegmentIntersectionParam{
  {0.0, 4.0},
  {0.0, -0.5},
  2.0,
  {0.0, 2.0}
},
    // origin to the positive X axis, on the circle
    CircleSegmentIntersectionParam{
  {2.0, 0.0},
  {0.0, 0.0},
  2.0,
  {2.0, 0.0}
},
    // origin to the positive Y axis, on the circle
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {0.0, 2.0},
  2.0,
  {0.0, 2.0}
},
    // origin to the upper-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, 8.0},
  5.0,
  {3.0, 4.0}
},
    // origin to the lower-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, -8.0},
  5.0,
  {-3.0, -4.0}
},
    // origin to the upper-left quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {-6.0, 8.0},
  5.0,
  {-3.0, 4.0}
},
    // origin to the lower-right quadrant (3-4-5 triangle)
    CircleSegmentIntersectionParam{
  {0.0, 0.0},
  {6.0, -8.0},
  5.0,
  {3.0, -4.0}
}
));

TEST(InterpolationUtils, lookaheadInterpolation)
{
  // test Lookahead Point Interpolation
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = nav2_util::getLookAheadPoint(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test interpolation
  dist = 3.8;
  pt = nav2_util::getLookAheadPoint(dist, path);
  EXPECT_EQ(pt.pose.position.x, 3.8);
}

TEST(InterpolationUtils, lookaheadExtrapolation)
{
  // Test extrapolation beyond goal
  double EPSILON = std::numeric_limits<float>::epsilon();

  nav_msgs::msg::Path path;
  double lookahead_dist = 10.0;
  // More than 2 poses
  path.poses.resize(4);
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = 1.0;
  path.poses[2].pose.position.x = 2.0;
  path.poses[3].pose.position.x = 3.0;
  auto pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses fwd
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[1].pose.position.x = 3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses at 45°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = 3.0;
  path.poses[1].pose.position.y = 3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(45.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(45.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at 90°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(90.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(90.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at 135°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[0].pose.position.y = 2.0;
  path.poses[1].pose.position.x = -3.0;
  path.poses[1].pose.position.y = 3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(135.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(135.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses back
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[1].pose.position.x = -3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, -10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, 0.0, EPSILON);

  // 2 poses at -135°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = -2.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = -3.0;
  path.poses[1].pose.position.y = -3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(-135.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-135.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at -90°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = -3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(-90.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-90.0 * M_PI / 180) * 10.0, EPSILON);

  // 2 poses at -45°
  path.poses.clear();
  path.poses.resize(2);
  path.poses[0].pose.position.x = 2.0;
  path.poses[0].pose.position.y = -2.0;
  path.poses[1].pose.position.x = 3.0;
  path.poses[1].pose.position.y = -3.0;
  pt = nav2_util::getLookAheadPoint(lookahead_dist, path, true);
  EXPECT_NEAR(pt.pose.position.x, cos(-45.0 * M_PI / 180) * 10.0, EPSILON);
  EXPECT_NEAR(pt.pose.position.y, sin(-45.0 * M_PI / 180) * 10.0, EPSILON);
}

TEST(UtilsTests, FindPathInversionTest)
{
  // Straight path, no inversions to be found
  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(nav2_util::findFirstPathInversion(path), 10u);

  // To short to process
  path.poses.erase(path.poses.begin(), path.poses.begin() + 7);
  EXPECT_EQ(nav2_util::findFirstPathInversion(path), 3u);

  // Has inversion at index 10, so should return 11 for the first point afterwards
  // 0 1 2 3 4 5 6 7 8 9 10 **9** 8 7 6 5 4 3 2 1
  path.poses.clear();
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(nav2_util::findFirstPathInversion(path), 11u);
}

TEST(UtilsTests, RemovePosesAfterPathInversionTest)
{
  nav_msgs::msg::Path path;
  // straight path
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(nav2_util::removePosesAfterFirstInversion(path), 0u);

  // try empty path
  path.poses.clear();
  EXPECT_EQ(nav2_util::removePosesAfterFirstInversion(path), 0u);

  // cusping path
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(nav2_util::removePosesAfterFirstInversion(path), 11u);
  // Check to see if removed
  EXPECT_EQ(path.poses.size(), 11u);
  EXPECT_EQ(path.poses.back().pose.position.x, 10);
}
