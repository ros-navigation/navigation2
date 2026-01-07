// Copyright (c) 2025, Polymath Robotics
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
// limitations under the License. Reserved.

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/corner_smoothing.hpp"
// #include "nav2_route/types.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(CornerSmoothingTest, test_corner_smoothing)
{
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 0;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 1;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 0.0;
  test_node3.nodeid = 2;
  test_node3.coords.x = 10.0;
  test_node3.coords.y = 10.0;

  double smoothing_radius = 2.0;

  CornerArc corner_arc(test_node1.coords, test_node2.coords, test_node3.coords, smoothing_radius);

  Coordinates start = corner_arc.getCornerStart();
  Coordinates end = corner_arc.getCornerEnd();

  EXPECT_TRUE(corner_arc.isCornerValid());
  EXPECT_EQ(start.x, 8.0);
  EXPECT_EQ(start.y, 0.0);
  EXPECT_EQ(end.x, 10.0);
  EXPECT_EQ(end.y, 2.0);
}

TEST(LargeRadiusTest, test_large_radius_smoothing)
{
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 0;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 1;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 0.0;
  test_node3.nodeid = 2;
  test_node3.coords.x = 10.0;
  test_node3.coords.y = 10.0;

  double smoothing_radius = 20.0;

  CornerArc corner_arc(test_node1.coords, test_node2.coords, test_node3.coords, smoothing_radius);

  EXPECT_FALSE(corner_arc.isCornerValid());
}

TEST(ColinearSmoothingTest, test_colinear_smoothing)
{
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 0;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 1;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 0.0;
  test_node3.nodeid = 2;
  test_node3.coords.x = 20.0;
  test_node3.coords.y = 0.0;

  double smoothing_radius = 2.0;

  CornerArc corner_arc(test_node1.coords, test_node2.coords, test_node3.coords, smoothing_radius);

  EXPECT_FALSE(corner_arc.isCornerValid());
}

TEST(DegeneratePointsTest, test_degenerate_points_smoothing)
{
  Node test_node1;
  test_node1.nodeid = 0;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;

  double smoothing_radius = 2.0;

  CornerArc corner_arc(test_node1.coords, test_node1.coords, test_node1.coords, smoothing_radius);

  EXPECT_FALSE(corner_arc.isCornerValid());
}
