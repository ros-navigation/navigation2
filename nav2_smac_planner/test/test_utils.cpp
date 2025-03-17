// Copyright (c) 2023 Open Navigation LLC
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using namespace nav2_smac_planner;  // NOLINT

TEST(transform_footprint_to_edges, test_basic)
{
  geometry_msgs::msg::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::msg::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  std::vector<geometry_msgs::msg::Point> footprint{p1, p2, p3, p4};
  std::vector<geometry_msgs::msg::Point> footprint_edges{p1, p2, p2, p3, p3, p4, p4, p1};

  // Identity pose
  geometry_msgs::msg::Pose pose0;
  auto result = transformFootprintToEdges(pose0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_EQ(p.x, q.x);
    EXPECT_EQ(p.y, q.y);
  }
}

TEST(transform_footprint_to_edges, test_transition_rotation)
{
  geometry_msgs::msg::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::msg::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  geometry_msgs::msg::Pose pose0;
  pose0.position.x = 1.0;
  pose0.position.y = 1.0;
  pose0.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI / 4.0);

  std::vector<geometry_msgs::msg::Point> footprint{p1, p2, p3, p4};

  // q1
  geometry_msgs::msg::Point q1;
  q1.x = 0.0 + pose0.position.x;
  q1.y = sqrt(2) + pose0.position.y;

  // q2
  geometry_msgs::msg::Point q2;
  q2.x = sqrt(2.0) + pose0.position.x;
  q2.y = 0.0 + pose0.position.y;

  // q3
  geometry_msgs::msg::Point q3;
  q3.x = 0.0 + pose0.position.x;
  q3.y = -sqrt(2) + pose0.position.y;

  // q4
  geometry_msgs::msg::Point q4;
  q4.x = -sqrt(2.0) + pose0.position.x;
  q4.y = 0.0 + pose0.position.y;

  std::vector<geometry_msgs::msg::Point> footprint_edges{q1, q2, q2, q3, q3, q4, q4, q1};
  auto result = transformFootprintToEdges(pose0, footprint);
  EXPECT_EQ(result.size(), 8u);

  for (size_t i = 0; i < result.size(); i++) {
    auto & p = result[i];
    auto & q = footprint_edges[i];
    EXPECT_NEAR(p.x, q.x, 1e-3);
    EXPECT_NEAR(p.y, q.y, 1e-3);
  }
}

TEST(create_marker, test_createMarker)
{
  geometry_msgs::msg::Point p1;
  p1.x = 1.0;
  p1.y = 1.0;

  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;

  geometry_msgs::msg::Point p3;
  p3.x = -1.0;
  p3.y = -1.0;

  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;
  std::vector<geometry_msgs::msg::Point> edges{p1, p2, p3, p4};

  auto marker1 = createMarker(edges, 10u, "test_frame", rclcpp::Time(0.));
  EXPECT_EQ(marker1.header.frame_id, "test_frame");
  EXPECT_EQ(rclcpp::Time(marker1.header.stamp).nanoseconds(), 0);
  EXPECT_EQ(marker1.ns, "planned_footprint");
  EXPECT_EQ(marker1.id, 10u);
  EXPECT_EQ(marker1.points.size(), 4u);

  edges.clear();
  auto marker2 = createMarker(edges, 8u, "test_frame2", rclcpp::Time(1.0, 0.0));
  EXPECT_EQ(marker2.header.frame_id, "test_frame2");
  EXPECT_EQ(rclcpp::Time(marker2.header.stamp).nanoseconds(), 1e9);
  EXPECT_EQ(marker2.id, 8u);
  EXPECT_EQ(marker2.points.size(), 0u);
}

TEST(convert_map_to_world_to_map, test_convert_map_to_world_to_map)
{
  auto costmap = nav2_costmap_2d::Costmap2D(10.0, 10.0, 0.05, 0.0, 0.0, 0);

  float mx = 200.0;
  float my = 100.0;
  geometry_msgs::msg::Pose pose = getWorldCoords(mx, my, &costmap);

  float mx1, my1;
  costmap.worldToMapContinuous(pose.position.x, pose.position.y, mx1, my1);
  EXPECT_NEAR(mx, mx1, 1e-3);
  EXPECT_NEAR(my, my1, 1e-3);
}
