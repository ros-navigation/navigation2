// Copyright (c) 2026 Origin
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
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/utils/footprint_utils.hpp"

using Footprint = std::vector<geometry_msgs::msg::Point>;
using nav2_behavior_tree::utils::buildOppositePairs;
using nav2_behavior_tree::utils::footprintToRobotFrame;
using nav2_behavior_tree::utils::inflateFootprint;

static geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

// Square footprint centered at the origin
static Footprint makeSquare(double half_side)
{
  return {
    makePoint(half_side, half_side),
    makePoint(-half_side, half_side),
    makePoint(-half_side, -half_side),
    makePoint(half_side, -half_side)
  };
}

// Rectangle footprint: front=0.45, rear=-0.65, left=0.375, right=-0.375
static Footprint makeRectangle()
{
  return {
    makePoint(0.45, 0.375),
    makePoint(-0.65, 0.375),
    makePoint(-0.65, -0.375),
    makePoint(0.45, -0.375)
  };
}

// Asymmetric clockwise pentagon, whose edges Clipper renumbered
static Footprint makeClockwisePentagon()
{
  return {
    makePoint(0.45, 0.05),
    makePoint(0.30, -0.30),
    makePoint(-0.35, -0.20),
    makePoint(-0.25, 0.30),
    makePoint(0.20, 0.28)
  };
}

static void getBounds(
  const Footprint & fp,
  double & min_x, double & max_x, double & min_y, double & max_y)
{
  min_x = min_y = 1e18;
  max_x = max_y = -1e18;
  for (const auto & pt : fp) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }
}

// Absolute polygon area
static double area(const Footprint & fp)
{
  double sum = 0.0;
  for (size_t i = 0; i < fp.size(); ++i) {
    const auto & a = fp[i];
    const auto & b = fp[(i + 1) % fp.size()];
    sum += a.x * b.y - b.x * a.y;
  }
  return std::fabs(sum) * 0.5;
}

// TF buffer with the robot at (2, 1) facing +y in the map frame
static std::shared_ptr<tf2_ros::Buffer> makeBuffer()
{
  auto tf = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = 1.0;
  transform.transform.rotation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);
  tf->setTransform(transform, "test", true);
  return tf;
}

// Footprint published the way a costmap does, placed at the robot pose in the map frame
static geometry_msgs::msg::PolygonStamped placeAtRobot(const Footprint & fp)
{
  Footprint placed;
  nav2_costmap_2d::transformFootprint(2.0, 1.0, M_PI_2, fp, placed);
  geometry_msgs::msg::PolygonStamped msg;
  msg.header.frame_id = "map";
  msg.polygon = nav2_costmap_2d::toPolygon(placed);
  return msg;
}

// Expect edge i of the inflated footprint to be parallel to edge i, delta away from it
static void expectEdgesOffsetBy(const Footprint & fp, const Footprint & inflated, double delta)
{
  ASSERT_EQ(inflated.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    const size_t j = (i + 1) % fp.size();
    const double ex = fp[j].x - fp[i].x;
    const double ey = fp[j].y - fp[i].y;
    const double ix = inflated[j].x - inflated[i].x;
    const double iy = inflated[j].y - inflated[i].y;
    EXPECT_NEAR(ex * iy - ey * ix, 0.0, 1e-9) << "edge " << i;
    const double dist =
      std::fabs((inflated[i].x - fp[i].x) * ey - (inflated[i].y - fp[i].y) * ex) /
      std::hypot(ex, ey);
    EXPECT_NEAR(dist, delta, 1e-9) << "edge " << i;
  }
}

TEST(FootprintUtilsTest, InflateSquareGrowsByDelta)
{
  auto fp = makeSquare(0.5);
  const double delta = 0.2;
  auto inflated = inflateFootprint(fp, delta);

  // Mitered offset keeps the vertex count for convex polygons, which isConfined() relies on
  EXPECT_EQ(inflated.size(), fp.size());

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);
  EXPECT_NEAR(max_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(max_y, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_y, 0.5 + delta, 0.01);
}

TEST(FootprintUtilsTest, InflateRectangleGrowsByDelta)
{
  auto fp = makeRectangle();
  const double delta = 0.1;
  auto inflated = inflateFootprint(fp, delta);

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);
  EXPECT_NEAR(max_x, 0.45 + delta, 0.01);
  EXPECT_NEAR(min_x, -0.65 - delta, 0.01);
  EXPECT_NEAR(max_y, 0.375 + delta, 0.01);
  EXPECT_NEAR(min_y, -0.375 - delta, 0.01);
}

TEST(FootprintUtilsTest, InflateKeepsEdgeOrderClockwise)
{
  const double delta = 0.05;
  const auto clockwise = makeClockwisePentagon();
  const Footprint counter_clockwise(clockwise.rbegin(), clockwise.rend());

  for (const auto & fp : {clockwise, counter_clockwise}) {
    const auto inflated = inflateFootprint(fp, delta);
    ASSERT_EQ(inflated.size(), fp.size());
    EXPECT_GT(area(inflated), area(fp));
    expectEdgesOffsetBy(fp, inflated, delta);
  }
}

TEST(FootprintUtilsTest, InflateRejectsConcaveAndSharpCorners)
{
  // Notch at the rear
  const Footprint concave = {
    makePoint(0.5, 0.3), makePoint(-0.5, 0.5), makePoint(-0.4, 0.0),
    makePoint(-0.5, -0.5), makePoint(0.5, -0.3)};
  EXPECT_TRUE(inflateFootprint(concave, 0.05).empty());

  // 10 degree nose, sharper than the 60 degree miter limit
  const Footprint sharp = {makePoint(1.0, 0.0), makePoint(-0.1, 0.1), makePoint(-0.1, -0.1)};
  EXPECT_TRUE(inflateFootprint(sharp, 0.05).empty());

  const Footprint degenerate = {makePoint(0.3, 0.3), makePoint(-0.3, -0.3)};
  EXPECT_TRUE(inflateFootprint(degenerate, 0.05).empty());
}

TEST(FootprintUtilsTest, InflateRejectsSharpConcaveCorner)
{
  // Narrow slit at the rear, an 8 degree concave corner between right angled corners
  const Footprint slit = {
    makePoint(0.5, 0.5), makePoint(-0.5, 0.5), makePoint(-0.5, 0.05), makePoint(0.2, 0.0),
    makePoint(-0.5, -0.05), makePoint(-0.5, -0.5), makePoint(0.5, -0.5)};
  EXPECT_TRUE(inflateFootprint(slit, 0.05).empty());
}

TEST(FootprintUtilsTest, InflateRejectsRepeatedPoints)
{
  auto repeated = makeRectangle();
  repeated.insert(repeated.begin() + 1, repeated[1]);
  EXPECT_TRUE(inflateFootprint(repeated, 0.05).empty());

  // Closed by repeating the first point
  auto closed = makeRectangle();
  closed.push_back(closed.front());
  EXPECT_TRUE(inflateFootprint(closed, 0.05).empty());

  // Repeating the concave corner of a notch hid it from the concave check
  const Footprint notch = {
    makePoint(0.5, 0.3), makePoint(-0.5, 0.5), makePoint(-0.4, 0.0), makePoint(-0.4, 0.0),
    makePoint(-0.5, -0.5), makePoint(0.5, -0.3)};
  EXPECT_TRUE(inflateFootprint(notch, 0.05).empty());
}

TEST(FootprintUtilsTest, InflateRejectsSelfIntersecting)
{
  // 7 point star turning the same way at every corner, but winding around twice
  Footprint star;
  for (int i = 0; i < 7; ++i) {
    const double angle = i * 4.0 * M_PI / 7.0;
    star.push_back(makePoint(0.4 * std::cos(angle), 0.4 * std::sin(angle)));
  }
  EXPECT_TRUE(inflateFootprint(star, 0.05).empty());
}

TEST(FootprintUtilsTest, InflateKeepsCollinearVertices)
{
  // Rectangle with a vertex in the middle of each long side
  const Footprint fp = {
    makePoint(0.45, 0.375), makePoint(-0.1, 0.375), makePoint(-0.65, 0.375),
    makePoint(-0.65, -0.375), makePoint(-0.1, -0.375), makePoint(0.45, -0.375)};
  const double delta = 0.05;
  expectEdgesOffsetBy(fp, inflateFootprint(fp, delta), delta);
}

TEST(FootprintUtilsTest, OppositePairsRectangle)
{
  auto opp = buildOppositePairs(makeRectangle());

  // Front <-> rear and left <-> right
  ASSERT_EQ(opp.size(), 4u);
  EXPECT_EQ(opp[0], 2u);
  EXPECT_EQ(opp[2], 0u);
  EXPECT_EQ(opp[1], 3u);
  EXPECT_EQ(opp[3], 1u);
}

TEST(FootprintUtilsTest, OppositePairsHexagon)
{
  Footprint hex;
  for (int i = 0; i < 6; ++i) {
    const double angle = i * M_PI / 3.0;
    hex.push_back(makePoint(std::cos(angle), std::sin(angle)));
  }

  // For a regular hexagon the opposite of edge i is edge (i + 3) % 6
  auto opp = buildOppositePairs(hex);
  ASSERT_EQ(opp.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(opp[i], (i + 3) % 6) << "edge " << i;
  }
}

TEST(FootprintUtilsTest, FootprintToRobotFrame)
{
  auto tf = makeBuffer();
  const auto fp = makeRectangle();

  Footprint robot_frame;
  ASSERT_TRUE(footprintToRobotFrame(placeAtRobot(fp), *tf, "base_link", 0.1, robot_frame));
  ASSERT_EQ(robot_frame.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    // Published points are float32
    EXPECT_NEAR(robot_frame[i].x, fp[i].x, 1e-5) << "vertex " << i;
    EXPECT_NEAR(robot_frame[i].y, fp[i].y, 1e-5) << "vertex " << i;
  }
}

TEST(FootprintUtilsTest, FootprintToRobotFrameFailsWithoutTf)
{
  auto tf = makeBuffer();
  auto msg = placeAtRobot(makeRectangle());
  msg.header.frame_id = "odom";  // no odom -> base_link transform

  Footprint robot_frame;
  EXPECT_FALSE(footprintToRobotFrame(msg, *tf, "base_link", 0.0, robot_frame));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
