// Copyright (c) 2020 Shivang Patel
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

#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

// Test basic footprint collision checking with no obstacles
// Verifies that a diamond-shaped footprint placed on an empty costmap returns zero cost
TEST(collision_footprint, test_basic)
{
  // Create a 100x100 costmap with 0.1m resolution, all cells initialized to cost 0
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a diamond-shaped footprint
  geometry_msgs::msg::Point p1;
  p1.x = -0.5;  // Left vertex
  p1.y = 0.0;
  geometry_msgs::msg::Point p2;
  p2.x = 0.0;   // Top vertex
  p2.y = 0.5;
  geometry_msgs::msg::Point p3;
  p3.x = 0.5;   // Right vertex
  p3.y = 0.0;
  geometry_msgs::msg::Point p4;
  p4.x = 0.0;   // Bottom vertex
  p4.y = -0.5;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Check footprint cost at position (5.0, 5.0) with no rotation
  // Should return 0 since no obstacles are present
  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);

  EXPECT_NEAR(value, 0.0, 0.001);
}

// Test point cost functionality
// Verifies that the pointCost method correctly returns the cost value at a specific grid cell
TEST(collision_footprint, test_point_cost)
{
  // Create empty costmap with all cells at cost 0
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test point cost at grid cell (50, 50) - should be 0 for empty costmap
  auto value = collision_checker.pointCost(50, 50);

  EXPECT_NEAR(value, 0.0, 0.001);
}

// Test world-to-map coordinate conversion and point cost retrieval
// Verifies that world coordinates are correctly converted to map coordinates
// and that costs can be retrieved and set at those locations
TEST(collision_footprint, test_world_to_map)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  unsigned int x, y;

  // Convert world coordinates (1.0, 1.0) to map coordinates
  // With 0.1m resolution and origin at (0,0), this should map to grid cell (10, 10)
  collision_checker.worldToMap(1.0, 1.0, x, y);

  // Check that the cost at the converted coordinates is 0 (empty space)
  auto value = collision_checker.pointCost(x, y);
  EXPECT_NEAR(value, 0.0, 0.001);

  // Set a cost of 200 at grid cell (50, 50) and verify it can be retrieved
  costmap_->setCost(50, 50, 200);
  collision_checker.worldToMap(5.0, 5.0, x, y);  // World (5.0, 5.0) -> Grid (50, 50)

  EXPECT_NEAR(collision_checker.pointCost(x, y), 200.0, 0.001);
}

// Test footprint collision detection when robot moves into obstacle areas
// Creates a costmap with obstacles everywhere except a safe zone, then tests
// footprint placement at different positions to verify collision detection
TEST(collision_footprint, test_footprint_at_pose_with_movement)
{
  // Create costmap with all cells set to cost 254 (high obstacle cost)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 254);

  // Create a safe zone (cost 0) in the center of the map from grid (40,40) to (60,60)
  // This corresponds to world coordinates roughly (4.0,4.0) to (6.0,6.0)
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap_->setCost(i, j, 0);
    }
  }

  // Create a 2x2 meter square footprint
  geometry_msgs::msg::Point p1;
  p1.x = -1.0;  // Bottom-left
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;   // Bottom-right
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;   // Top-right
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;  // Top-left
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test footprint at center of safe zone - should have zero cost
  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);
  EXPECT_NEAR(value, 0.0, 0.001);

  // Test footprint moved slightly up - should hit obstacle area (cost 254)
  auto up_value = collision_checker.footprintCostAtPose(5.0, 4.9, 0.0, footprint);
  EXPECT_NEAR(up_value, 254.0, 0.001);

  // Test footprint moved slightly down - should hit obstacle area (cost 254)
  auto down_value = collision_checker.footprintCostAtPose(5.0, 5.2, 0.0, footprint);
  EXPECT_NEAR(down_value, 254.0, 0.001);
}

// Test point and line cost detection with strategically placed obstacles
// Places obstacles at specific locations and verifies they are detected when the
// footprint perimeter intersects them during different robot movements
TEST(collision_footprint, test_point_and_line_cost)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.10000, 0, 0.0, 0.0);

  // Place obstacles at strategic locations:
  costmap_->setCost(62, 50, 254);  // Obstacle to the right of center
  costmap_->setCost(39, 60, 254);  // Obstacle to the left and up from center

  // Create a 2x2 meter square footprint
  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test footprint at center position - should not hit obstacles
  auto value = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint);
  EXPECT_NEAR(value, 0.0, 0.001);

  // Move footprint left - should detect obstacle at (39, 60)
  auto left_value = collision_checker.footprintCostAtPose(4.9, 5.0, 0.0, footprint);
  EXPECT_NEAR(left_value, 254.0, 0.001);

  // Move footprint right - should detect obstacle at (62, 50)
  auto right_value = collision_checker.footprintCostAtPose(5.2, 5.0, 0.0, footprint);
  EXPECT_NEAR(right_value, 254.0, 0.001);
}

// Test utility function for footprints with insufficient points
// Verifies that distance calculation functions handle edge cases correctly
// when footprint has fewer than 3 points (minimum for a valid polygon)
TEST(collision_footprint, not_enough_points)
{
  geometry_msgs::msg::Point p1;
  p1.x = 2.0;
  p1.y = 2.0;

  geometry_msgs::msg::Point p2;
  p2.x = -2.0;
  p2.y = -2.0;

  // Create footprint with only 2 points (insufficient for polygon)
  std::vector<geometry_msgs::msg::Point> footprint = {p1, p2};
  double min_dist = 0.0;
  double max_dist = 0.0;

  // calculateMinAndMaxDistances should handle this gracefully
  std::tie(min_dist, max_dist) = nav2_costmap_2d::calculateMinAndMaxDistances(footprint);
  EXPECT_EQ(min_dist, std::numeric_limits<double>::max());
  EXPECT_EQ(max_dist, 0.0f);
}

// Test conversion from Point to Point32 message types
// Verifies that geometry message type conversions preserve coordinate values
TEST(collision_footprint, to_point_32) {
  geometry_msgs::msg::Point p;
  p.x = 123.0;
  p.y = 456.0;
  p.z = 789.0;

  // Convert to Point32 format
  geometry_msgs::msg::Point32 p32;
  p32 = nav2_costmap_2d::toPoint32(p);
  
  // Verify all coordinates are preserved
  EXPECT_NEAR(p.x, p32.x, 1e-5);
  EXPECT_NEAR(p.y, p32.y, 1e-5);
  EXPECT_NEAR(p.z, p32.z, 1e-5);
}

// Test conversion from vector of Points to Polygon message type
// Verifies that footprint data can be converted to ROS message format
TEST(collision_footprint, to_polygon) {
  geometry_msgs::msg::Point p1;
  p1.x = 1.2;
  p1.y = 3.4;
  p1.z = 5.1;

  geometry_msgs::msg::Point p2;
  p2.x = -5.6;
  p2.y = -7.8;
  p2.z = -9.1;
  std::vector<geometry_msgs::msg::Point> pts = {p1, p2};

  // Convert to Polygon message format
  geometry_msgs::msg::Polygon poly;
  poly = nav2_costmap_2d::toPolygon(pts);

  // Verify structure and coordinate preservation
  EXPECT_EQ(2u, sizeof(poly.points) / sizeof(poly.points[0]));
  EXPECT_NEAR(poly.points[0].x, p1.x, 1e-5);
  EXPECT_NEAR(poly.points[0].y, p1.y, 1e-5);
  EXPECT_NEAR(poly.points[0].z, p1.z, 1e-5);
  EXPECT_NEAR(poly.points[1].x, p2.x, 1e-5);
  EXPECT_NEAR(poly.points[1].y, p2.y, 1e-5);
  EXPECT_NEAR(poly.points[1].z, p2.z, 1e-5);
}

// Test successful parsing of footprint from string representation
// Verifies that string-based footprint configuration works correctly
TEST(collision_footprint, make_footprint_from_string) {
  std::vector<geometry_msgs::msg::Point> footprint;
  
  // Parse a valid footprint string with scientific notation
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2]]", footprint);
  
  EXPECT_EQ(result, true);
  EXPECT_EQ(4u, footprint.size());
  
  // Verify all coordinate values are parsed correctly
  EXPECT_NEAR(footprint[0].x, 1.0, 1e-5);
  EXPECT_NEAR(footprint[0].y, 2.2, 1e-5);
  EXPECT_NEAR(footprint[1].x, 0.3, 1e-5);
  EXPECT_NEAR(footprint[1].y, -4e4, 1e-5);
  EXPECT_NEAR(footprint[2].x, -0.3, 1e-5);
  EXPECT_NEAR(footprint[2].y, -4e4, 1e-5);
  EXPECT_NEAR(footprint[3].x, -1.0, 1e-5);
  EXPECT_NEAR(footprint[3].y, 2.2, 1e-5);
}

// Test error handling for malformed footprint strings
// Verifies that parsing fails gracefully with invalid input
TEST(collision_footprint, make_footprint_from_string_parse_error) {
  std::vector<geometry_msgs::msg::Point> footprint;
  
  // Try to parse a malformed string (missing closing bracket)
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[bad_string", footprint);
  EXPECT_EQ(result, false);
}

// Test error handling for footprints with too few points
// Verifies that polygons with insufficient vertices are rejected
TEST(collision_footprint, make_footprint_from_string_two_points_error) {
  std::vector<geometry_msgs::msg::Point> footprint;
  
  // Try to parse footprint with only 2 points (need at least 3 for polygon)
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4]", footprint);
  EXPECT_EQ(result, false);
}

// Test error handling for points with wrong number of coordinates
// Verifies that points with extra coordinates are rejected
TEST(collision_footprint, make_footprint_from_string_not_pairs) {
  std::vector<geometry_msgs::msg::Point> footprint;
  
  // Try to parse footprint with a point having 3 coordinates (should be 2)
  bool result = nav2_costmap_2d::makeFootprintFromString(
    "[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2, 5.6]]", footprint);
  EXPECT_EQ(result, false);
}

// Test full area checking functionality when no obstacles are present
// Verifies that both perimeter-only and full-area checking return the same result
// when the footprint area is completely free of obstacles
TEST(collision_footprint, test_full_area_check_no_collision) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a 2x2 meter rectangular footprint
  geometry_msgs::msg::Point p1;
  p1.x = -1.0;  // Bottom-left
  p1.y = -1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;   // Bottom-right
  p2.y = -1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;   // Top-right
  p3.y = 1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;  // Top-left
  p4.y = 1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test perimeter only check at a valid position (5.0, 5.0) within the 10x10m costmap
  auto perimeter_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, false);
  EXPECT_NEAR(perimeter_cost, 0.0, 0.001);

  // Test full area check at the same position
  auto full_area_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, true);
  EXPECT_NEAR(full_area_cost, 0.0, 0.001);

  // Both should be the same when no obstacles present
  EXPECT_NEAR(perimeter_cost, full_area_cost, 0.001);
}

// Test that full area checking detects obstacles inside the footprint
// Obstacles that are inside the footprint but not on the perimeter
// should only be detected by full area checking
TEST(collision_footprint, test_full_area_check_interior_obstacle) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a 2x2 meter rectangular footprint that will be centered at (5.0, 5.0)
  // This maps to grid coordinates around (50, 50) with the 0.1m resolution
  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = -1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = -1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = 1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = 1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Place an obstacle in the interior of the footprint (not on perimeter)
  // Grid cell (50, 50) corresponds to world coordinates (5.0, 5.0) - center of footprint
  costmap_->setCost(50, 50, 200);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test perimeter only check - should NOT detect interior obstacle
  auto perimeter_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, false);
  EXPECT_NEAR(perimeter_cost, 0.0, 0.001);

  // Test full area check - SHOULD detect interior obstacle
  auto full_area_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, true);
  EXPECT_NEAR(full_area_cost, 200.0, 0.001);
}

// Test full area checking with lethal obstacles for early termination
// Verifies that when a lethal obstacle is found during full area checking,
// the algorithm returns immediately with lethal cost (performance optimization)
TEST(collision_footprint, test_full_area_check_lethal_obstacle) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a triangular footprint (simpler shape for testing)
  geometry_msgs::msg::Point p1;
  p1.x = 0.0;   // Top vertex
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = -1.0;  // Bottom-left vertex
  p2.y = -1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;   // Bottom-right vertex
  p3.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3};

  // Place a lethal obstacle in the interior of the triangle
  // Grid cell (50, 49) should be inside the triangle when centered at (5.0, 5.0)
  costmap_->setCost(50, 49, nav2_costmap_2d::LETHAL_OBSTACLE);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test full area check with lethal obstacle - should return lethal cost immediately
  auto full_area_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, true);
  EXPECT_NEAR(full_area_cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE), 0.001);
}

// Test full area checking with rotated footprints
// Verifies that the full area checking works correctly when the footprint
// is rotated, ensuring the OpenCV polygon rasterization handles rotation properly
TEST(collision_footprint, test_full_area_check_rotated_footprint) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a rectangular footprint that's taller than it is wide
  // This will test rotation effects more clearly
  geometry_msgs::msg::Point p1;
  p1.x = -0.5;  // Left edge
  p1.y = -1.5;  // Bottom edge (tall rectangle)
  geometry_msgs::msg::Point p2;
  p2.x = 0.5;   // Right edge
  p2.y = -1.5;
  geometry_msgs::msg::Point p3;
  p3.x = 0.5;
  p3.y = 1.5;   // Top edge
  geometry_msgs::msg::Point p4;
  p4.x = -0.5;
  p4.y = 1.5;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Place obstacle where it would be inside the footprint when rotated 90 degrees
  // Grid cell (52, 50) should be inside the rotated footprint
  costmap_->setCost(52, 50, 150);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test with 90-degree rotation (M_PI/2 radians)
  // The tall rectangle becomes a wide rectangle, covering the obstacle
  auto rotated_cost = collision_checker.footprintCostAtPose(5.0, 5.0, M_PI/2, footprint, true);
  EXPECT_GT(rotated_cost, 0.0);  // Should detect the obstacle
}

// Test the lineCost function directly
// Verifies that line cost calculation works correctly for line segments,
// including early termination when lethal obstacles are encountered
TEST(collision_footprint, test_line_cost_function) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test line cost with no obstacles - should return 0
  auto line_cost = collision_checker.lineCost(10, 20, 10, 20);
  EXPECT_NEAR(line_cost, 0.0, 0.001);

  // Add obstacle in the line path from (10,10) to (20,20)
  costmap_->setCost(15, 15, 100);
  line_cost = collision_checker.lineCost(10, 20, 10, 20);
  EXPECT_NEAR(line_cost, 100.0, 0.001);

  // Test with lethal obstacle - should return immediately without checking rest of line
  costmap_->setCost(12, 12, nav2_costmap_2d::LETHAL_OBSTACLE);
  line_cost = collision_checker.lineCost(10, 20, 10, 20);
  EXPECT_NEAR(line_cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE), 0.001);
}

// Test handling of footprints that extend outside the costmap boundaries
// Verifies that out-of-bounds footprint coordinates are handled correctly
// and return lethal cost (indicating collision/invalid placement)
TEST(collision_footprint, test_out_of_bounds_footprint) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a large footprint that will extend beyond map boundaries
  geometry_msgs::msg::Point p1;
  p1.x = -2.0;  // Will extend to negative world coordinates
  p1.y = -2.0;
  geometry_msgs::msg::Point p2;
  p2.x = 2.0;
  p2.y = -2.0;
  geometry_msgs::msg::Point p3;
  p3.x = 2.0;
  p3.y = 2.0;
  geometry_msgs::msg::Point p4;
  p4.x = -2.0;
  p4.y = 2.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Place footprint so part of it extends outside the map (negative world coordinates)
  // With costmap starting at origin (0,0), placing at (-1.0, -1.0) will put parts out of bounds
  auto out_of_bounds_cost = collision_checker.footprintCostAtPose(-1.0, -1.0, 0.0, footprint, true);
  EXPECT_EQ(out_of_bounds_cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
}

// Test the setCostmap and getCostmap functionality
// Verifies that the collision checker can switch between different costmaps
// and that cost queries are performed on the correct costmap
TEST(collision_footprint, test_set_costmap_function) {
  // Create two different costmaps with different sizes
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap1 =
    std::make_shared<nav2_costmap_2d::Costmap2D>(50, 50, 0.1, 0, 0, 0);
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap2 =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Set different costs at the same grid location in both costmaps
  costmap1->setCost(25, 25, 100);
  costmap2->setCost(25, 25, 200);

  // Initialize collision checker with first costmap
  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap1);

  // Test with first costmap - should return cost 100
  auto cost1 = collision_checker.pointCost(25, 25);
  EXPECT_NEAR(cost1, 100.0, 0.001);

  // Switch to second costmap using setCostmap
  collision_checker.setCostmap(costmap2);
  auto cost2 = collision_checker.pointCost(25, 25);
  EXPECT_NEAR(cost2, 200.0, 0.001);

  // Verify getCostmap function returns the correct costmap reference
  auto retrieved_costmap = collision_checker.getCostmap();
  EXPECT_EQ(retrieved_costmap, costmap2);
}

// Test full area checking with complex polygons (non-rectangular shapes)
// Verifies that the OpenCV polygon rasterization works correctly for
// more complex shapes like pentagons, ensuring the implementation is robust
TEST(collision_footprint, test_complex_polygon_full_area) {
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ =
    std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, 0, 0, 0);

  // Create a complex polygon (pentagon) to test non-trivial polygon rasterization
  geometry_msgs::msg::Point p1;
  p1.x = 0.0;   // Top vertex
  p1.y = 1.5;
  geometry_msgs::msg::Point p2;
  p2.x = 1.4;   // Top-right vertex
  p2.y = 0.5;
  geometry_msgs::msg::Point p3;
  p3.x = 0.9;   // Bottom-right vertex
  p3.y = -1.2;
  geometry_msgs::msg::Point p4;
  p4.x = -0.9;  // Bottom-left vertex
  p4.y = -1.2;
  geometry_msgs::msg::Point p5;
  p5.x = -1.4;  // Top-left vertex
  p5.y = 0.5;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4, p5};

  // Place obstacle in what should be the interior of the pentagon
  costmap_->setCost(50, 50, 180);

  nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>
  collision_checker(costmap_);

  // Test that full area check detects interior obstacle in complex polygon
  auto perimeter_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, false);
  auto full_area_cost = collision_checker.footprintCostAtPose(5.0, 5.0, 0.0, footprint, true);

  // Perimeter should not detect interior obstacle
  EXPECT_LT(perimeter_cost, 180.0);
  // Full area should detect it (demonstrating the value of full area checking)
  EXPECT_NEAR(full_area_cost, 180.0, 0.001);
}
