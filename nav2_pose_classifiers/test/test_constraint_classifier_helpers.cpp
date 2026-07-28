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
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_pose_classifiers/constraint_classifier.hpp"

// ---------------------------------------------------------------------------
// Test fixture — declared as friend in ConstraintClassifier.
// ---------------------------------------------------------------------------

namespace nav2_pose_classifiers
{

class ConstraintClassifierHelperTest : public ::testing::Test
{
protected:
  ConstraintClassifier classifier_;

  // Wrappers that forward to private methods
  nav2_costmap_2d::Footprint inflateFootprint(
    const nav2_costmap_2d::Footprint & fp, double delta)
  {
    return classifier_.inflateFootprint(fp, delta);
  }

  nav2_costmap_2d::Footprint orientFootprint(
    const nav2_costmap_2d::Footprint & fp,
    double x, double y, double cos_th, double sin_th)
  {
    return classifier_.orientFootprint(fp, x, y, cos_th, sin_th);
  }

  std::vector<size_t> buildOppositePairs(const nav2_costmap_2d::Footprint & fp)
  {
    return classifier_.buildOppositePairs(fp);
  }

  // Helper to build a point
  static geometry_msgs::msg::Point makePoint(double x, double y)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return p;
  }

  // Build a unit square footprint centered at origin: vertices at (±0.5, ±0.5)
  static nav2_costmap_2d::Footprint makeSquare(double half_side = 0.5)
  {
    return {
      makePoint(half_side, half_side),
      makePoint(-half_side, half_side),
      makePoint(-half_side, -half_side),
      makePoint(half_side, -half_side)
    };
  }

  // Build a rectangle footprint: front=0.45, rear=-0.65, left=0.375, right=-0.375
  static nav2_costmap_2d::Footprint makeRectangle()
  {
    return {
      makePoint(0.45, 0.375),
      makePoint(-0.65, 0.375),
      makePoint(-0.65, -0.375),
      makePoint(0.45, -0.375)
    };
  }

  // Compute the bounding box extents of a footprint
  static void getBounds(
    const nav2_costmap_2d::Footprint & fp,
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
};

// ===========================================================================
// inflateFootprint tests
// ===========================================================================

TEST_F(ConstraintClassifierHelperTest, InflateSquarePreservesVertexCount)
{
  auto fp = makeSquare(0.5);
  auto inflated = inflateFootprint(fp, 0.1);

  // Clipper with jtMiter + etClosedPolygon preserves vertex count for convex polygons
  EXPECT_EQ(inflated.size(), fp.size());
}

TEST_F(ConstraintClassifierHelperTest, InflateSquareGrowsByDelta)
{
  auto fp = makeSquare(0.5);
  const double delta = 0.2;
  auto inflated = inflateFootprint(fp, delta);

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);

  // Mitered offset on a square: each edge moves outward by delta
  // So the half-side goes from 0.5 to 0.5 + 0.2 = 0.7
  EXPECT_NEAR(max_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_x, 0.5 + delta, 0.01);
  EXPECT_NEAR(max_y, 0.5 + delta, 0.01);
  EXPECT_NEAR(-min_y, 0.5 + delta, 0.01);
}

TEST_F(ConstraintClassifierHelperTest, InflateRectangleGrowsByDelta)
{
  auto fp = makeRectangle();
  const double delta = 0.1;
  auto inflated = inflateFootprint(fp, delta);

  double min_x, max_x, min_y, max_y;
  getBounds(inflated, min_x, max_x, min_y, max_y);

  // Rectangle: front=0.45, rear=-0.65, left=0.375, right=-0.375
  // After mitered inflation by 0.1, each edge moves outward by delta
  EXPECT_NEAR(max_x, 0.45 + delta, 0.01);
  EXPECT_NEAR(min_x, -0.65 - delta, 0.01);
  EXPECT_NEAR(max_y, 0.375 + delta, 0.01);
  EXPECT_NEAR(min_y, -0.375 - delta, 0.01);
}

TEST_F(ConstraintClassifierHelperTest, InflateZeroDeltaReturnsSameFootprint)
{
  auto fp = makeSquare(0.5);
  auto inflated = inflateFootprint(fp, 0.0);

  ASSERT_EQ(inflated.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(inflated[i].x, fp[i].x, 0.001);
    EXPECT_NEAR(inflated[i].y, fp[i].y, 0.001);
  }
}

TEST_F(ConstraintClassifierHelperTest, InflateMultipleStepsGrowsMonotonically)
{
  auto fp = makeSquare(0.5);

  double prev_max_x = 0.5;
  for (int step = 1; step <= 5; ++step) {
    auto inflated = inflateFootprint(fp, step * 0.1);
    double min_x, max_x, min_y, max_y;
    getBounds(inflated, min_x, max_x, min_y, max_y);

    EXPECT_GT(max_x, prev_max_x);
    prev_max_x = max_x;
  }
}

// ===========================================================================
// orientFootprint tests
// ===========================================================================

TEST_F(ConstraintClassifierHelperTest, OrientIdentityTransform)
{
  auto fp = makeSquare(0.5);

  // Zero translation, zero rotation (cos=1, sin=0)
  auto oriented = orientFootprint(fp, 0.0, 0.0, 1.0, 0.0);

  ASSERT_EQ(oriented.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(oriented[i].x, fp[i].x, 1e-9);
    EXPECT_NEAR(oriented[i].y, fp[i].y, 1e-9);
  }
}

TEST_F(ConstraintClassifierHelperTest, OrientPureTranslation)
{
  auto fp = makeSquare(0.5);
  const double tx = 3.0, ty = -2.0;

  auto oriented = orientFootprint(fp, tx, ty, 1.0, 0.0);

  ASSERT_EQ(oriented.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(oriented[i].x, fp[i].x + tx, 1e-9);
    EXPECT_NEAR(oriented[i].y, fp[i].y + ty, 1e-9);
  }
}

TEST_F(ConstraintClassifierHelperTest, Orient90DegreeRotation)
{
  auto fp = makeSquare(0.5);
  // 90 degrees: cos=0, sin=1
  auto oriented = orientFootprint(fp, 0.0, 0.0, 0.0, 1.0);

  // (0.5, 0.5) rotated 90° -> (-0.5, 0.5)
  ASSERT_EQ(oriented.size(), fp.size());
  EXPECT_NEAR(oriented[0].x, -0.5, 1e-9);
  EXPECT_NEAR(oriented[0].y, 0.5, 1e-9);

  // (-0.5, 0.5) rotated 90° -> (-0.5, -0.5)
  EXPECT_NEAR(oriented[1].x, -0.5, 1e-9);
  EXPECT_NEAR(oriented[1].y, -0.5, 1e-9);
}

TEST_F(ConstraintClassifierHelperTest, Orient180DegreeRotation)
{
  auto fp = makeSquare(0.5);
  // 180 degrees: cos=-1, sin=0
  auto oriented = orientFootprint(fp, 0.0, 0.0, -1.0, 0.0);

  // Each point should be negated
  ASSERT_EQ(oriented.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    EXPECT_NEAR(oriented[i].x, -fp[i].x, 1e-9);
    EXPECT_NEAR(oriented[i].y, -fp[i].y, 1e-9);
  }
}

TEST_F(ConstraintClassifierHelperTest, OrientTranslationAndRotation)
{
  auto fp = makeSquare(0.5);
  const double tx = 1.0, ty = 2.0;
  const double theta = M_PI / 4.0;  // 45 degrees
  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);

  auto oriented = orientFootprint(fp, tx, ty, cos_th, sin_th);

  // Verify each point: rotated + translated
  ASSERT_EQ(oriented.size(), fp.size());
  for (size_t i = 0; i < fp.size(); ++i) {
    double expected_x = tx + (fp[i].x * cos_th - fp[i].y * sin_th);
    double expected_y = ty + (fp[i].x * sin_th + fp[i].y * cos_th);
    EXPECT_NEAR(oriented[i].x, expected_x, 1e-9);
    EXPECT_NEAR(oriented[i].y, expected_y, 1e-9);
  }
}

TEST_F(ConstraintClassifierHelperTest, OrientPreservesVertexCount)
{
  auto fp = makeRectangle();
  auto oriented = orientFootprint(fp, 5.0, 3.0, 0.707, 0.707);
  EXPECT_EQ(oriented.size(), fp.size());
}

// ===========================================================================
// buildOppositePairs tests
// ===========================================================================

TEST_F(ConstraintClassifierHelperTest, OppositePairsSquareSymmetry)
{
  // Unit square: edges 0(top), 1(left), 2(bottom), 3(right)
  // Opposite of top=bottom, left=right
  auto fp = makeSquare(0.5);
  auto opp = buildOppositePairs(fp);

  ASSERT_EQ(opp.size(), 4u);

  // Edge 0 (top) opposite should be edge 2 (bottom)
  EXPECT_EQ(opp[0], 2u);
  // Edge 2 (bottom) opposite should be edge 0 (top)
  EXPECT_EQ(opp[2], 0u);
  // Edge 1 (left) opposite should be edge 3 (right)
  EXPECT_EQ(opp[1], 3u);
  // Edge 3 (right) opposite should be edge 1 (left)
  EXPECT_EQ(opp[3], 1u);
}

TEST_F(ConstraintClassifierHelperTest, OppositePairsRectangle)
{
  // Rectangle with 4 edges — should still have proper opposite pairs
  auto fp = makeRectangle();
  auto opp = buildOppositePairs(fp);

  ASSERT_EQ(opp.size(), 4u);

  // All opposite indices should be valid (< 4)
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_LT(opp[i], 4u) << "Edge " << i << " has invalid opposite";
  }

  // Opposite of front edge should be rear edge and vice versa
  EXPECT_EQ(opp[0], 2u);  // front -> rear
  EXPECT_EQ(opp[2], 0u);  // rear -> front
  EXPECT_EQ(opp[1], 3u);  // left -> right
  EXPECT_EQ(opp[3], 1u);  // right -> left
}

TEST_F(ConstraintClassifierHelperTest, OppositePairsAllValid)
{
  // Test with a larger polygon (hexagon-like)
  nav2_costmap_2d::Footprint hex;
  for (int i = 0; i < 6; ++i) {
    double angle = i * M_PI / 3.0;
    hex.push_back(makePoint(std::cos(angle), std::sin(angle)));
  }

  auto opp = buildOppositePairs(hex);

  ASSERT_EQ(opp.size(), 6u);

  // All indices should be valid
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_LT(opp[i], 6u) << "Edge " << i << " has invalid opposite";
  }

  // For a regular hexagon, opposite of edge i should be edge (i+3)%6
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(opp[i], (i + 3) % 6)
      << "Edge " << i << " opposite expected " << (i + 3) % 6 << " got " << opp[i];
  }
}

TEST_F(ConstraintClassifierHelperTest, OppositePairsSymmetric)
{
  // For any convex polygon, if opp[i] = j, then opp[j] = i
  auto fp = makeSquare(0.5);
  auto opp = buildOppositePairs(fp);

  for (size_t i = 0; i < opp.size(); ++i) {
    if (opp[i] < opp.size()) {
      EXPECT_EQ(opp[opp[i]], i)
        << "Symmetry broken: opp[" << i << "]=" << opp[i]
        << " but opp[" << opp[i] << "]=" << opp[opp[i]];
    }
  }
}

TEST_F(ConstraintClassifierHelperTest, OppositePairsNeverSelf)
{
  auto fp = makeRectangle();
  auto opp = buildOppositePairs(fp);

  for (size_t i = 0; i < opp.size(); ++i) {
    EXPECT_NE(opp[i], i) << "Edge " << i << " maps to itself";
  }
}

// ===========================================================================
// Combined tests — inflate then orient
// ===========================================================================

TEST_F(ConstraintClassifierHelperTest, InflateThenOrientPreservesVertexCount)
{
  auto fp = makeRectangle();
  auto inflated = inflateFootprint(fp, 0.15);
  auto oriented = orientFootprint(inflated, 2.0, 3.0, 0.866, 0.5);

  EXPECT_EQ(oriented.size(), fp.size());
}

TEST_F(ConstraintClassifierHelperTest, InflateThenOrientCentroidAtPose)
{
  auto fp = makeSquare(0.5);
  auto inflated = inflateFootprint(fp, 0.1);

  const double tx = 5.0, ty = -3.0;
  auto oriented = orientFootprint(inflated, tx, ty, 1.0, 0.0);

  // Centroid of the oriented footprint should be near (tx, ty)
  double cx = 0.0, cy = 0.0;
  for (const auto & pt : oriented) {
    cx += pt.x;
    cy += pt.y;
  }
  cx /= oriented.size();
  cy /= oriented.size();

  EXPECT_NEAR(cx, tx, 0.01);
  EXPECT_NEAR(cy, ty, 0.01);
}

}  // namespace nav2_pose_classifiers

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
