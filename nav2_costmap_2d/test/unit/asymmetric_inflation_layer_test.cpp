// Copyright (c) 2026 Marc Blöchlinger
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

// Unit tests for AsymmetricInflationLayer — pure-algorithm coverage.
//
// Tests exercise cost_lut_disfavored_, buildPathSpatialHash, and
// computeObstacleSide via a test-subclass that exposes protected members
// and methods without requiring a full LayeredCostmap / LifecycleNode stack.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"

namespace nav2_costmap_2d
{

class TestableAsymmetricInflationLayer : public AsymmetricInflationLayer
{
public:
  using AsymmetricInflationLayer::buildPathSpatialHash;
  using AsymmetricInflationLayer::computeObstacleSide;
  using AsymmetricInflationLayer::cost_lut_disfavored_;

  void setResolution(double r) {resolution_ = r;}
  void setInscribedRadius(double r) {inscribed_radius_ = r;}
  void setInflationRadius(double r) {inflation_radius_ = r;}
  void setCostScalingFactorLeft(double c)
  {
    cost_scaling_factor_left_ = c;
    cost_scaling_factor_ = std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
  }
  void setCostScalingFactorRight(double c)
  {
    cost_scaling_factor_right_ = c;
    cost_scaling_factor_ = std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
  }
  void setCellInflationRadius(unsigned int r) {cell_inflation_radius_ = r;}
  void rebuildCaches() {computeAsymmetricCaches();}
};

}  // namespace nav2_costmap_2d

using nav2_costmap_2d::TestableAsymmetricInflationLayer;
using nav2_costmap_2d::Side;

// ============================================================
// DisfavoredLutTest — LUT mathematical correctness
// ============================================================

// LUT must be built with c_side (the smaller scaling factor), not c_max.
TEST(DisfavoredLutTest, lut_uses_c_side_scaling_factor)
{
  auto layer = std::make_unique<TestableAsymmetricInflationLayer>();
  layer->setResolution(0.1);
  layer->setInscribedRadius(0.3);
  layer->setInflationRadius(2.0);
  layer->setCellInflationRadius(20);
  // c_left=1, c_right=7 → c_side=1 (left is disfavored)
  layer->setCostScalingFactorLeft(1.0);
  layer->setCostScalingFactorRight(7.0);
  layer->rebuildCaches();

  ASSERT_FALSE(layer->cost_lut_disfavored_.empty());

  const double resolution = 0.1;
  const double inscribed_radius = 0.3;
  const double c_side = 1.0;
  const int lut_precision = 100;

  // Decay region: spot-check several distances beyond the inscribed radius.
  for (int d_scaled = lut_precision * 4; d_scaled <= lut_precision * 19;
    d_scaled += lut_precision)
  {
    double distance = static_cast<double>(d_scaled) / lut_precision;
    double factor = exp(-c_side * (distance * resolution - inscribed_radius));
    unsigned char expected =
      static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);

    EXPECT_EQ(layer->cost_lut_disfavored_[d_scaled], expected)
      << "LUT mismatch at d_scaled=" << d_scaled;
  }

  // Inscribed region: every cell in [1, 3 cells) must equal INSCRIBED_INFLATED_OBSTACLE.
  for (int d_scaled = 1; d_scaled < lut_precision * 3; ++d_scaled) {
    EXPECT_EQ(
      layer->cost_lut_disfavored_[d_scaled],
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "Expected INSCRIBED at d_scaled=" << d_scaled;
  }

  // Distance 0 must be LETHAL_OBSTACLE.
  EXPECT_EQ(layer->cost_lut_disfavored_[0], nav2_costmap_2d::LETHAL_OBSTACLE);
}

// Swapping left/right with the same absolute values must produce an identical LUT
// because c_side = min(c_left, c_right) is unchanged.
TEST(DisfavoredLutTest, lut_reflects_updated_c_side)
{
  auto layer = std::make_unique<TestableAsymmetricInflationLayer>();
  layer->setResolution(0.1);
  layer->setInscribedRadius(0.3);
  layer->setCellInflationRadius(10);
  layer->setCostScalingFactorLeft(1.0);
  layer->setCostScalingFactorRight(7.0);
  layer->rebuildCaches();

  std::vector<unsigned char> lut_c1 = layer->cost_lut_disfavored_;

  // Swap sides: c_side = min(7, 1) = 1 — same as before.
  layer->setCostScalingFactorLeft(7.0);
  layer->setCostScalingFactorRight(1.0);
  layer->rebuildCaches();

  EXPECT_EQ(lut_c1, layer->cost_lut_disfavored_);
}

// ============================================================
// CacheEdgeCasesTest — computeAsymmetricCaches edge conditions
// ============================================================

// When cell_inflation_radius == 0, computeAsymmetricCaches must return without
// allocating the LUT.
TEST(CacheEdgeCasesTest, zero_cell_radius_produces_empty_lut)
{
  auto layer = std::make_unique<TestableAsymmetricInflationLayer>();
  layer->setCellInflationRadius(0);
  layer->rebuildCaches();
  EXPECT_TRUE(layer->cost_lut_disfavored_.empty());
}

// When c_left == c_right, c_side = that value; the LUT must use it for decay.
TEST(CacheEdgeCasesTest, equal_factors_use_that_value_in_lut)
{
  auto layer = std::make_unique<TestableAsymmetricInflationLayer>();
  layer->setResolution(0.1);
  layer->setInscribedRadius(0.3);
  layer->setCellInflationRadius(10);
  layer->setCostScalingFactorLeft(5.0);
  layer->setCostScalingFactorRight(5.0);
  layer->rebuildCaches();

  ASSERT_FALSE(layer->cost_lut_disfavored_.empty());

  const double c_side = 5.0;
  const double resolution = 0.1;
  const double inscribed_radius = 0.3;
  const int lut_precision = 100;

  // Spot-check four distances in the decay region (beyond 3 cells inscribed).
  for (int d_scaled : {400, 500, 700, 900}) {
    double distance = static_cast<double>(d_scaled) / lut_precision;
    double factor = exp(-c_side * (distance * resolution - inscribed_radius));
    unsigned char expected =
      static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    EXPECT_EQ(layer->cost_lut_disfavored_[d_scaled], expected)
      << "d_scaled=" << d_scaled;
  }
}

// ============================================================
// SpatialHashTest — buildPathSpatialHash correctness
// ============================================================

class SpatialHashTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    layer_ = std::make_unique<TestableAsymmetricInflationLayer>();
    layer_->setInflationRadius(1.0);
  }

  // Replicates the hash-key formula used inside buildPathSpatialHash.
  static uint64_t makeKey(int64_t bx, int64_t by)
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(bx)) << 32) |
           static_cast<uint32_t>(by);
  }

  std::unique_ptr<TestableAsymmetricInflationLayer> layer_;
};

TEST_F(SpatialHashTest, empty_input_produces_empty_hash)
{
  auto hash = layer_->buildPathSpatialHash({});
  EXPECT_TRUE(hash.empty());
}

// A 3 m horizontal segment (bucket_size = inflation_radius = 1.0 m) must appear
// in every 1 m-wide bucket it sweeps through along the central row.
TEST_F(SpatialHashTest, long_segment_appears_in_all_covered_buckets)
{
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{0.0, 0.5}, {3.0, 0.5}}
  };
  auto hash = layer_->buildPathSpatialHash(segs);
  ASSERT_FALSE(hash.empty());

  // Buckets b_x in [0, 3] at b_y = 0 (floor(0.5/1.0)=0) must all contain seg 0.
  for (int64_t bx = 0; bx <= 3; ++bx) {
    uint64_t key = makeKey(bx, 0);
    auto it = hash.find(key);
    ASSERT_NE(it, hash.end()) << "Bucket (" << bx << ",0) is missing from hash";
    bool found = std::find(
      it->second.begin(), it->second.end(), size_t{0}) != it->second.end();
    EXPECT_TRUE(found) << "Segment 0 not found in bucket (" << bx << ",0)";
  }
}

// A zero-length segment must still be inserted into the hash bucket that
// contains the segment point.
TEST_F(SpatialHashTest, zero_length_segment_is_hashed)
{
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{1.5, 2.0}, {1.5, 2.0}}
  };
  auto hash = layer_->buildPathSpatialHash(segs);
  ASSERT_FALSE(hash.empty());

  // Bucket containing (1.5, 2.0): floor(1.5/1.0)=1, floor(2.0/1.0)=2.
  uint64_t key = makeKey(1, 2);
  auto it = hash.find(key);
  ASSERT_NE(it, hash.end()) << "Bucket (1,2) must contain the zero-length segment";
  bool found = std::find(
    it->second.begin(), it->second.end(), size_t{0}) != it->second.end();
  EXPECT_TRUE(found);
}

// ============================================================
// ObstacleSideTest — computeObstacleSide correctness
// ============================================================

class ObstacleSideTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 40x40 cells, 0.1 m resolution, origin at (0,0).
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(40, 40, 0.1, 0.0, 0.0);
    layer_ = std::make_unique<TestableAsymmetricInflationLayer>();
    layer_->setResolution(0.1);
    layer_->setInscribedRadius(0.3);
    layer_->setInflationRadius(1.0);
    layer_->setCostScalingFactorLeft(2.0);
    layer_->setCostScalingFactorRight(8.0);
  }

  std::vector<nav2_costmap_2d::AsymmetricPathSegment> makeSegments(
    const std::vector<std::pair<double, double>> & path)
  {
    std::vector<nav2_costmap_2d::AsymmetricPathSegment> segments;
    for (size_t i = 1; i < path.size(); ++i) {
      segments.push_back({path[i - 1], path[i]});
    }
    return segments;
  }

  std::unordered_map<uint64_t, std::vector<size_t>> buildSpatialHash(
    const std::vector<nav2_costmap_2d::AsymmetricPathSegment> & path_segments)
  {
    return layer_->buildPathSpatialHash(path_segments);
  }

  // Replicates the broad-phase + narrow pipeline used in updateCosts Phase 2.
  Side classifyObstacleSide(
    unsigned int mx, unsigned int my,
    const std::vector<nav2_costmap_2d::AsymmetricPathSegment> & path_segments,
    const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
    double bucket_size)
  {
    double cx, cy;
    costmap_->mapToWorld(mx, my, cx, cy);
    int64_t b_x = static_cast<int64_t>(std::floor(cx / bucket_size));
    int64_t b_y = static_cast<int64_t>(std::floor(cy / bucket_size));
    uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
      static_cast<uint32_t>(b_y);
    auto it = spatial_hash.find(key);
    if (it == spatial_hash.end()) {
      return Side::Neutral;
    }
    return layer_->computeObstacleSide(cx, cy, it->second, path_segments);
  }

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<TestableAsymmetricInflationLayer> layer_;
};

// Straight path along +x: cells north → +1 (left), south → -1 (right),
// cells outside the inflation radius → 0 (neutral).
TEST_F(ObstacleSideTest, computeObstacleSide_classification)
{
  std::vector<std::pair<double, double>> path = {
    {0.5, 2.0}, {1.5, 2.0}, {2.5, 2.0}, {3.5, 2.0}
  };

  double bucket_size = std::max(layer_->getInflationRadius(), 1.0);
  auto path_segments = makeSegments(path);
  auto spatial_hash = buildSpatialHash(path_segments);

  unsigned int mx_n, my_n;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 2.5, mx_n, my_n));
  EXPECT_EQ(classifyObstacleSide(mx_n, my_n, path_segments, spatial_hash, bucket_size), Side::Left);

  unsigned int mx_s, my_s;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 1.5, mx_s, my_s));
  EXPECT_EQ(classifyObstacleSide(mx_s, my_s, path_segments, spatial_hash, bucket_size),
    Side::Right);

  // 1.5 m from path — beyond inflation_radius 1.0 m → neutral.
  unsigned int mx_far, my_far;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 3.5, mx_far, my_far));
  EXPECT_EQ(classifyObstacleSide(mx_far, my_far, path_segments, spatial_hash, bucket_size),
    Side::Neutral);
}

// L-shaped path: a cell near the corner must be assigned to the nearest segment.
TEST_F(ObstacleSideTest, computeObstacleSide_closest_segment_selection)
{
  std::vector<std::pair<double, double>> path = {
    {0.5, 2.0}, {1.0, 2.0}, {1.5, 2.0}, {2.0, 2.0},
    {2.0, 1.5}, {2.0, 1.0}, {2.0, 0.5}
  };

  double bucket_size = std::max(layer_->getInflationRadius(), 1.0);
  auto path_segments = makeSegments(path);
  auto spatial_hash = buildSpatialHash(path_segments);

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(2.3, 1.7, mx, my));
  // Nearest segment is the vertical leg going south; (2.3, 1.7) is to its east,
  // which is LEFT of southbound travel → +1.
  EXPECT_EQ(classifyObstacleSide(mx, my, path_segments, spatial_hash, bucket_size), Side::Left);
}

// Separated path islands must not be bridged: a cell in the gap returns 0.
TEST_F(ObstacleSideTest, computeObstacleSide_does_not_bridge_filtered_path_gaps)
{
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> path_segments = {
    {{0.5, 2.0}, {1.0, 2.0}},
    {{3.0, 2.0}, {3.5, 2.0}}
  };

  double bucket_size = std::max(layer_->getInflationRadius(), 1.0);
  auto spatial_hash = buildSpatialHash(path_segments);

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 2.5, mx, my));
  EXPECT_EQ(classifyObstacleSide(mx, my, path_segments, spatial_hash, bucket_size), Side::Neutral);
}

// A zero-length segment always produces cross = 0 → neutral, even when the
// query point coincides with the segment location.
TEST_F(ObstacleSideTest, computeObstacleSide_zero_length_segment_is_neutral)
{
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{2.0, 2.0}, {2.0, 2.0}}
  };
  std::vector<size_t> candidates = {0};
  EXPECT_EQ(layer_->computeObstacleSide(2.0, 2.0, candidates, segs), Side::Neutral);
}

// All candidates rejected by AABB: min_dist_sq stays at max → beyond
// inflation_radius → returns 0.
TEST_F(ObstacleSideTest, computeObstacleSide_all_aabb_rejected_is_neutral)
{
  layer_->setInflationRadius(0.3);
  // Segment AABB padded by 0.3: [2.7, 3.8] x [2.7, 3.3] — excludes (1.5, 2.0).
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{3.0, 3.0}, {3.5, 3.0}}
  };
  std::vector<size_t> candidates = {0};
  EXPECT_EQ(layer_->computeObstacleSide(1.5, 2.0, candidates, segs), Side::Neutral);
}

// A cell that passes the AABB check but whose exact Euclidean distance to the
// nearest point on the segment exceeds inflation_radius returns 0.
// The corner (3.0, 1.0) of the AABB [−1, 3]×[−1, 1] has dist = √2 ≈ 1.41 > 1.0.
TEST_F(ObstacleSideTest, computeObstacleSide_corner_of_aabb_beyond_radius_is_neutral)
{
  layer_->setInflationRadius(1.0);
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{0.0, 0.0}, {2.0, 0.0}}
  };
  std::vector<size_t> candidates = {0};
  EXPECT_EQ(layer_->computeObstacleSide(3.0, 1.0, candidates, segs), Side::Neutral);
}

// A cell that projects onto the segment with cross product = 0 returns 0 (neutral).
TEST_F(ObstacleSideTest, computeObstacleSide_cell_on_segment_is_neutral)
{
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    {{0.5, 2.0}, {3.5, 2.0}}
  };
  std::vector<size_t> candidates = {0};
  // (1.5, 2.0) lies on the segment; cross product = 0.
  EXPECT_EQ(layer_->computeObstacleSide(1.5, 2.0, candidates, segs), Side::Neutral);
}

// When two segments have opposite cross products, the nearest one determines
// the classification.
TEST_F(ObstacleSideTest, computeObstacleSide_closest_segment_wins)
{
  // Need inflation_radius large enough to include both segments.
  layer_->setInflationRadius(2.0);
  std::vector<nav2_costmap_2d::AsymmetricPathSegment> segs = {
    // Segment 0 at y=1.8: cell (2.0,2.0) is NORTH → LEFT (+1), dist=0.2.
    {{0.5, 1.8}, {3.5, 1.8}},
    // Segment 1 at y=3.5: cell (2.0,2.0) is SOUTH → RIGHT (−1), dist=1.5.
    {{0.5, 3.5}, {3.5, 3.5}}
  };
  std::vector<size_t> candidates = {0, 1};
  // Segment 0 is closer → best_cross > 0 → LEFT (+1).
  EXPECT_EQ(layer_->computeObstacleSide(2.0, 2.0, candidates, segs), Side::Left);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
