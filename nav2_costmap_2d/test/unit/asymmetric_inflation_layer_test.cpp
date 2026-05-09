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
// These tests exercise getEffectiveDistance and computeObstacleSide via a
// test-subclass that exposes the protected members and methods without
// requiring a full LayeredCostmap / LifecycleNode stack.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdint>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"

namespace nav2_costmap_2d
{

class TestableAsymmetricInflationLayer : public AsymmetricInflationLayer
{
public:
  // Expose protected methods for testing
  using AsymmetricInflationLayer::getEffectiveDistance;
  using AsymmetricInflationLayer::computeObstacleSide;

  // Plain setters so tests can configure the math without going through ROS init
  void setResolution(double r) {resolution_ = r;}
  void setInscribedRadius(double r) {inscribed_radius_ = r;}
  void setInflationRadius(double r)
  {
    inflation_radius_ = r;
  }
  void setCostScalingFactorLeft(double c)
  {
    cost_scaling_factor_left_ = c;
    updateCostScalingFactor();
  }

  void setCostScalingFactorRight(double c)
  {
    cost_scaling_factor_right_ = c;
    updateCostScalingFactor();
  }

private:
  void updateCostScalingFactor()
  {
    cost_scaling_factor_ = std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
  }
};

}  // namespace nav2_costmap_2d

using nav2_costmap_2d::TestableAsymmetricInflationLayer;

class EffectiveDistanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    layer_ = std::make_unique<TestableAsymmetricInflationLayer>();
    layer_->setResolution(0.1);
    layer_->setInscribedRadius(0.3);  // 3 cells
    layer_->setInflationRadius(2.0);
  }

  std::unique_ptr<TestableAsymmetricInflationLayer> layer_;
};

// Formula under test (inscribed_cells = inscribed_radius / resolution = 0.3 / 0.1 = 3):
//   d <= inscribed_cells: eff = d  (no asymmetry inside the collision core)
//   d >  inscribed_cells: eff = inscribed_cells + (d - inscribed_cells) * (c_side / c_max)
// where c_max = max(c_left, c_right) and neutral side uses c_side = c_max.

// Test 1: when c_left == c_right, scale=1 for all sides, so the function is the identity.
TEST_F(EffectiveDistanceTest, getEffectiveDistance_symmetric_when_sides_equal)
{
  layer_->setCostScalingFactorLeft(4.0);
  layer_->setCostScalingFactorRight(4.0);
  for (double d = 0.0; d < 20.0; d += 0.5) {
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, +1), d);
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, -1), d);
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, 0), d);
  }
}

// Test 2: inside the inscribed radius all sides are treated identically, regardless of factors.
TEST_F(EffectiveDistanceTest, getEffectiveDistance_symmetric_inside_inscribed_radius)
{
  layer_->setCostScalingFactorLeft(1.0);
  layer_->setCostScalingFactorRight(7.0);
  const double inscribed_cells = 3.0;  // 0.3 / 0.1
  for (double d = 0.0; d < inscribed_cells; d += 0.5) {
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, +1), d);
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, -1), d);
    EXPECT_DOUBLE_EQ(layer_->getEffectiveDistance(d, 0), d);
  }
}

// Test 3: with c_left=1, c_right=7 (so c_max=7), the LEFT effective distance is
// compressed (smaller decay rate → bigger reach) while RIGHT and neutral match the
// physical distance.
TEST_F(EffectiveDistanceTest, getEffectiveDistance_left_compressed_right_stretched)
{
  layer_->setCostScalingFactorLeft(1.0);
  layer_->setCostScalingFactorRight(7.0);
  const double inscribed_cells = 3.0;
  // d=4: excess = 1.0 cell beyond inscribed radius; asymmetry is active here.
  const double d = 4.0;
  const double excess = d - inscribed_cells;
  const double left = layer_->getEffectiveDistance(d, +1);
  const double right = layer_->getEffectiveDistance(d, -1);
  const double neutral = layer_->getEffectiveDistance(d, 0);

  EXPECT_DOUBLE_EQ(left, inscribed_cells + excess * (1.0 / 7.0));
  EXPECT_DOUBLE_EQ(right, inscribed_cells + excess);  // c_right == c_max → scale=1
  EXPECT_DOUBLE_EQ(neutral, d);                       // neutral uses c_max → scale=1
  EXPECT_LT(left, right) << "LEFT (smaller decay) should have compressed eff_dist";
  EXPECT_DOUBLE_EQ(right, neutral) << "RIGHT == c_max so it matches neutral";
}

// Test 4: swapping the per-side decay rates swaps left/right effective distances.
// Use d outside the inscribed radius so asymmetry is active.
TEST_F(EffectiveDistanceTest, getEffectiveDistance_swaps_when_sides_swap)
{
  const double d = 4.0;  // outside inscribed_cells=3 so asymmetry is active
  layer_->setCostScalingFactorLeft(1.0);
  layer_->setCostScalingFactorRight(7.0);
  double pos_left = layer_->getEffectiveDistance(d, +1);
  double pos_right = layer_->getEffectiveDistance(d, -1);

  layer_->setCostScalingFactorLeft(7.0);
  layer_->setCostScalingFactorRight(1.0);
  double neg_left = layer_->getEffectiveDistance(d, +1);
  double neg_right = layer_->getEffectiveDistance(d, -1);

  EXPECT_DOUBLE_EQ(pos_left, neg_right);
  EXPECT_DOUBLE_EQ(pos_right, neg_left);
  EXPECT_NE(pos_left, pos_right) << "test setup: sides must differ to be meaningful";
}

class ObstacleSideTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 40x40 cells, 0.1m resolution, origin at (0,0) — world range [0, 4]m.
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(40, 40, 0.1, 0.0, 0.0);
    layer_ = std::make_unique<TestableAsymmetricInflationLayer>();
    layer_->setResolution(0.1);
    layer_->setInscribedRadius(0.3);
    layer_->setInflationRadius(1.0);
    // Side classification doesn't depend on the decay rates; pick any unequal pair.
    layer_->setCostScalingFactorLeft(2.0);
    layer_->setCostScalingFactorRight(8.0);
  }

  // Helper method to simulate the spatial hash construction from updateCosts
  std::unordered_map<uint64_t, std::vector<size_t>> buildSpatialHash(
    const std::vector<std::pair<double, double>> & path,
    double bucket_size)
  {
    std::unordered_map<uint64_t, std::vector<size_t>> spatial_hash;
    double inflation_radius = layer_->getInflationRadius();

    for (size_t p = 0; p < path.size() - 1; ++p) {
      double ax = path[p].first;
      double ay = path[p].second;
      double bx = path[p + 1].first;
      double by = path[p + 1].second;

      double min_x = std::min(ax, bx) - inflation_radius;
      double max_x = std::max(ax, bx) + inflation_radius;
      double min_y = std::min(ay, by) - inflation_radius;
      double max_y = std::max(ay, by) + inflation_radius;

      int64_t min_bx = static_cast<int64_t>(std::floor(min_x / bucket_size));
      int64_t max_bx = static_cast<int64_t>(std::floor(max_x / bucket_size));
      int64_t min_by = static_cast<int64_t>(std::floor(min_y / bucket_size));
      int64_t max_by = static_cast<int64_t>(std::floor(max_y / bucket_size));

      for (int64_t b_x = min_bx; b_x <= max_bx; ++b_x) {
        for (int64_t b_y = min_by; b_y <= max_by; ++b_y) {
          uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
            (static_cast<uint32_t>(b_y));
          spatial_hash[key].push_back(p);
        }
      }
    }
    return spatial_hash;
  }

  // Replicates the Phase 2 broad-phase + narrow/exact pipeline:
  // hash lookup → if miss return 0, else call computeObstacleSide.
  int8_t classifyObstacleSide(
    unsigned int mx, unsigned int my,
    const std::vector<std::pair<double, double>> & path,
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
      return 0;
    }
    return layer_->computeObstacleSide(cx, cy, it->second, path);
  }

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<TestableAsymmetricInflationLayer> layer_;
};

// Test 5: straight path along +x axis
//   - cells north of the path should be classified +1 (left)
//   - cells south of the path should be classified -1 (right)
//   - cells beyond inflation_radius should be classified 0 (neutral)
TEST_F(ObstacleSideTest, computeObstacleSide_classification)
{
  // Path along y=2.0 from x=0.5 to x=3.5
  std::vector<std::pair<double, double>> path = {
    {0.5, 2.0}, {1.5, 2.0}, {2.5, 2.0}, {3.5, 2.0}
  };

  double bucket_size = std::max(layer_->getInflationRadius(), 1.0);
  auto spatial_hash = buildSpatialHash(path, bucket_size);

  // North cell: world (2.0, 2.5) → map indices
  unsigned int mx_n, my_n;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 2.5, mx_n, my_n));
  EXPECT_EQ(classifyObstacleSide(mx_n, my_n, path, spatial_hash, bucket_size), 1);

  // South cell: world (2.0, 1.5)
  unsigned int mx_s, my_s;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 1.5, mx_s, my_s));
  EXPECT_EQ(classifyObstacleSide(mx_s, my_s, path, spatial_hash, bucket_size), -1);

  // Far north cell: world (2.0, 3.5) — 1.5m from path, beyond inflation_radius 1.0
  unsigned int mx_far, my_far;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 3.5, mx_far, my_far));
  EXPECT_EQ(classifyObstacleSide(mx_far, my_far, path, spatial_hash, bucket_size), 0);
}

// Test 6: L-shaped path — obstacle near the corner is assigned to the nearest segment,
// not the one whose vertex is closest.
TEST_F(ObstacleSideTest, computeObstacleSide_closest_segment_selection)
{
  // L-shape: horizontal leg along y=2.0 from x=0.5 to x=2.0, then vertical leg down
  // to y=0.5 at x=2.0. A test cell at world (2.3, 1.7) is right of the vertical leg.
  std::vector<std::pair<double, double>> path = {
    {0.5, 2.0}, {1.0, 2.0}, {1.5, 2.0}, {2.0, 2.0},
    {2.0, 1.5}, {2.0, 1.0}, {2.0, 0.5}
  };

  double bucket_size = std::max(layer_->getInflationRadius(), 1.0);
  auto spatial_hash = buildSpatialHash(path, bucket_size);

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(2.3, 1.7, mx, my));
  // Nearest segment is the vertical leg (2.0,2.0)->(2.0,1.5)->... (direction = (0,-1)).
  // A cell at +x of a southbound path is on the LEFT of its forward motion → +1.
  EXPECT_EQ(classifyObstacleSide(mx, my, path, spatial_hash, bucket_size), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
