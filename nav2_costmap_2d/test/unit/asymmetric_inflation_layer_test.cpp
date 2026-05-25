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
// Tests exercise cost_lut_disfavored_ and computeObstacleSide via a test-subclass
// that exposes protected members and methods without requiring a full
// LayeredCostmap / LifecycleNode stack.

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
  // Expose protected methods and members for testing
  using AsymmetricInflationLayer::buildPathSpatialHash;
  using AsymmetricInflationLayer::computeObstacleSide;
  using AsymmetricInflationLayer::cost_lut_disfavored_;

  // Plain setters so tests can configure the math without going through ROS init
  void setResolution(double r) {resolution_ = r;}
  void setInscribedRadius(double r) {inscribed_radius_ = r;}
  void setInflationRadius(double r) {inflation_radius_ = r;}
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
  void setCellInflationRadius(unsigned int r) {cell_inflation_radius_ = r;}
  void rebuildCaches() {computeAsymmetricCaches();}

private:
  void updateCostScalingFactor()
  {
    cost_scaling_factor_ = std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
  }
};

}  // namespace nav2_costmap_2d

using nav2_costmap_2d::TestableAsymmetricInflationLayer;

// Test 1: cost_lut_disfavored_ is built with c_side (the smaller scaling factor).
// Verifies the mathematical equivalence: using c_max on effective distance equals
// using c_side on physical distance, so the LUT must reflect c_side.
TEST(DisfavoredLutTest, lut_uses_c_side_scaling_factor)
{
  auto layer = std::make_unique<TestableAsymmetricInflationLayer>();
  // resolution=0.1m, inscribed_radius=0.3m (3 cells), cell_inflation_radius=20 cells
  layer->setResolution(0.1);
  layer->setInscribedRadius(0.3);
  layer->setInflationRadius(2.0);
  layer->setCellInflationRadius(20);

  // c_left=1, c_right=7 → c_side=1 (left is disfavored), c_max=7
  layer->setCostScalingFactorLeft(1.0);
  layer->setCostScalingFactorRight(7.0);
  layer->rebuildCaches();

  ASSERT_FALSE(layer->cost_lut_disfavored_.empty());

  const double resolution = 0.1;
  const double inscribed_radius = 0.3;
  const double c_side = 1.0;
  const int lut_precision = 100;  // COST_LUT_PRECISION

  // Spot-check several distances beyond the inscribed radius
  for (int d_scaled = lut_precision * 4; d_scaled <= lut_precision * 19;
    d_scaled += lut_precision)
  {
    double distance = static_cast<double>(d_scaled) / lut_precision;  // cells
    double factor = exp(-c_side * (distance * resolution - inscribed_radius));
    unsigned char expected =
      static_cast<unsigned char>((nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);

    EXPECT_EQ(layer->cost_lut_disfavored_[d_scaled], expected)
      << "LUT mismatch at d_scaled=" << d_scaled;
  }

  // Inside the inscribed radius: costs must equal INSCRIBED_INFLATED_OBSTACLE
  for (int d_scaled = 1; d_scaled < lut_precision * 3; ++d_scaled) {
    EXPECT_EQ(
      layer->cost_lut_disfavored_[d_scaled],
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      << "Expected INSCRIBED at d_scaled=" << d_scaled;
  }

  // At distance=0: must be LETHAL_OBSTACLE
  EXPECT_EQ(layer->cost_lut_disfavored_[0], nav2_costmap_2d::LETHAL_OBSTACLE);
}

// Test 2: swapping side factors rebuilds the LUT with the new c_side.
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

  // Swap sides: now c_right=1 is disfavored, c_left=7 is favored
  layer->setCostScalingFactorLeft(7.0);
  layer->setCostScalingFactorRight(1.0);
  layer->rebuildCaches();

  // With both cases having c_side=1, LUTs should be identical
  EXPECT_EQ(lut_c1, layer->cost_lut_disfavored_);
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

  // Replicates the Phase 2 broad-phase + narrow/exact pipeline:
  // hash lookup → if miss return 0, else call computeObstacleSide.
  int8_t classifyObstacleSide(
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
      return 0;
    }
    return layer_->computeObstacleSide(cx, cy, it->second, path_segments);
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
  auto path_segments = makeSegments(path);
  auto spatial_hash = buildSpatialHash(path_segments);

  // North cell: world (2.0, 2.5) → map indices
  unsigned int mx_n, my_n;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 2.5, mx_n, my_n));
  EXPECT_EQ(classifyObstacleSide(mx_n, my_n, path_segments, spatial_hash, bucket_size), 1);

  // South cell: world (2.0, 1.5)
  unsigned int mx_s, my_s;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 1.5, mx_s, my_s));
  EXPECT_EQ(classifyObstacleSide(mx_s, my_s, path_segments, spatial_hash, bucket_size), -1);

  // Far north cell: world (2.0, 3.5) — 1.5m from path, beyond inflation_radius 1.0
  unsigned int mx_far, my_far;
  ASSERT_TRUE(costmap_->worldToMap(2.0, 3.5, mx_far, my_far));
  EXPECT_EQ(classifyObstacleSide(mx_far, my_far, path_segments, spatial_hash, bucket_size), 0);
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
  auto path_segments = makeSegments(path);
  auto spatial_hash = buildSpatialHash(path_segments);

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(2.3, 1.7, mx, my));
  // Nearest segment is the vertical leg (2.0,2.0)->(2.0,1.5)->... (direction = (0,-1)).
  // A cell at +x of a southbound path is on the LEFT of its forward motion → +1.
  EXPECT_EQ(classifyObstacleSide(mx, my, path_segments, spatial_hash, bucket_size), 1);
}

// Test 7: separated path islands must not create an artificial segment through the gap.
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
  EXPECT_EQ(classifyObstacleSide(mx, my, path_segments, spatial_hash, bucket_size), 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
