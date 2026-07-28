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

/**
 * @file asymmetric_inflation_overhead_benchmark.cpp
 * @brief Measures the per-update overhead of AsymmetricInflationLayer in a
 *        standalone path-aware inflation pipeline.
 *
 * Three fixtures are compared across the registered costmap sizes at
 * 0.05 m/cell, 50 iterations each:
 *
 *   InflationFixture   — InflationLayer::updateCosts() alone
 *                        (the distance-transform symmetric baseline)
 *
 *   LegacyInflationFixture
 *                      — LegacyInflationLayer::updateCosts() alone
 *                        (the pre-DT symmetric baseline)
 *
 *   AsymmetricFixture  — AsymmetricInflationLayer::updateCosts() alone
 *                        (inherited distance-transform baseline plus
 *                        asymmetric overlay)
 *
 * Comparing AsymmetricFixture against InflationFixture shows the overlay
 * overhead on top of the DT baseline; LegacyInflationFixture keeps the old
 * symmetric algorithm visible as a separate reference point.
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "benchmark/benchmark.h"
#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/legacy_inflation_layer.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace
{

static constexpr const char * kGlobalFrame = "map";
static constexpr double kResolution = 0.05;         ///< metres per cell
static constexpr double kInflationRadius = 2.0;    ///< metres
/// Asymmetric per-side decay rates so the overlay work is active.
static constexpr double kCostScalingFactorLeft = 4.0;
static constexpr double kCostScalingFactorRight = 1.0;
static constexpr double kCostScalingFactor =
  (kCostScalingFactorLeft > kCostScalingFactorRight) ?
  kCostScalingFactorLeft : kCostScalingFactorRight;
static constexpr double kOccupancy = 0.50;          ///< lethal-obstacle fraction
static constexpr unsigned int kObstacleSeed = 42;

/**
 * @brief Exposes the protected globalPathCallback for path injection in benchmarks.
 */
class BenchmarkAsymmetricInflationLayer : public nav2_costmap_2d::AsymmetricInflationLayer
{
public:
  /** @brief Deliver @p msg as if it arrived on the plan topic. */
  void injectPath(nav_msgs::msg::Path::ConstSharedPtr msg) {globalPathCallback(msg);}
};

/**
 * @brief Seed lethal rectangular obstacles covering ~@p occupancy_pct of the costmap.
 *
 * Clears the entire map first, then places random-sized rectangles until the
 * target cell count is reached.
 */
void generateRectangularObstacles(
  nav2_costmap_2d::Costmap2D & costmap,
  double occupancy_pct,
  unsigned int seed = kObstacleSeed)
{
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const std::size_t total = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  const std::size_t target = static_cast<std::size_t>(
    static_cast<double>(total) * occupancy_pct);

  const unsigned int min_w = std::max(1u, static_cast<unsigned int>(size_x * 0.05));
  const unsigned int max_w = std::max(min_w + 1u, static_cast<unsigned int>(size_x * 0.15));
  const unsigned int min_h = std::max(1u, static_cast<unsigned int>(size_y * 0.05));
  const unsigned int max_h = std::max(min_h + 1u, static_cast<unsigned int>(size_y * 0.15));

  std::mt19937 gen(seed);
  std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
  std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);
  std::uniform_int_distribution<unsigned int> dist_w(min_w, max_w);
  std::uniform_int_distribution<unsigned int> dist_h(min_h, max_h);

  std::memset(costmap.getCharMap(), nav2_costmap_2d::FREE_SPACE, total);

  std::size_t occupied = 0;
  for (unsigned int attempts = 0; occupied < target && attempts < 10000u; ++attempts) {
    unsigned int rx = dist_x(gen);
    unsigned int ry = dist_y(gen);
    unsigned int ex = std::min(rx + dist_w(gen), size_x);
    unsigned int ey = std::min(ry + dist_h(gen), size_y);
    for (unsigned int cy = ry; cy < ey && occupied < target; ++cy) {
      for (unsigned int cx = rx; cx < ex && occupied < target; ++cx) {
        if (costmap.getCost(cx, cy) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          costmap.setCost(cx, cy, nav2_costmap_2d::LETHAL_OBSTACLE);
          ++occupied;
        }
      }
    }
  }
}

/** @brief Build a 0.6 m × 0.4 m rectangular robot footprint. */
std::vector<geometry_msgs::msg::Point> makeFootprint()
{
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x = 0.3; fp[0].y = 0.2;
  fp[1].x = 0.3; fp[1].y = -0.2;
  fp[2].x = -0.3; fp[2].y = -0.2;
  fp[3].x = -0.3; fp[3].y = 0.2;
  return fp;
}

/**
 * @brief Clear and re-seed lethal obstacles within a square sub-region.
 *
 * Simulates one cycle of incremental sensor data: the patch is cleared then
 * re-filled with random rectangular obstacles at @p kOccupancy density.
 * Called during paused timing so only the resulting updateCosts() is measured.
 *
 * @param costmap  The costmap to modify in-place.
 * @param ox       Left cell coordinate of the patch.
 * @param oy       Bottom cell coordinate of the patch.
 * @param side     Side length of the square patch in cells.
 * @param seed     RNG seed; increment each call to vary the obstacle layout.
 */
void modifyPatch(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned int ox, unsigned int oy, unsigned int side, unsigned int seed)
{
  const unsigned int ex = ox + side;
  const unsigned int ey = oy + side;

  for (unsigned int cy = oy; cy < ey; ++cy) {
    for (unsigned int cx = ox; cx < ex; ++cx) {
      costmap.setCost(cx, cy, nav2_costmap_2d::FREE_SPACE);
    }
  }

  const std::size_t target = static_cast<std::size_t>(
    static_cast<double>(side) * static_cast<double>(side) * kOccupancy);
  const unsigned int min_d = std::max(1u, side / 10u);
  const unsigned int max_d = std::max(min_d + 1u, side / 5u);

  std::mt19937 gen(seed);
  std::uniform_int_distribution<unsigned int> dist_x(ox, ex - 1u);
  std::uniform_int_distribution<unsigned int> dist_y(oy, ey - 1u);
  std::uniform_int_distribution<unsigned int> dist_w(min_d, max_d);
  std::uniform_int_distribution<unsigned int> dist_h(min_d, max_d);

  std::size_t occupied = 0;
  for (unsigned int a = 0; occupied < target && a < 10000u; ++a) {
    unsigned int rx = dist_x(gen);
    unsigned int ry = dist_y(gen);
    unsigned int bx = std::min(rx + dist_w(gen), ex);
    unsigned int by = std::min(ry + dist_h(gen), ey);
    for (unsigned int cy = ry; cy < by && occupied < target; ++cy) {
      for (unsigned int cx = rx; cx < bx && occupied < target; ++cx) {
        if (costmap.getCost(cx, cy) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          costmap.setCost(cx, cy, nav2_costmap_2d::LETHAL_OBSTACLE);
          ++occupied;
        }
      }
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Baseline: InflationLayer alone
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for the baseline pipeline: InflationLayer only.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class InflationFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("inflation.inflation_radius", kInflationRadius),
      rclcpp::Parameter("inflation.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("inflation.inflate_unknown", false),
      rclcpp::Parameter("inflation.inflate_around_unknown", false),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::InflationLayer>();
    layer_->initialize(layers_.get(), "inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    layers_->setFootprint(makeFootprint());

    generateRectangularObstacles(*layers_->getCostmap(), kOccupancy);
  }

  void TearDown(benchmark::State &) override
  {
    layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> layer_;
};

BENCHMARK_DEFINE_F(InflationFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const int w = static_cast<int>(width_);
  const int h = static_cast<int>(height_);

  for (auto _ : state) {
    state.PauseTiming();
    generateRectangularObstacles(*costmap, kOccupancy);
    state.ResumeTiming();

    layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(InflationFixture, UpdateCosts)
->Args({60, 60})      // 3x3 m
->Args({100, 100})    // 5x5 m
->Args({200, 200})    // 10x10 m
->Args({400, 400})    // 20x20 m
->Args({800, 800})    // 40x40 m
->Args({1600, 1600})  // 80x80 m
->Args({3200, 3200})  // 160x160 m
->Args({6400, 6400})  // 320x320 m
->Args({12800, 12800})  // 640x640 m
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Baseline: LegacyInflationLayer alone
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for the legacy baseline pipeline: LegacyInflationLayer only.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class LegacyInflationFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("legacy_inflation.inflation_radius", kInflationRadius),
      rclcpp::Parameter("legacy_inflation.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("legacy_inflation.inflate_unknown", false),
      rclcpp::Parameter("legacy_inflation.inflate_around_unknown", false),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("legacy_inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::LegacyInflationLayer>();
    layer_->initialize(layers_.get(), "legacy_inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    layers_->setFootprint(makeFootprint());

    generateRectangularObstacles(*layers_->getCostmap(), kOccupancy);
  }

  void TearDown(benchmark::State &) override
  {
    layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::LegacyInflationLayer> layer_;
};

BENCHMARK_DEFINE_F(LegacyInflationFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const int w = static_cast<int>(width_);
  const int h = static_cast<int>(height_);

  for (auto _ : state) {
    state.PauseTiming();
    generateRectangularObstacles(*costmap, kOccupancy);
    state.ResumeTiming();

    layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(LegacyInflationFixture, UpdateCosts)
->Args({60, 60})      // 3x3 m
->Args({100, 100})    // 5x5 m
->Args({200, 200})    // 10x10 m
->Args({400, 400})    // 20x20 m
->Args({800, 800})    // 40x40 m
->Args({1600, 1600})  // 80x80 m
->Args({3200, 3200})  // 160x160 m
->Args({6400, 6400})  // 320x320 m
->Args({12800, 12800})  // 640x640 m
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Full map: AsymmetricInflationLayer standalone
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for the standalone asymmetric pipeline.
 *
 * AsymmetricInflationLayer writes the inherited distance-transform symmetric
 * baseline and then raises costs where the path-aware overlay exceeds it.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class AsymmetricFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("asymmetric_inflation_layer.enabled", true),
      rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", kInflationRadius),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_unknown", false),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.cost_scaling_factor_left", kCostScalingFactorLeft),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.cost_scaling_factor_right", kCostScalingFactorRight),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_around_unknown", false),
      rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 1.5),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.plan_topic", std::string("/benchmark_asymmetric_plan")),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("asymmetric_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    asym_layer_ = std::make_shared<BenchmarkAsymmetricInflationLayer>();
    asym_layer_->initialize(
      layers_.get(), "asymmetric_inflation_layer", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(asym_layer_));

    layers_->setFootprint(makeFootprint());

    injectDiagonalPath();
    generateRectangularObstacles(*layers_->getCostmap(), kOccupancy);
  }

  void TearDown(benchmark::State &) override
  {
    asym_layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<BenchmarkAsymmetricInflationLayer> asym_layer_;

private:
  void injectDiagonalPath()
  {
    const double width_m = static_cast<double>(width_) * kResolution;
    const double height_m = static_cast<double>(height_) * kResolution;
    const double diag = std::hypot(width_m, height_m);
    const double step = 0.5;
    const int n = std::max(2, static_cast<int>(std::ceil(diag / step)) + 1);

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = kGlobalFrame;

    for (int i = 0; i < n; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(n - 1);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = kGlobalFrame;
      ps.pose.position.x = t * width_m;
      ps.pose.position.y = t * height_m;
      ps.pose.orientation.w = 1.0;
      path->poses.push_back(ps);
    }

    asym_layer_->injectPath(path);
  }
};

BENCHMARK_DEFINE_F(AsymmetricFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const int w = static_cast<int>(width_);
  const int h = static_cast<int>(height_);

  for (auto _ : state) {
    state.PauseTiming();
    generateRectangularObstacles(*costmap, kOccupancy);
    state.ResumeTiming();

    asym_layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(AsymmetricFixture, UpdateCosts)
->Args({60, 60})      // 3x3 m
->Args({100, 100})    // 5x5 m
->Args({200, 200})    // 10x10 m
->Args({400, 400})    // 20x20 m
->Args({800, 800})    // 40x40 m
->Args({1600, 1600})  // 80x80 m
->Args({3200, 3200})  // 160x160 m
->Args({6400, 6400})  // 320x320 m
->Args({12800, 12800})  // 640x640 m
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Incremental: InflationLayer with a small dirty window
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for incremental updates: InflationLayer only.
 *
 * Pre-warms the costmap with a full updateCosts() call, then benchmarks
 * updateCosts() on a small moving patch — simulating an upstream obstacle
 * layer that dirtied only a local window.
 *
 * The patch_side_cells argument represents the window after updateBounds()
 * has expanded the raw dirty region by inflation_radius_ on each side.
 *
 * Benchmark arguments: {width_cells, height_cells, patch_side_cells}.
 */
class IncrementalInflationFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));
    patch_ = static_cast<unsigned int>(state.range(2));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("inflation.inflation_radius", kInflationRadius),
      rclcpp::Parameter("inflation.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("inflation.inflate_unknown", false),
      rclcpp::Parameter("inflation.inflate_around_unknown", false),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("incremental_inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::InflationLayer>();
    layer_->initialize(layers_.get(), "inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    layers_->setFootprint(makeFootprint());

    // Pre-warm: full update to put the layer in a valid post-update state.
    auto * costmap = layers_->getCostmap();
    generateRectangularObstacles(*costmap, kOccupancy);
    layer_->updateCosts(*costmap, 0, 0, static_cast<int>(width_), static_cast<int>(height_));
  }

  void TearDown(benchmark::State &) override
  {
    layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  unsigned int patch_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> layer_;
};

BENCHMARK_DEFINE_F(IncrementalInflationFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const unsigned int max_ox = (width_ > patch_) ? width_ - patch_ : 0u;
  const unsigned int max_oy = (height_ > patch_) ? height_ - patch_ : 0u;
  unsigned int step = 0;

  for (auto _ : state) {
    state.PauseTiming();
    // Cycle patch position with co-prime strides to avoid hot-cache repetition.
    const unsigned int ox = max_ox ? (step * 37u) % max_ox : 0u;
    const unsigned int oy = max_oy ? (step * 53u) % max_oy : 0u;
    modifyPatch(*costmap, ox, oy, patch_, kObstacleSeed + step);
    ++step;
    state.ResumeTiming();

    layer_->updateCosts(
      *costmap,
      static_cast<int>(ox), static_cast<int>(oy),
      static_cast<int>(ox + patch_), static_cast<int>(oy + patch_));
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["patch_side_m"] = static_cast<double>(patch_) * kResolution;
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(IncrementalInflationFixture, UpdateCosts)
->Args({800, 800, 100})    // 40×40 m map, ~5 m patch
->Args({800, 800, 200})    // 40×40 m map, ~10 m patch
->Args({1600, 1600, 100})  // 80×80 m map, ~5 m patch
->Args({1600, 1600, 200})  // 80×80 m map, ~10 m patch
->Args({3200, 3200, 100})  // 160×160 m map, ~5 m patch
->Args({3200, 3200, 200})  // 160×160 m map, ~10 m patch
->Args({6400, 6400, 100})  // 320×320 m map, ~5 m patch
->Args({6400, 6400, 200})  // 320×320 m map, ~10 m patch
->Args({12800, 12800, 100})  // 640×640 m map, ~5 m patch
->Args({12800, 12800, 200})  // 640×640 m map, ~10 m patch
->Args({25600, 25600, 100})  // 1280×1280 m map, ~5 m patch
->Args({25600, 25600, 200})  // 1280×1280 m map, ~10 m patch
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Incremental: LegacyInflationLayer with a small dirty window
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for incremental updates: LegacyInflationLayer only.
 *
 * Benchmark arguments: {width_cells, height_cells, patch_side_cells}.
 */
class IncrementalLegacyInflationFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));
    patch_ = static_cast<unsigned int>(state.range(2));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("legacy_inflation.inflation_radius", kInflationRadius),
      rclcpp::Parameter("legacy_inflation.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("legacy_inflation.inflate_unknown", false),
      rclcpp::Parameter("legacy_inflation.inflate_around_unknown", false),
    });
    node_ =
      std::make_shared<nav2::LifecycleNode>("incremental_legacy_inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::LegacyInflationLayer>();
    layer_->initialize(layers_.get(), "legacy_inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    layers_->setFootprint(makeFootprint());

    // Pre-warm: full update to put the layer in a valid post-update state.
    auto * costmap = layers_->getCostmap();
    generateRectangularObstacles(*costmap, kOccupancy);
    layer_->updateCosts(*costmap, 0, 0, static_cast<int>(width_), static_cast<int>(height_));
  }

  void TearDown(benchmark::State &) override
  {
    layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  unsigned int patch_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::LegacyInflationLayer> layer_;
};

BENCHMARK_DEFINE_F(IncrementalLegacyInflationFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const unsigned int max_ox = (width_ > patch_) ? width_ - patch_ : 0u;
  const unsigned int max_oy = (height_ > patch_) ? height_ - patch_ : 0u;
  unsigned int step = 0;

  for (auto _ : state) {
    state.PauseTiming();
    const unsigned int ox = max_ox ? (step * 37u) % max_ox : 0u;
    const unsigned int oy = max_oy ? (step * 53u) % max_oy : 0u;
    modifyPatch(*costmap, ox, oy, patch_, kObstacleSeed + step);
    ++step;
    state.ResumeTiming();

    layer_->updateCosts(
      *costmap,
      static_cast<int>(ox), static_cast<int>(oy),
      static_cast<int>(ox + patch_), static_cast<int>(oy + patch_));
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["patch_side_m"] = static_cast<double>(patch_) * kResolution;
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(IncrementalLegacyInflationFixture, UpdateCosts)
->Args({800, 800, 100})    // 40×40 m map, ~5 m patch
->Args({800, 800, 200})    // 40×40 m map, ~10 m patch
->Args({1600, 1600, 100})  // 80×80 m map, ~5 m patch
->Args({1600, 1600, 200})  // 80×80 m map, ~10 m patch
->Args({3200, 3200, 100})  // 160×160 m map, ~5 m patch
->Args({3200, 3200, 200})  // 160×160 m map, ~10 m patch
->Args({6400, 6400, 100})  // 320×320 m map, ~5 m patch
->Args({6400, 6400, 200})  // 320×320 m map, ~10 m patch
->Args({12800, 12800, 100})  // 640×640 m map, ~5 m patch
->Args({12800, 12800, 200})  // 640×640 m map, ~10 m patch
->Args({25600, 25600, 100})  // 1280×1280 m map, ~5 m patch
->Args({25600, 25600, 200})  // 1280×1280 m map, ~10 m patch
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Incremental: AsymmetricInflationLayer standalone
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for incremental updates: standalone asymmetric pipeline.
 *
 * Same structure as IncrementalInflationFixture but benchmarks
 * AsymmetricInflationLayer, which internally runs the inherited symmetric
 * baseline before the asymmetric overlay. A diagonal path is injected so the
 * overlay asymmetric layer is active.
 *
 * Benchmark arguments:
 *   {width_cells, height_cells, patch_side_cells, fixed_path_length_m}
 *
 * fixed_path_length_m == 0 keeps the historical full-map diagonal path, so path
 * length grows with map size. Non-zero values keep the path length fixed and
 * center the dirty patch on that path, isolating global-map-size effects from
 * path-length effects.
 */
class IncrementalAsymmetricFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));
    patch_ = static_cast<unsigned int>(state.range(2));
    fixed_path_length_m_ = static_cast<double>(state.range(3));

    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("asymmetric_inflation_layer.enabled", true),
      rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", kInflationRadius),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_unknown", false),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.cost_scaling_factor_left", kCostScalingFactorLeft),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.cost_scaling_factor_right", kCostScalingFactorRight),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_around_unknown", false),
      rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 1.5),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.plan_topic", std::string("/benchmark_incr_asymmetric_plan")),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("incremental_asymmetric_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    asym_layer_ = std::make_shared<BenchmarkAsymmetricInflationLayer>();
    asym_layer_->initialize(
      layers_.get(), "asymmetric_inflation_layer", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(asym_layer_));

    layers_->setFootprint(makeFootprint());
    injectDiagonalPath();

    // Pre-warm: full update so the layer starts in a valid post-update state.
    auto * costmap = layers_->getCostmap();
    generateRectangularObstacles(*costmap, kOccupancy);
    asym_layer_->updateCosts(
      *costmap, 0, 0, static_cast<int>(width_), static_cast<int>(height_));
  }

  void TearDown(benchmark::State &) override
  {
    asym_layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  unsigned int patch_{0};
  double fixed_path_length_m_{0.0};
  double actual_path_length_m_{0.0};
  std::size_t path_pose_count_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<BenchmarkAsymmetricInflationLayer> asym_layer_;

private:
  void injectDiagonalPath()
  {
    const double width_m = static_cast<double>(width_) * kResolution;
    const double height_m = static_cast<double>(height_) * kResolution;
    const double diag = std::hypot(width_m, height_m);
    double start_x = 0.0;
    double start_y = 0.0;
    double end_x = width_m;
    double end_y = height_m;

    if (fixed_path_length_m_ > 0.0) {
      const double path_length = std::min(fixed_path_length_m_, diag * 0.95);
      const double axis_delta = path_length / std::sqrt(2.0);
      const double center_x = width_m * 0.5;
      const double center_y = height_m * 0.5;
      start_x = std::clamp(center_x - axis_delta * 0.5, 0.0, width_m);
      start_y = std::clamp(center_y - axis_delta * 0.5, 0.0, height_m);
      end_x = std::clamp(center_x + axis_delta * 0.5, 0.0, width_m);
      end_y = std::clamp(center_y + axis_delta * 0.5, 0.0, height_m);
    }

    actual_path_length_m_ = std::hypot(end_x - start_x, end_y - start_y);
    const double step = 0.5;
    const int n = std::max(2, static_cast<int>(std::ceil(actual_path_length_m_ / step)) + 1);

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = kGlobalFrame;
    for (int i = 0; i < n; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(n - 1);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = kGlobalFrame;
      ps.pose.position.x = start_x + t * (end_x - start_x);
      ps.pose.position.y = start_y + t * (end_y - start_y);
      ps.pose.orientation.w = 1.0;
      path->poses.push_back(ps);
    }
    path_pose_count_ = path->poses.size();
    asym_layer_->injectPath(path);
  }
};

BENCHMARK_DEFINE_F(IncrementalAsymmetricFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const unsigned int max_ox = (width_ > patch_) ? width_ - patch_ : 0u;
  const unsigned int max_oy = (height_ > patch_) ? height_ - patch_ : 0u;
  unsigned int step = 0;

  for (auto _ : state) {
    state.PauseTiming();
    const bool fixed_path = fixed_path_length_m_ > 0.0;
    const unsigned int ox = fixed_path ?
      ((width_ > patch_) ? (width_ - patch_) / 2u : 0u) :
      (max_ox ? (step * 37u) % max_ox : 0u);
    const unsigned int oy = fixed_path ?
      ((height_ > patch_) ? (height_ - patch_) / 2u : 0u) :
      (max_oy ? (step * 53u) % max_oy : 0u);
    modifyPatch(*costmap, ox, oy, patch_, kObstacleSeed + step);
    ++step;
    state.ResumeTiming();

    asym_layer_->updateCosts(
      *costmap,
      static_cast<int>(ox), static_cast<int>(oy),
      static_cast<int>(ox + patch_), static_cast<int>(oy + patch_));
  }

  state.counters["cells"] = static_cast<double>(width_) * static_cast<double>(height_);
  state.counters["patch_side_m"] = static_cast<double>(patch_) * kResolution;
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
  state.counters["fixed_path_length_m"] = fixed_path_length_m_;
  state.counters["actual_path_length_m"] = actual_path_length_m_;
  state.counters["path_poses"] = static_cast<double>(path_pose_count_);
}

BENCHMARK_REGISTER_F(IncrementalAsymmetricFixture, UpdateCosts)
->Args({800, 800, 100, 0})  // 40×40 m map, ~5 m patch, full diagonal path
->Args({800, 800, 200, 0})  // 40×40 m map, ~10 m patch, full diagonal path
->Args({1600, 1600, 100, 0})  // 80×80 m map, ~5 m patch, full diagonal path
->Args({1600, 1600, 200, 0})  // 80×80 m map, ~10 m patch, full diagonal path
->Args({3200, 3200, 100, 0})  // 160×160 m map, ~5 m patch, full diagonal path
->Args({3200, 3200, 200, 0})  // 160×160 m map, ~10 m patch, full diagonal path
->Args({6400, 6400, 100, 0})  // 320×320 m map, ~5 m patch, full diagonal path
->Args({6400, 6400, 200, 0})  // 320×320 m map, ~10 m patch, full diagonal path
->Args({12800, 12800, 100, 0})  // 640×640 m map, ~5 m patch, full diagonal path
->Args({12800, 12800, 200, 0})  // 640×640 m map, ~10 m patch, full diagonal path
->Args({25600, 25600, 100, 0})  // 1280×1280 m map, ~5 m patch, full diagonal path
->Args({25600, 25600, 200, 0})  // 1280×1280 m map, ~10 m patch, full diagonal path
->Args({800, 800, 100, 20})  // 40×40 m map, ~5 m patch, fixed 20 m path
->Args({1600, 1600, 100, 20})  // 80×80 m map, ~5 m patch, fixed 20 m path
->Args({3200, 3200, 100, 20})  // 160×160 m map, ~5 m patch, fixed 20 m path
->Args({6400, 6400, 100, 20})  // 320×320 m map, ~5 m patch, fixed 20 m path
->Args({12800, 12800, 100, 20})  // 640×640 m map, ~5 m patch, fixed 20 m path
->Args({25600, 25600, 100, 20})  // 1280×1280 m map, ~5 m patch, fixed 20 m path
->Iterations(50)
->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto ret =
    rcutils_logging_set_logger_level("inflation_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("legacy_inflation_benchmark",
    RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("asymmetric_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("incremental_inflation_benchmark",
    RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("incremental_legacy_inflation_benchmark",
    RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("incremental_asymmetric_benchmark",
    RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("rclcpp_lifecycle", RCUTILS_LOG_SEVERITY_ERROR);
  (void)ret;

  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
    rclcpp::shutdown();
    return 1;
  }
  benchmark::RunSpecifiedBenchmarks();

  rclcpp::shutdown();
  return 0;
}
