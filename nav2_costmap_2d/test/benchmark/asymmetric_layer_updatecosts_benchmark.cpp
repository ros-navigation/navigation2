// Copyright (c) 2026
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
 *        realistic chained pipeline.
 *
 * Two fixtures are compared at four costmap sizes (3×3 m, 5×5 m, 10×10 m,
 * 20×20 m) at 0.05 m/cell, 50 iterations each:
 *
 *   LegacyOnlyFixture  — LegacyInflationLayer::updateCosts() alone
 *                        (the current pipeline without the new layer)
 *
 *   ChainedFixture     — LegacyInflationLayer::updateCosts() followed by
 *                        AsymmetricInflationLayer::updateCosts() on the same
 *                        costmap (the full production pipeline with the new layer)
 *
 * The difference between the two reported times is the actual overhead that
 * enabling the asymmetric layer adds to each costmap update cycle.
 * Crucially, the asymmetric BFS operates on the symmetric baseline that
 * LegacyInflationLayer has already written, so only cells on the disfavored
 * side whose asymmetric cost exceeds the symmetric cost are written — matching
 * real production behaviour and giving a tighter bound than testing the layer
 * in isolation.
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
#include "nav2_costmap_2d/legacy_inflation_layer.hpp"
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace
{

static constexpr const char * kGlobalFrame = "map";
static constexpr double kResolution = 0.05;         ///< metres per cell
static constexpr double kInflationRadius = 0.55;    ///< metres
static constexpr double kCostScalingFactor = 10.0;
static constexpr double kAsymmetryFactor = 0.5;     ///< non-zero → full BFS runs
static constexpr double kOccupancy = 0.10;          ///< lethal-obstacle fraction
static constexpr unsigned int kObstacleSeed = 42;

/**
 * @brief Exposes the protected globalPathCallback for path injection in benchmarks.
 */
class BenchmarkAsymmetricInflationLayer : public nav2_costmap_2d::AsymmetricInflationLayer
{
public:
  /** @brief Deliver @p msg as if it arrived on the plan topic. */
  void injectPath(nav_msgs::msg::Path::SharedPtr msg) {globalPathCallback(msg);}
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
  const unsigned int total = size_x * size_y;
  const unsigned int target = static_cast<unsigned int>(total * occupancy_pct);

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

  unsigned int occupied = 0;
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

}  // namespace

// ---------------------------------------------------------------------------
// Baseline: LegacyInflationLayer alone
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for the baseline pipeline: LegacyInflationLayer only.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class LegacyOnlyFixture : public benchmark::Fixture
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
    node_ = std::make_shared<nav2::LifecycleNode>("legacy_only_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::LegacyInflationLayer>();
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
  std::shared_ptr<nav2_costmap_2d::LegacyInflationLayer> layer_;
};

BENCHMARK_DEFINE_F(LegacyOnlyFixture, UpdateCosts)(benchmark::State & state)
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

  state.counters["cells"] = static_cast<double>(width_ * height_);
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(LegacyOnlyFixture, UpdateCosts)
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
// Full pipeline: LegacyInflationLayer + AsymmetricInflationLayer chained
// ---------------------------------------------------------------------------

/**
 * @brief Fixture for the full chained pipeline:
 *        LegacyInflationLayer then AsymmetricInflationLayer on the same costmap.
 *
 * This reproduces the production call sequence.  LegacyInflationLayer writes
 * the symmetric baseline; AsymmetricInflationLayer then raises costs only where
 * the asymmetric effective distance exceeds the existing symmetric cost, i.e.
 * on the disfavoured side of obstacles.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class ChainedFixture : public benchmark::Fixture
{
public:
  void SetUp(benchmark::State & state) override
  {
    width_ = static_cast<unsigned int>(state.range(0));
    height_ = static_cast<unsigned int>(state.range(1));

    // Both layers share one node; parameters are separated by their name prefix.
    auto opts = rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("inflation.inflation_radius", kInflationRadius),
      rclcpp::Parameter("inflation.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("inflation.inflate_unknown", false),
      rclcpp::Parameter("inflation.inflate_around_unknown", false),
      rclcpp::Parameter("asymmetric_inflation_layer.enabled", true),
      rclcpp::Parameter("asymmetric_inflation_layer.inflation_radius", kInflationRadius),
      rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("asymmetric_inflation_layer.asymmetry_factor", kAsymmetryFactor),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_around_unknown", false),
      rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 1.5),
      rclcpp::Parameter("asymmetric_inflation_layer.neutral_threshold", 2.0),
      rclcpp::Parameter(
        "asymmetric_inflation_layer.plan_topic", std::string("/benchmark_chain_plan")),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("chained_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    legacy_layer_ = std::make_shared<nav2_costmap_2d::LegacyInflationLayer>();
    legacy_layer_->initialize(layers_.get(), "inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(legacy_layer_));

    asym_layer_ = std::make_shared<BenchmarkAsymmetricInflationLayer>();
    asym_layer_->initialize(
      layers_.get(), "asymmetric_inflation_layer", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(asym_layer_));

    // setFootprint after both addPlugin calls so onFootprintChanged() reaches both layers.
    layers_->setFootprint(makeFootprint());

    injectDiagonalPath();
    generateRectangularObstacles(*layers_->getCostmap(), kOccupancy);
  }

  void TearDown(benchmark::State &) override
  {
    asym_layer_.reset();
    legacy_layer_.reset();
    layers_.reset();
    node_.reset();
  }

  unsigned int width_{0};
  unsigned int height_{0};
  nav2::LifecycleNode::SharedPtr node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::LegacyInflationLayer> legacy_layer_;
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

BENCHMARK_DEFINE_F(ChainedFixture, UpdateCosts)(benchmark::State & state)
{
  auto * costmap = layers_->getCostmap();
  const int w = static_cast<int>(width_);
  const int h = static_cast<int>(height_);

  for (auto _ : state) {
    state.PauseTiming();
    generateRectangularObstacles(*costmap, kOccupancy);
    state.ResumeTiming();

    // Production call order: symmetric baseline first, asymmetric bumps second.
    legacy_layer_->updateCosts(*costmap, 0, 0, w, h);
    asym_layer_->updateCosts(*costmap, 0, 0, w, h);
  }

  state.counters["cells"] = static_cast<double>(width_ * height_);
  state.counters["map_side_m"] = static_cast<double>(width_) * kResolution;
}

BENCHMARK_REGISTER_F(ChainedFixture, UpdateCosts)
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
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto ret =
    rcutils_logging_set_logger_level("legacy_only_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level("chained_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
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
