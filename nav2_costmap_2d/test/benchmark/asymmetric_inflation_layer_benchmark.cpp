// Copyright 2026 Duatic AG
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
 * @file asymmetric_inflation_layer_benchmark.cpp
 * @brief Performance comparison between LegacyInflationLayer and
 *        AsymmetricInflationLayer::updateCosts() at four costmap sizes
 *        (3×3 m, 5×5 m, 10×10 m, 20×20 m) at 0.05 m/cell resolution.
 *
 * The asymmetric layer is benchmarked with asymmetry fully active
 * (asymmetry_factor=0.5, diagonal path injected), which exercises the full
 * Dial's-algorithm BFS.  Each configuration is run for 50 iterations;
 * the costmap is reset to 10% random rectangular obstacles between each
 * iteration (under timing pause) for consistent per-iteration measurements.
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
 * @brief Subclass that exposes the protected globalPathCallback for benchmarking.
 *
 * Injects a path directly without a ROS publisher / executor spin, avoiding
 * message-passing latency noise in the measurements.
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
 * target cell count is reached.  Adapted from
 * inflation_layer_updatecosts_benchmark.cpp (file-static there, so not linkable).
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

/**
 * @brief Build a 0.6 m × 0.4 m rectangular robot footprint.
 */
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
// LegacyInflationLayer benchmark
// ---------------------------------------------------------------------------

/**
 * @brief Fixture that times LegacyInflationLayer::updateCosts().
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class LegacyInflationLayerFixture : public benchmark::Fixture
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
    node_ = std::make_shared<nav2::LifecycleNode>("legacy_inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<nav2_costmap_2d::LegacyInflationLayer>();
    layer_->initialize(layers_.get(), "inflation", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    // setFootprint after addPlugin so onFootprintChanged() reaches the layer.
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

/**
 * @brief Benchmark body: reset costmap (under pause) then time one updateCosts().
 */
BENCHMARK_DEFINE_F(LegacyInflationLayerFixture, UpdateCosts)(benchmark::State & state)
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

BENCHMARK_REGISTER_F(LegacyInflationLayerFixture, UpdateCosts)
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
// AsymmetricInflationLayer benchmark (asymmetry fully active)
// ---------------------------------------------------------------------------

/**
 * @brief Fixture that times AsymmetricInflationLayer::updateCosts() with a
 *        diagonal path injected so use_asymmetry = true (full BFS runs).
 *
 * This represents the production steady-state: a plan has been published,
 * asymmetry_factor is non-zero, and the BFS executes on every costmap update.
 *
 * Benchmark arguments: {width_cells, height_cells}.
 */
class AsymmetricInflationLayerFixture : public benchmark::Fixture
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
      rclcpp::Parameter("asymmetric_inflation_layer.cost_scaling_factor", kCostScalingFactor),
      rclcpp::Parameter("asymmetric_inflation_layer.asymmetry_factor", kAsymmetryFactor),
      rclcpp::Parameter("asymmetric_inflation_layer.inflate_around_unknown", false),
      rclcpp::Parameter("asymmetric_inflation_layer.goal_distance_threshold", 1.5),
      rclcpp::Parameter("asymmetric_inflation_layer.neutral_threshold", 2.0),
      // Absolute topic so joinWithParentNamespace() leaves it unchanged.
      rclcpp::Parameter(
        "asymmetric_inflation_layer.plan_topic", std::string("/benchmark_asym_plan")),
    });
    node_ = std::make_shared<nav2::LifecycleNode>("asymmetric_inflation_benchmark", "", opts);

    layers_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(kGlobalFrame, false, false);
    layers_->resizeMap(width_, height_, kResolution, 0.0, 0.0);

    layer_ = std::make_shared<BenchmarkAsymmetricInflationLayer>();
    layer_->initialize(layers_.get(), "asymmetric_inflation_layer", nullptr, node_, nullptr);
    layers_->addPlugin(std::static_pointer_cast<nav2_costmap_2d::Layer>(layer_));
    // setFootprint after addPlugin: onFootprintChanged() sets inscribed_radius_
    // before the first updateCosts() call.
    layers_->setFootprint(makeFootprint());

    // Inject a diagonal path spanning the full map so use_asymmetry = true inside
    // updateCosts().  The goal (far corner) is always >> goal_distance_threshold_
    // from the robot at (0, 0).
    injectDiagonalPath();

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
  std::shared_ptr<BenchmarkAsymmetricInflationLayer> layer_;

private:
  void injectDiagonalPath()
  {
    const double width_m = static_cast<double>(width_) * kResolution;
    const double height_m = static_cast<double>(height_) * kResolution;
    const double diag = std::hypot(width_m, height_m);
    const double step = 0.5;  // one pose every 0.5 m along the diagonal
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

    layer_->injectPath(path);
  }
};

/**
 * @brief Benchmark body: reset costmap (under pause) then time one updateCosts().
 */
BENCHMARK_DEFINE_F(AsymmetricInflationLayerFixture, UpdateCosts)(benchmark::State & state)
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

BENCHMARK_REGISTER_F(AsymmetricInflationLayerFixture, UpdateCosts)
->Args({60, 60})      // 3x3 m
->Args({100, 100})    // 5x5 m
->Args({200, 200})    // 10x10 m
->Args({400, 400})    // 20x20 m
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

  // Silence ROS log noise so benchmark output is readable.
  auto ret = rcutils_logging_set_logger_level(
    "legacy_inflation_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
  ret = rcutils_logging_set_logger_level(
    "asymmetric_inflation_benchmark", RCUTILS_LOG_SEVERITY_ERROR);
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
